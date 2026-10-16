"""Paralel SAC eğitimi v2: doğru model kaydı + sağlamlık + çarpmama odaklı ödül.

    python racelab/rl/train_sac_v2.py --name v2_fresh --workers 4 --idx-offset 1 --hours 2
    python racelab/rl/train_sac_v2.py --name v2_ft --init racelab/rl/runs/sac_g995/final_full.pt --idx-offset 5

v1'den (train_sac.py) farklar
- Ortam: torcs_env_v2.TorcsEnvV2 (güvenlik payı cezası, hıza bağlı ceza, adım düşürme + gözlem gürültüsü).
- Öğrenci her yayınlanan politika sürümünün ağırlıklarını hafızada tutar; eval sonucu gelince
  **eval'i yapan sürümün** ağırlıkları kaydedilir (v1'deki best.pt hatası).
- Worker 0 eval'leri dönüşümlü: "clean" (pertürbasyon yok) ve "perturbed" (adım düşürme açık).
  Hasarsız tur atan her eval → candidates/ altında aday model. Seçim: select_models.py.
- --init ile önceki bir eğitimin actor+critic ağırlıklarından devam edilebilir.
- Her --snapshot-min dakikada bir snapshots/ altına güncel politika.
- Güvenli erken durdurma: runs/<name>/STOP dosyası oluştur → final.pt kaydedilip kapanır.

Çıktılar: racelab/rl/runs/<name>/  progress.csv, episodes.csv, best.pt, final.pt, final_full.pt,
          candidates/*.pt, snapshots/*.pt, config.json
"""
import argparse
import csv
import io
import json
import multiprocessing as mp
import os
import queue
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def eval_key(ep):
    """Büyük = iyi. Hasarsız bitiren > hasarlı bitiren > bitirmeyen (mesafeye göre)."""
    if ep["lap_time"]:
        return (2 if ep["damage"] == 0 else 1, -ep["lap_time"], 0)
    return (0, 0, ep["distance"])


# ------------------------------------------------------------------- worker
def worker(wid, args, run_dir, tq, sq, stop_ev):
    import torch
    torch.set_num_threads(1)
    from sac import Actor
    from common import OBS_DIM, OBS_DIM_POS, OBS_DIM_POS2, ACT_DIM
    OBS_DIM = OBS_DIM_POS2 if args.pos_obs2 else (OBS_DIM_POS if args.pos_obs else OBS_DIM)

    kw = dict(idx=args.idx_offset + wid, track=args.track, max_steps=args.max_steps,
              drop_prob=args.drop_prob, obs_noise=args.obs_noise, seed=args.seed + wid,
              tcs_threshold=args.tcs_th, safe_pos=args.safe_pos,
              pedal_gain=args.pedal_gain, pedal_bias=args.pedal_bias,
              assist_front=args.assist_front, upshift=args.upshift,
              pos_obs=2 if args.pos_obs2 else args.pos_obs)
    if args.env == "v4":
        from torcs_env_v4 import TorcsEnvV4
        env = TorcsEnvV4(c_change=args.c_change, c_flip=args.c_flip, c_mag=args.c_mag, c_speed=args.c_speed, **kw)
    elif args.env == "v3":
        from torcs_env_v3 import TorcsEnvV3
        env = TorcsEnvV3(steer_change_pen=args.steer_change_pen, steer_mag_pen=args.steer_mag_pen, **kw)
    else:
        from torcs_env_v2 import TorcsEnvV2
        env = TorcsEnvV2(**kw)
    actor = Actor(OBS_DIM, ACT_DIM, tuple(args.hidden))
    latest = os.path.join(run_dir, "actor_latest.pt")
    version, ep_count, n_eval = -1, 0, 0
    rng = np.random.default_rng(args.seed + wid)

    while not stop_ev.is_set():
        if os.path.exists(latest):
            try:
                with open(latest, "rb") as f:  # hızlıca oku-bırak (Windows dosya kilidi)
                    raw = f.read()
                ck = torch.load(io.BytesIO(raw), map_location="cpu")
                if ck["version"] != version:
                    actor.load_state_dict(ck["actor"])
                    version = ck["version"]
            except Exception:
                pass
        is_eval = wid == 0 and version >= 0 and ep_count % args.eval_every == 0
        kind = "train"
        if is_eval:
            kind = "clean" if n_eval % 2 == 0 else "perturbed"
            n_eval += 1
            env.drop_prob = 0.0 if kind == "clean" else args.eval_drop_prob
            env.obs_noise = 0.0
        else:
            env.drop_prob, env.obs_noise = args.drop_prob, args.obs_noise
        warmup = version < 0
        o = env.reset()
        buf = []
        steps, t0 = 0, time.time()
        while True:
            if warmup:
                a = np.array([o[0] * 3 - o[1] * 0.5 + rng.normal(0, 0.3), rng.uniform(-0.2, 1.0)],
                             dtype=np.float32)
                a = np.clip(a, -1, 1)
            else:
                with torch.no_grad():
                    a, _ = actor(torch.as_tensor(o).unsqueeze(0), deterministic=is_eval, with_logp=False)
                a = a.squeeze(0).numpy()
            o2, r, d, info = env.step(a)
            steps += 1
            if not is_eval:
                term = float(d and not info.get("truncated", False))
                buf.append((o, a, r, o2, term))
                if len(buf) >= 256:
                    tq.put(_pack(buf))
                    buf = []
            o = o2
            if d:
                break
            if stop_ev.is_set():
                env.close()
                return
        if buf:
            tq.put(_pack(buf))
        ep_count += 1
        reason = ("lap" if info.get("lap_time") else "crash" if info["crash"] else "offtrack" if info["offtrack"]
                  else "stuck" if info["stuck"] else "truncated")
        sq.put({"worker": wid, "eval": is_eval, "kind": kind, "version": version, "steps": steps,
                "return": round(env.ep_return, 2), "distance": round(info.get("distance", 0), 1),
                "damage": info.get("damage", 0), "lap_time": info.get("lap_time"), "reason": reason,
                "sps": round(steps / max(time.time() - t0, 1e-6))})
    env.close()


def _pack(buf):
    o, a, r, o2, d = zip(*buf)
    return (np.asarray(o, np.float32), np.asarray(a, np.float32), np.asarray(r, np.float32),
            np.asarray(o2, np.float32), np.asarray(d, np.float32))


# ------------------------------------------------------------------- learner
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="v2")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--idx-offset", type=int, default=1,
                    help="ilk worker'ın scr_server idx'i (port=3001+idx); idx 0 GUI/benchmark'a ayrılı")
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--max-env-steps", type=int, default=100_000_000)
    ap.add_argument("--track", default="corkscrew")
    ap.add_argument("--max-steps", type=int, default=12000)
    ap.add_argument("--init", default=None, help="final_full.pt: actor+critic'ten devam")
    ap.add_argument("--warmup", type=int, default=20000)
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--graph-k", type=int, default=8)
    ap.add_argument("--utd", type=float, default=0.25)
    ap.add_argument("--buffer", type=int, default=2_000_000)
    ap.add_argument("--hidden", type=int, nargs="+", default=[256, 256])
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.995)
    ap.add_argument("--drop-prob", type=float, default=0.05, help="eğitimde komut düşürme olasılığı")
    ap.add_argument("--obs-noise", type=float, default=0.01, help="eğitimde gözlem gürültüsü (std)")
    ap.add_argument("--eval-drop-prob", type=float, default=0.05, help="'perturbed' eval'deki düşürme")
    ap.add_argument("--eval-every", type=int, default=3)
    ap.add_argument("--publish-every", type=int, default=500)
    ap.add_argument("--keep-versions", type=int, default=200, help="hafızada tutulan son politika sürümü")
    ap.add_argument("--snapshot-min", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--env", choices=["v2", "v3", "v4"], default="v2",
                    help="v3: sert direksiyon hız sınırı (başarısız); v4: yumuşak zikzak cezaları")
    ap.add_argument("--c-change", type=float, default=0.3, help="(v4) |Δsteer|×v/100 cezası")
    ap.add_argument("--c-flip", type=float, default=0.2, help="(v4) yön değiştirme × v/100 cezası")
    ap.add_argument("--c-mag", type=float, default=0.3, help="(v4) steer²×düzlük×v/100 cezası")
    ap.add_argument("--c-speed", type=float, default=0.0, help="(v4) ek ilerleme ödülü (m başına)")
    ap.add_argument("--tcs-th", type=float, default=2.0, help="patinaj kontrolü eşiği m/s (v1-v4: 2.0, v6: 5.0)")
    ap.add_argument("--safe-pos", type=float, default=0.75, help="|trackPos| bu değeri aşınca kenar cezası")
    ap.add_argument("--pedal-gain", type=float, default=1.0, help="v7: 1.3333")
    ap.add_argument("--pedal-bias", type=float, default=0.0, help="v7: 0.3333")
    ap.add_argument("--assist-front", type=float, default=0.0, help="v8 düzlük gaz yardımcısı (m), 0=kapalı")
    ap.add_argument("--upshift", type=float, default=None, help="vites büyütme devri (varsayılan 18000)")
    ap.add_argument("--init-actor", default=None, help="v9: sadece aktör ağırlıklarından başla (aday .pt)")
    ap.add_argument("--critic-warmup", type=int, default=0, help="v9: aktör sabitken yapılacak critic güncellemesi")
    ap.add_argument("--init-alpha", type=float, default=None, help="v9: başlangıç entropi katsayısı")
    ap.add_argument("--actor-lr", type=float, default=None, help="v9: aktör öğrenme hızı (varsayılan --lr)")
    ap.add_argument("--fixed-alpha", action="store_true", help="v9: alpha'yı --init-alpha değerinde sabitle")
    ap.add_argument("--pos-obs", action="store_true", help="v10: gözleme pist konumu harmonikleri (32→44)")
    ap.add_argument("--pos-obs2", action="store_true", help="D1: harmonikler k=64'e kadar (32→46)")
    ap.add_argument("--steer-change-pen", type=float, default=0.3, help="(v3) |Δsteer| cezası")
    ap.add_argument("--steer-mag-pen", type=float, default=0.05, help="(v3) steer² × hız/100 cezası")
    args = ap.parse_args()
    assert 1 <= args.idx_offset and args.idx_offset + args.workers - 1 <= 9, "scr_server idx 1..9 olmalı"

    import torch
    from sac import SAC, ReplayBuffer, expand_obs_inputs
    from common import OBS_DIM, OBS_DIM_POS, OBS_DIM_POS2, ACT_DIM
    BASE_OBS = OBS_DIM
    OBS_DIM = OBS_DIM_POS2 if args.pos_obs2 else (OBS_DIM_POS if args.pos_obs else OBS_DIM)

    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run_dir = os.path.join(HERE, "runs", args.name)
    cand_dir = os.path.join(run_dir, "candidates")
    snap_dir = os.path.join(run_dir, "snapshots")
    for d in (run_dir, cand_dir, snap_dir):
        os.makedirs(d, exist_ok=True)
    with open(os.path.join(run_dir, "config.json"), "w") as f:
        json.dump(dict(vars(args), device=str(device)), f, indent=2)
    latest = os.path.join(run_dir, "actor_latest.pt")
    if os.path.exists(latest):
        os.remove(latest)

    agent = SAC(OBS_DIM, ACT_DIM, device, tuple(args.hidden), lr=args.lr, gamma=args.gamma)
    if args.init:
        ck = torch.load(args.init, map_location=device, weights_only=False)
        if ck["actor"]["net.0.weight"].shape[1] == BASE_OBS and OBS_DIM != BASE_OBS:
            ck = expand_obs_inputs(ck, BASE_OBS, OBS_DIM, ACT_DIM)  # v10: 32→44, yeni girişler sıfır
            print(f"init genişletildi: {BASE_OBS} → {OBS_DIM} giriş", flush=True)
        agent.actor.load_state_dict(ck["actor"])
        agent.critic.load_state_dict(ck["critic"])
        agent.critic_targ.load_state_dict(ck["critic_targ"])
        with torch.no_grad():
            agent.log_alpha.copy_(ck["log_alpha"].to(device))
        print(f"init: {args.init}", flush=True)
    if args.init_actor:  # v9: sadece aktör (ör. bir adayın ağırlıkları); critic sıfırdan, önce ısıtılır
        ck = torch.load(args.init_actor, map_location=device, weights_only=False)
        agent.actor.load_state_dict(ck["actor"])
        print(f"init-actor: {args.init_actor}", flush=True)
    if args.actor_lr is not None:  # v9: aktör critic'ten çok daha yavaş öğrensin
        for g in agent.opt_actor.param_groups:
            g["lr"] = args.actor_lr
    if args.fixed_alpha:  # v9: alpha sabit (otomatik entropi ayarı kararlı aktörü gürültüye zorluyordu)
        for g in agent.opt_alpha.param_groups:
            g["lr"] = 0.0
    if args.init_alpha is not None:
        with torch.no_grad():
            agent.log_alpha.fill_(float(np.log(args.init_alpha)))
    rb = ReplayBuffer(OBS_DIM, ACT_DIM, args.buffer, device)

    ctx = mp.get_context("spawn")
    tq, sq, stop_ev = ctx.Queue(maxsize=2000), ctx.Queue(), ctx.Event()
    procs = [ctx.Process(target=worker, args=(w, args, run_dir, tq, sq, stop_ev), daemon=True)
             for w in range(args.workers)]

    prog_f = open(os.path.join(run_dir, "progress.csv"), "w", newline="")
    prog = csv.writer(prog_f)
    prog.writerow(["wall_min", "env_steps", "updates", "episodes", "train_ret_avg20", "train_dist_avg20",
                   "train_lap_rate20", "best_eval_dist", "best_eval_lap", "n_candidates", "alpha",
                   "loss_q", "loss_pi", "q_mean", "env_sps", "upd_ps"])
    ep_f = open(os.path.join(run_dir, "episodes.csv"), "w", newline="")
    ep_w = csv.DictWriter(ep_f, fieldnames=["wall_min", "worker", "eval", "kind", "version", "steps", "return",
                                            "distance", "damage", "lap_time", "reason", "sps"])
    ep_w.writeheader()

    t_start = time.time()
    env_steps = updates = episodes = n_cand = 0
    version = -1
    versions = {}  # sürüm → CPU'daki actor ağırlıkları
    recent = []
    best = None
    losses = (0, 0, agent.alpha.item(), 0)
    graphed, loss_t = None, None
    last_log, steps_at_log, upd_at_log = time.time(), 0, 0
    last_snap = time.time()

    def actor_cpu():
        return {k: v.detach().cpu().clone() for k, v in agent.actor.state_dict().items()}

    def save_actor(path, weights, **meta):
        torch.save(dict({"actor": weights, "hidden": args.hidden}, **meta), path)

    def publish():
        nonlocal version
        version += 1
        w = actor_cpu()
        tmp = latest + ".tmp"
        torch.save({"actor": w, "version": version, "hidden": args.hidden}, tmp)
        for _ in range(50):
            try:
                os.replace(tmp, latest)
                versions[version] = w
                for old in [v for v in versions if v <= version - args.keep_versions]:
                    del versions[old]
                return
            except PermissionError:
                time.sleep(0.01)
        version -= 1

    if args.init or args.init_actor:
        publish()  # worker'lar ısınma yerine yüklenen politikayla başlasın
    for p in procs:
        p.start()

    try:
        stop_file = os.path.join(run_dir, "STOP")  # bu dosya oluşturulursa her şeyi kaydedip kapanır
        if os.path.exists(stop_file):
            os.remove(stop_file)
        while (time.time() - t_start < args.hours * 3600 and env_steps < args.max_env_steps
               and not os.path.exists(stop_file)):
            got = 0
            while True:
                try:
                    o, a, r, o2, d = tq.get_nowait()
                except queue.Empty:
                    break
                rb.add_batch(o, a, r, o2, d)
                env_steps += len(r)
                got += 1
            while True:
                try:
                    ep = sq.get_nowait()
                except queue.Empty:
                    break
                episodes += 1
                ep["wall_min"] = round((time.time() - t_start) / 60, 2)
                ep_w.writerow(ep)
                if not ep["eval"]:
                    recent = (recent + [ep])[-20:]
                    continue
                w = versions.get(ep["version"])
                if w is None:
                    continue  # sürüm hafızadan düşmüş (çok eski)
                meta = {"eval": ep, "version": ep["version"], "env_steps": env_steps}
                if ep["lap_time"] and ep["damage"] == 0:
                    n_cand += 1
                    save_actor(os.path.join(cand_dir, f"v{ep['version']:05d}_{ep['kind']}_{ep['lap_time']:.2f}.pt"),
                               w, **meta)
                if best is None or eval_key(ep) > eval_key(best):
                    best = ep
                    save_actor(os.path.join(run_dir, "best.pt"), w, **meta)
                    print(f"[best] {ep['kind']} dist={ep['distance']} lap={ep['lap_time']} dmg={ep['damage']} "
                          f"v={ep['version']} steps={env_steps}", flush=True)

            if rb.n >= args.warmup:
                if version < 0:
                    publish()
                if updates < args.critic_warmup:  # v9: aktör sabit, critic ısınıyor (yayın yok)
                    n_upd = min(int(env_steps * args.utd) - updates, 200)
                    for _ in range(max(n_upd, 0)):
                        loss_t = torch.stack([agent.update_critic_only(*rb.sample(args.batch)),
                                              torch.zeros((), device=device), agent.alpha.detach(),
                                              torch.zeros((), device=device)])
                        updates += 1
                    if n_upd <= 0:
                        time.sleep(0.005)
                    continue
                if graphed is None and device.type == "cuda":
                    graphed = agent.make_graphed_update(rb, args.batch, args.graph_k)
                n_upd = int(env_steps * args.utd) - updates
                for _ in range(min(max(n_upd, 0), 400) // args.graph_k if graphed else min(max(n_upd, 0), 200)):
                    if graphed:
                        loss_t = graphed()
                        k = args.graph_k
                    else:
                        losses = agent.update(*rb.sample(args.batch))
                        k = 1
                    if (updates + k) // args.publish_every > updates // args.publish_every:
                        publish()
                    updates += k
                if n_upd <= 0 and got == 0:
                    time.sleep(0.005)
            else:
                time.sleep(0.05)

            if time.time() - last_snap > args.snapshot_min * 60 and version >= 0:
                m = round((time.time() - t_start) / 60)
                save_actor(os.path.join(snap_dir, f"{m:04d}min_v{version:05d}.pt"), versions[version],
                           version=version, env_steps=env_steps)
                last_snap = time.time()

            if time.time() - last_log > 15:
                if loss_t is not None:
                    losses = tuple(loss_t.tolist())
                dt = time.time() - last_log
                ra = np.mean([e["return"] for e in recent]) if recent else 0
                rd = np.mean([e["distance"] for e in recent]) if recent else 0
                lr20 = np.mean([1.0 if e["lap_time"] else 0.0 for e in recent]) if recent else 0
                row = [round((time.time() - t_start) / 60, 2), env_steps, updates, episodes, round(ra, 1),
                       round(rd, 1), round(lr20, 2), best and best["distance"], best and best["lap_time"], n_cand,
                       round(losses[2], 4), round(losses[0], 3), round(losses[1], 3), round(losses[3], 2),
                       round((env_steps - steps_at_log) / dt), round((updates - upd_at_log) / dt)]
                prog.writerow(row)
                prog_f.flush()
                ep_f.flush()
                print(f"{row[0]:7.1f}m steps={env_steps:>9} eps={episodes:>6} dist20={rd:7.1f} lap20={lr20:.2f} "
                      f"best={best and best['distance']}/{best and best['lap_time']} aday={n_cand} "
                      f"alpha={losses[2]:.3f} env/s={row[14]} upd/s={row[15]}", flush=True)
                last_log, steps_at_log, upd_at_log = time.time(), env_steps, updates
    finally:
        torch.save(agent.state_dict(), os.path.join(run_dir, "final_full.pt"))
        save_actor(os.path.join(run_dir, "final.pt"), actor_cpu(), version=version)
        stop_ev.set()
        t_stop = time.time()
        for p in procs:
            while p.is_alive() and time.time() - t_stop < 15:
                try:
                    tq.get(timeout=0.1)
                except queue.Empty:
                    pass
            if p.is_alive():
                p.terminate()
        from torcs_env import kill_training_torcs
        kill_training_torcs(range(args.idx_offset, args.idx_offset + args.workers))
        prog_f.close()
        ep_f.close()
        print("bitti:", run_dir)


if __name__ == "__main__":
    main()
