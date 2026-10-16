"""Paralel SAC eğitimi: N TORCS ortamı veri toplar, GPU öğrenir.

    python racelab/rl/train_sac.py --name sac_v1 --workers 8 --hours 1

Çıktılar: racelab/rl/runs/<name>/
    progress.csv     — zaman, adım, episode istatistikleri, kayıplar
    episodes.csv     — her episode (eval dahil): mesafe, hasar, tur süresi, sebep
    actor_latest.pt  — worker'ların kullandığı güncel politika
    best.pt          — en iyi deterministik (eval) koşudaki politika
    config.json      — eğitim ayarları

Worker 0 her `--eval-every` episode'da bir deterministik (gürültüsüz) tur atar;
"en iyi" seçimi sadece bu eval turlarına göre yapılır.
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
    """Büyük = iyi. Bitiren > bitirmeyen; bitirende hasarsız ve kısa tur iyi."""
    if ep["lap_time"]:
        return (2 if ep["damage"] == 0 else 1, -ep["lap_time"], 0)
    return (0, 0, ep["distance"])


# ------------------------------------------------------------------- worker
def worker(wid, args, run_dir, tq, sq, stop_ev):
    import torch
    torch.set_num_threads(1)
    from sac import Actor
    from torcs_env import TorcsEnv
    from common import OBS_DIM, ACT_DIM

    env = TorcsEnv(idx=args.idx_offset + wid, track=args.track, max_steps=args.max_steps)
    actor = Actor(OBS_DIM, ACT_DIM, tuple(args.hidden))
    latest = os.path.join(run_dir, "actor_latest.pt")
    version, ep_count = -1, 0
    rng = np.random.default_rng(args.seed + wid)

    while not stop_ev.is_set():
        if os.path.exists(latest):
            try:
                with open(latest, "rb") as f:  # dosyayı hızlıca oku-bırak (Windows kilidi)
                    raw = f.read()
                ck = torch.load(io.BytesIO(raw), map_location="cpu")
                if ck["version"] != version:
                    actor.load_state_dict(ck["actor"])
                    version = ck["version"]
            except Exception:
                pass  # yazılırken okunduysa bir sonraki episode'da tekrar dene
        is_eval = wid == 0 and version >= 0 and ep_count % args.eval_every == 0
        warmup = version < 0
        o = env.reset()
        buf = []
        steps, t0 = 0, time.time()
        while True:
            if warmup:
                # rastgele ama yolda kalmaya meyilli ısınma: direksiyon pist eksenine
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
        sq.put({"worker": wid, "eval": is_eval, "version": version, "steps": steps,
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
    ap.add_argument("--name", default="sac_v1")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--idx-offset", type=int, default=1,
                    help="ilk worker'ın scr_server idx'i (port=3001+idx). idx 0 GUI/benchmark'a ayrılı; "
                         "idx en fazla 9 → paralel deneylerde aralıklar çakışmasın")
    ap.add_argument("--hours", type=float, default=1.0)
    ap.add_argument("--max-env-steps", type=int, default=20_000_000)
    ap.add_argument("--track", default="corkscrew")
    ap.add_argument("--max-steps", type=int, default=12000, help="episode başına adım (50/s)")
    ap.add_argument("--warmup", type=int, default=20000, help="öğrenme başlamadan önce toplanan adım")
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--graph-k", type=int, default=8, help="CUDA Graph çağrısı başına güncelleme")
    ap.add_argument("--utd", type=float, default=0.25, help="toplanan adım başına hedef güncelleme sayısı")
    ap.add_argument("--buffer", type=int, default=2_000_000)
    ap.add_argument("--hidden", type=int, nargs="+", default=[256, 256])
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--eval-every", type=int, default=5)
    ap.add_argument("--publish-every", type=int, default=500, help="kaç güncellemede bir worker'lara politika")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    assert 1 <= args.idx_offset and args.idx_offset + args.workers - 1 <= 9, "scr_server idx 1..9 olmalı"

    import torch
    from sac import SAC, ReplayBuffer
    from common import OBS_DIM, ACT_DIM

    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run_dir = os.path.join(HERE, "runs", args.name)
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "config.json"), "w") as f:
        json.dump(dict(vars(args), device=str(device)), f, indent=2)
    latest = os.path.join(run_dir, "actor_latest.pt")
    if os.path.exists(latest):
        os.remove(latest)  # worker'lar ısınma politikasıyla başlasın

    agent = SAC(OBS_DIM, ACT_DIM, device, tuple(args.hidden), lr=args.lr, gamma=args.gamma)
    rb = ReplayBuffer(OBS_DIM, ACT_DIM, args.buffer, device)

    ctx = mp.get_context("spawn")
    tq, sq, stop_ev = ctx.Queue(maxsize=2000), ctx.Queue(), ctx.Event()
    procs = [ctx.Process(target=worker, args=(w, args, run_dir, tq, sq, stop_ev), daemon=True)
             for w in range(args.workers)]
    for p in procs:
        p.start()

    prog_f = open(os.path.join(run_dir, "progress.csv"), "w", newline="")
    prog = csv.writer(prog_f)
    prog.writerow(["wall_min", "env_steps", "updates", "episodes", "train_ret_avg20", "train_dist_avg20",
                   "best_eval_dist", "best_eval_lap", "last_eval_dist", "last_eval_lap", "alpha",
                   "loss_q", "loss_pi", "q_mean", "env_sps", "upd_ps"])
    ep_f = open(os.path.join(run_dir, "episodes.csv"), "w", newline="")
    ep_w = csv.DictWriter(ep_f, fieldnames=["wall_min", "worker", "eval", "version", "steps", "return",
                                            "distance", "damage", "lap_time", "reason", "sps"])
    ep_w.writeheader()

    t_start = time.time()
    env_steps = updates = episodes = 0
    version = -1
    recent = []
    best, last_eval = None, None
    losses = (0, 0, agent.alpha.item(), 0)
    graphed, loss_t = None, None
    last_log, steps_at_log, upd_at_log = time.time(), 0, 0

    def publish():
        nonlocal version
        version += 1
        tmp = latest + ".tmp"
        torch.save({"actor": {k: v.cpu() for k, v in agent.actor.state_dict().items()},
                    "version": version, "hidden": args.hidden}, tmp)
        for _ in range(50):  # Windows: worker dosyayı okurken değiştirilemez, tekrar dene
            try:
                os.replace(tmp, latest)
                return
            except PermissionError:
                time.sleep(0.01)
        version -= 1  # bu yayın atlandı

    try:
        while time.time() - t_start < args.hours * 3600 and env_steps < args.max_env_steps:
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
                if ep["eval"]:
                    last_eval = ep
                    if best is None or eval_key(ep) > eval_key(best):
                        best = ep
                        torch.save({"actor": {k: v.cpu() for k, v in agent.actor.state_dict().items()},
                                    "hidden": args.hidden, "eval": ep, "env_steps": env_steps},
                                   os.path.join(run_dir, "best.pt"))
                        # best.pt anlık ağırlıklar; eval'i yapan versiyon bir önceki yayın olabilir
                        print(f"[best] dist={ep['distance']} lap={ep['lap_time']} dmg={ep['damage']} "
                              f"v={ep['version']} steps={env_steps}", flush=True)
                else:
                    recent = (recent + [ep])[-20:]

            if rb.n >= args.warmup:
                if version < 0:
                    publish()
                    if device.type == "cuda":
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

            if time.time() - last_log > 15:
                if loss_t is not None:
                    losses = tuple(loss_t.tolist())
                dt = time.time() - last_log
                ra = np.mean([e["return"] for e in recent]) if recent else 0
                rd = np.mean([e["distance"] for e in recent]) if recent else 0
                row = [round((time.time() - t_start) / 60, 2), env_steps, updates, episodes, round(ra, 1),
                       round(rd, 1), best and best["distance"], best and best["lap_time"],
                       last_eval and last_eval["distance"], last_eval and last_eval["lap_time"],
                       round(losses[2], 4), round(losses[0], 3), round(losses[1], 3), round(losses[3], 2),
                       round((env_steps - steps_at_log) / dt), round((updates - upd_at_log) / dt)]
                prog.writerow(row)
                prog_f.flush()
                ep_f.flush()
                print(f"{row[0]:7.1f}m steps={env_steps:>9} upd={updates:>8} eps={episodes:>6} "
                      f"dist20={rd:7.1f} best={best and best['distance']}/{best and best['lap_time']} "
                      f"alpha={losses[2]:.3f} env/s={row[14]} upd/s={row[15]}", flush=True)
                last_log, steps_at_log, upd_at_log = time.time(), env_steps, updates
    finally:
        torch.save(agent.state_dict(), os.path.join(run_dir, "final_full.pt"))
        torch.save({"actor": {k: v.cpu() for k, v in agent.actor.state_dict().items()}, "hidden": args.hidden},
                   os.path.join(run_dir, "final.pt"))
        stop_ev.set()
        t_stop = time.time()
        for p in procs:
            while p.is_alive() and time.time() - t_stop < 15:
                try:
                    tq.get(timeout=0.1)  # dolu kuyruk worker'ı bloklamasın
                except queue.Empty:
                    pass
            if p.is_alive():
                p.terminate()
        from torcs_env import kill_training_torcs
        kill_training_torcs()  # yedek temizlik; elle açılmış GUI TORCS'a dokunmaz
        prog_f.close()
        ep_f.close()
        print("bitti:", run_dir)


if __name__ == "__main__":
    main()
