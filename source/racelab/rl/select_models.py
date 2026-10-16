"""Aday modelleri yarış koşulunda tekrar tekrar koşturup en güvenilirini seçer.

    python racelab/rl/select_models.py --run v2_fresh --top 20 --repeat 10
    python racelab/rl/select_models.py --models a.pt b.pt --repeat 10

Her model `--repeat` kez, TORCS'un GUI'deki 10 ms bekleme süresiyle (`-t 10000`) ve
erken bitirme olmadan (yarış TORCS kapatana kadar) koşturulur. Sıralama:
  1) hasarsız bitirme oranı  2) bitirme oranı  3) ortalama tur süresi
Sonuçlar: runs/<run>/selection.csv (ya da --out). Skor tablosuna yazmaz; seçilen modeli
run_benchmark.py ile resmi olarak kaydet.

Paralel değerlendirme scr_server idx --idx-from.. aralığını kullanır (varsayılan 5..8);
eğitimle aynı anda çalışacaksa çakışmayan bir aralık seç.
"""
import argparse
import csv
import glob
import multiprocessing as mp
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def run_model(job):
    path, idx, repeat, udp_timeout, env_ver, tcs_th, pg, pb, af, up = job
    import torch
    torch.set_num_threads(1)
    from common import ACT_DIM, OBS_DIM, pos_mode_for
    from sac import Actor
    from torcs_env_v2 import TorcsEnvV2
    from torcs_env_v3 import TorcsEnvV3
    from torcs_env_v4 import TorcsEnvV4

    ck = torch.load(path, map_location="cpu", weights_only=False)
    obs_dim = ck["actor"]["net.0.weight"].shape[1]  # 32 klasik, 44 konum bilen (v10)
    actor = Actor(obs_dim, ACT_DIM, tuple(ck.get("hidden", [256, 256])))
    actor.load_state_dict(ck["actor"])
    Env = {"v3": TorcsEnvV3, "v4": TorcsEnvV4}.get(env_ver, TorcsEnvV2)
    env = Env(idx=idx, race_mode=True, udp_timeout_us=udp_timeout, max_steps=30000, tcs_threshold=tcs_th,
              pedal_gain=pg, pedal_bias=pb, assist_front=af, upshift=up, pos_obs=pos_mode_for(obs_dim))
    res = []
    for _ in range(repeat):
        o = env.reset()
        while True:
            with torch.no_grad():
                a, _ = actor(torch.as_tensor(o).unsqueeze(0), deterministic=True, with_logp=False)
            o, r, d, info = env.step(a.squeeze(0).numpy())
            if d:
                break
        res.append({"lap": info.get("lap_time"), "damage": info.get("damage", 0),
                    "hits": info.get("hits", 0), "dist": info.get("distance", 0),
                    "off_s": info.get("offtrack_steps", 0) * 0.02})
    env.close()
    laps = [x["lap"] for x in res if x["lap"]]
    clean = [x for x in res if x["lap"] and x["damage"] == 0]
    return {
        "model": os.path.relpath(path, os.path.dirname(os.path.dirname(HERE))).replace("\\", "/"),
        "runs": repeat,
        "clean_finish_rate": round(len(clean) / repeat, 2),
        "finish_rate": round(len(laps) / repeat, 2),
        "mean_lap": round(float(np.mean(laps)), 3) if laps else None,
        "best_lap": round(min(laps), 3) if laps else None,
        "mean_clean_lap": round(float(np.mean([x["lap"] for x in clean])), 3) if clean else None,
        "mean_damage": round(float(np.mean([x["damage"] for x in res])), 1),
        "mean_hits": round(float(np.mean([x["hits"] for x in res])), 2),
        "mean_dist": round(float(np.mean([x["dist"] for x in res])), 1),
        "mean_offtrack_s": round(float(np.mean([x["off_s"] for x in res])), 2),
    }


def rank_key(r):
    return (-r["clean_finish_rate"], -r["finish_rate"], r["mean_clean_lap"] or r["mean_lap"] or 1e9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", help="runs/<run> altındaki candidates + snapshots + best/final")
    ap.add_argument("--models", nargs="*", default=[])
    ap.add_argument("--top", type=int, default=20, help="en hızlı eval turuna sahip kaç aday")
    ap.add_argument("--repeat", type=int, default=10)
    ap.add_argument("--udp-timeout-us", type=int, default=10000)
    ap.add_argument("--idx-from", type=int, default=5)
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--out", default=None)
    ap.add_argument("--env", choices=["v2", "v3", "v4"], default="v2", help="modelin eğitildiği ortam sürümü")
    ap.add_argument("--tcs-th", type=float, default=2.0, help="modelin eğitildiği TCS eşiği (v6 modelleri: 5.0)")
    ap.add_argument("--pedal-gain", type=float, default=1.0)
    ap.add_argument("--pedal-bias", type=float, default=0.0)
    ap.add_argument("--assist-front", type=float, default=0.0)
    ap.add_argument("--upshift", type=float, default=None)
    args = ap.parse_args()
    assert 0 <= args.idx_from and args.idx_from + args.procs - 1 <= 9, "scr_server idx 0..9 olmalı (port 3001..3010)"

    models = [os.path.abspath(m) for m in args.models]
    if args.run:
        rd = os.path.join(HERE, "runs", args.run)
        cands = glob.glob(os.path.join(rd, "candidates", "*.pt"))
        lap = lambda p: float(re.search(r"_(\d+\.\d+)\.pt$", p).group(1))  # noqa: E731
        # perturbed eval'de bitirenler daha sağlam → önce onlar, sonra tur süresi
        cands.sort(key=lambda p: ("_clean_" in p, lap(p)))
        models += cands[:args.top]
        models += sorted(glob.glob(os.path.join(rd, "snapshots", "*.pt")))[-3:]
        models += [p for p in (os.path.join(rd, "best.pt"), os.path.join(rd, "final.pt")) if os.path.exists(p)]
    models = list(dict.fromkeys(models))
    if not models:
        sys.exit("değerlendirilecek model yok")
    out = args.out or os.path.join(HERE, "runs", args.run or ".", "selection.csv")
    print(f"{len(models)} model × {args.repeat} koşu, {args.procs} paralel", flush=True)

    jobs = [(m, args.idx_from + i % args.procs, args.repeat, args.udp_timeout_us, args.env, args.tcs_th,
             args.pedal_gain, args.pedal_bias, args.assist_front, args.upshift)
            for i, m in enumerate(models)]
    results = []
    ctx = mp.get_context("spawn")
    # aynı idx'i iki süreç aynı anda kullanmasın: her "dalga"da procs kadar iş
    with ctx.Pool(args.procs) as pool:
        for i in range(0, len(jobs), args.procs):
            for r in pool.map(run_model, jobs[i:i + args.procs]):
                results.append(r)
                print(f"  {r['clean_finish_rate']:.0%} hasarsız | {r['finish_rate']:.0%} bitiş | "
                      f"tur {r['mean_lap']} (en iyi {r['best_lap']}) | hasar {r['mean_damage']} | {r['model']}",
                      flush=True)
    results.sort(key=rank_key)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    print("\nEN İYİ 5:")
    for r in results[:5]:
        print(f"  {r['clean_finish_rate']:.0%} hasarsız, tur ort {r['mean_clean_lap']} → {r['model']}")
    print("kaydedildi:", out)


if __name__ == "__main__":
    main()
