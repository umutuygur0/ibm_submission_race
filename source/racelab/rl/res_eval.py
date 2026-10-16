"""Artık RL adaylarının değerlendirmesi (deterministik politika): det, kalıcı gecikme + kaçırma, sadece kaçırma.

    python racelab/rl/res_eval.py --run res3 --top 6 --n 10 --idx 9
Adaylar run'ın candidates/ klasöründen (dosya adındaki tur süresine göre en iyi --top tanesi) + final.pt.
"""
import argparse
import glob
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from residual_env import RES_OBS, ResidualEnv  # noqa: E402
from sac import Actor  # noqa: E402


def lap(actor, base, idx, delay, drop, seed, cfg):
    env = ResidualEnv(idx, base, d_steer=cfg["d_steer"], d_pedal=cfg["d_pedal"], drop_prob=drop,
                      p_delay=1.0 if delay else 0.0, seed=seed, off_end=cfg.get("off_end", 1.2))
    o = env.reset()
    mp, n0 = 0.0, env.s["distRaced"]
    while True:
        with torch.no_grad():
            a, _ = actor(torch.as_tensor(o).unsqueeze(0), deterministic=True, with_logp=False)
        o, r, d, info = env.step(a.squeeze(0).numpy())
        if env.s["distRaced"] - n0 > 50:
            mp = max(mp, abs(env.s["trackPos"]))
        if d:
            break
    info["max_pos"] = mp
    return info.get("lap_time"), info.get("damage", 0), info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--top", type=int, default=6)
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--idx", type=int, default=9)
    ap.add_argument("--pattern", default="*clean*.pt")
    ap.add_argument("--no-final", action="store_true")
    a = ap.parse_args()
    import json
    rd = os.path.join(HERE, "runs", a.run)
    cfg = json.load(open(os.path.join(rd, "config.json")))
    base = cfg["res_params"]
    files = sorted(glob.glob(os.path.join(rd, "candidates", a.pattern)),
                   key=lambda f: float(os.path.basename(f).rsplit("_", 1)[1][:-3]))[:a.top]
    if not a.no_final and os.path.exists(os.path.join(rd, "final.pt")):
        files.append(os.path.join(rd, "final.pt"))
    torch.set_num_threads(1)
    for f in files:
        ck = torch.load(f, map_location="cpu", weights_only=False)
        actor = Actor(RES_OBS, 2, tuple(ck.get("hidden", [256, 256])))
        actor.load_state_dict(ck["actor"])
        det = lap(actor, base, a.idx, False, 0.0, 0, cfg)
        lag = [lap(actor, base, a.idx, True, 0.002, s, cfg) for s in range(a.n)]
        drp = [lap(actor, base, a.idx, False, 0.002, 100 + s, cfg) for s in range(a.n)]
        ok = lambda rr: sum(1 for t, d, _ in rr if t and d == 0)  # noqa: E731
        mean = lambda rr: np.mean([t for t, d, _ in rr if t and d == 0] or [0])  # noqa: E731
        onk = lambda rr: sum(1 for t, d, i in rr if t and d == 0 and i["max_pos"] <= 1.0)  # noqa: E731
        print(f"{os.path.basename(f):32s} det {det[0]} hasar {det[1]} |pos| {det[2]['max_pos']:.3f} | gecikme {ok(lag)}/{a.n} "
              f"(pist içi {onk(lag)}) ort {mean(lag):.3f} | kaçırma {ok(drp)}/{a.n} (pist içi {onk(drp)}) ort {mean(drp):.3f}",
              flush=True)


if __name__ == "__main__":
    main()
