"""Bir modelin Corkscrew'da (deterministik, race_mode) zikzak ölçümü.

    python racelab/rl/zigzag.py model1.pt model2.pt [--idx 9]

Hız>150 km/h iken: saniyede direksiyon yön değişimi, ortalama |steer|, ortalama gaz;
ayrıca tur süresi/hasar. v2/v4 modelleri için (v3 sert sınır için değil).
"""
import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ACT_DIM, OBS_DIM, to_command  # noqa: E402
from sac import Actor  # noqa: E402
from torcs_env_v2 import TorcsEnvV2  # noqa: E402


def measure(path, idx):
    ck = torch.load(path, map_location="cpu", weights_only=False)
    actor = Actor(OBS_DIM, ACT_DIM, tuple(ck.get("hidden", [256, 256])))
    actor.load_state_dict(ck["actor"])
    env = TorcsEnvV2(idx=idx, race_mode=True, max_steps=30000)
    o = env.reset()
    log = []
    while True:
        with torch.no_grad():
            a, _ = actor(torch.as_tensor(o).unsqueeze(0), deterministic=True, with_logp=False)
        a = a.squeeze(0).numpy()
        cmd = to_command(a, env.s)
        log.append((env.s["speedX"], float(a[0]), cmd["accel"]))
        o, r, d, info = env.step(a)
        if d:
            break
    fast = [x for x in log if x[0] > 150]
    st = [x[1] for x in fast]
    flips = sum(1 for p, q in zip(st, st[1:]) if p * q < 0 and abs(p - q) > 0.05)
    secs = max(len(fast) * 0.02, 1e-9)
    return {"lap": info.get("lap_time"), "damage": info.get("damage"), "dist": round(info.get("distance", 0)),
            "flips_per_s": round(flips / secs, 1), "mean_abs_steer": round(float(np.mean(np.abs(st))), 3) if st else None,
            "mean_accel": round(float(np.mean([x[2] for x in fast])), 2) if fast else None,
            "max_speed": round(max(x[0] for x in log), 1)}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="+")
    ap.add_argument("--idx", type=int, default=9)
    args = ap.parse_args()
    for m in args.models:
        print(measure(m, args.idx), m, flush=True)
