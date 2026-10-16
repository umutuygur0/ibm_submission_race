"""Parametre sağlamlığı: tüm sayısal parametreler ve bölge çarpanları ±rel oranında rastgele oynatılır, N tur (det).
Ayarın parametre uzayında 'bıçak sırtı' olup olmadığını ölçer.

    python racelab/planner/param_robust.py --params runs/x.json --n 20 --rel 0.01 --idx 0
"""
import argparse
import copy
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build_line_submission import load_params  # noqa: E402
from sim import run_lap  # noqa: E402
from try_one import make_driver  # noqa: E402

SKIP = {"margin", "max_gear", "ctrl", "line", "st_csign", "launch", "offset", "v_max"}


def perturb(rng, p, rel):
    q = copy.deepcopy(p)
    for k, v in p.items():
        if k in SKIP or isinstance(v, bool) or not isinstance(v, (int, float)):
            continue
        q[k] = v * (1 + rng.uniform(-rel, rel))
    for key in ("zones", "bzones"):
        if q.get(key):
            q[key] = [[a, b, f * (1 + rng.uniform(-rel, rel))] for a, b, f in q[key]]
    if q.get("kzones"):
        q["kzones"] = [[a, b] + [f * (1 + rng.uniform(-rel, rel)) for f in fs] for a, b, *fs in q["kzones"]]
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--rel", type=float, default=0.01)
    ap.add_argument("--idx", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    p = load_params(a.params)
    p["ctrl"] = "st"
    ok = 0
    laps = []
    for i in range(a.n):
        q = perturb(rng, p, a.rel)
        drv, _ = make_driver(q)
        r = run_lap(a.idx, drv, 1000000)
        good = r["lap"] is not None and r["damage"] == 0
        ok += good
        if good:
            laps.append(r["lap"])
        print(f"{i}: tur {r['lap']} hasar {r['damage']} |pos| {r['max_pos']:.3f} {r['reason']}", flush=True)
    print(f"== {os.path.basename(a.params)} ±{a.rel*100:.1f}%: hasarsız bitiren {ok}/{a.n}, tur ort "
          f"{sum(laps)/max(len(laps),1):.3f}")


if __name__ == "__main__":
    main()
