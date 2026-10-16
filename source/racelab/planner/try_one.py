"""Tek parametre setiyle tek tur (hızlı deneme). Çizgi önbelleği: planner/cache/line_m<pay>.npz

    python racelab/planner/try_one.py --idx 1 --set mu=2.2 ld0=8 [--tele out.npy] [--to 10000]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from line_driver import DEFAULTS, DS, LineDriver, build_plan  # noqa: E402
from plan_line import load_track, min_curvature_line  # noqa: E402
from sim import run_lap  # noqa: E402

CACHE = os.path.join(HERE, "cache")
_TRACK = None


def get_track():
    global _TRACK
    if _TRACK is None:
        _TRACK = load_track(DS)
    return _TRACK


def get_line(margin, variant=None):
    os.makedirs(CACHE, exist_ok=True)
    if variant:  # line_variants.py çıktısı (ör. "irls", "mc0")
        d = np.load(os.path.join(CACHE, f"line_{variant}_m{margin:.2f}.npz"))
        return d["n"], d["P"]
    f = os.path.join(CACHE, f"line_m{margin:.2f}.npz")
    if os.path.exists(f):
        d = np.load(f)
        return d["n"], d["P"]
    s, c, w, L = get_track()
    n, P = min_curvature_line(c[:, :2], w, margin=margin)
    tmp = f + f".{os.getpid()}.tmp.npz"
    np.savez(tmp, n=n, P=P)
    try:
        os.replace(tmp, f)
    except OSError:
        pass
    return n, P


def make_driver(params):
    p = dict(DEFAULTS, **params)
    plan = build_plan(p, track=get_track(), line=get_line(p["margin"], p.get("line")))
    return LineDriver(plan, p), plan


def parse_set(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        out[k] = json.loads(v)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--idx", type=int, default=1)
    ap.add_argument("--set", nargs="*")
    ap.add_argument("--params", help="JSON dosyası")
    ap.add_argument("--to", type=int, default=1000000)
    ap.add_argument("--tele")
    a = ap.parse_args()
    params = json.load(open(a.params)) if a.params else {}
    params.update(parse_set(a.set))
    drv, plan = make_driver(params)
    r = run_lap(a.idx, drv, a.to, record=bool(a.tele))
    if a.tele:
        np.save(a.tele, r.pop("tele"))
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}), flush=True)
