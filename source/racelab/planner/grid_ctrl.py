"""Kontrolcü kazançları ızgarası, det + delay1 (9 paralel).

    python racelab/planner/grid_ctrl.py --params runs/snap_z12_final.json --grid st_kh=1.2,1.44,1.7 st_ke=1.0,1.3,1.7
"""
import argparse
import copy
import itertools
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import search  # noqa: E402
from zone_search2 import to_params  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--grid", nargs="+", required=True)
    ap.add_argument("--rel", action="store_true", help="değerler mevcut değerin çarpanı")
    a = ap.parse_args()
    d = json.load(open(a.params))
    base = {k: v for k, v in d.items() if not k.startswith("_")}
    base.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]),
                _delay1=True, _poslim=9.0, _poslim_rob=9.0, ctrl="st")
    keys = [g.split("=")[0] for g in a.grid]
    vals = [list(map(float, g.split("=")[1].split(","))) for g in a.grid]
    cands = []
    for combo in itertools.product(*vals):
        c = copy.deepcopy(base)
        for k, v in zip(keys, combo):
            c[k] = c.get(k, 0.0) * v if a.rel else v
        c["_tag"] = combo
        cands.append(c)
    q = mp.Queue()
    for i in range(1, 10):
        q.put(i)
    pool = mp.Pool(9, initializer=search._init, initargs=(q,))
    res = pool.map(search.evaluate, [to_params(c) for c in cands])
    rows = [(c["_tag"], r.get("det_lap", r["lap"]), r.get("delay1_lap"), round(r["max_pos"], 4),
             round(r.get("delay1_pos", 9), 4), r["damage"], r["reason"]) for c, (_, r) in zip(cands, res)]
    print("anahtarlar:", keys, " mevcut:", [base.get(k) for k in keys])
    for row in sorted(rows, key=lambda x: (max(x[3], x[4]) > 1.0, (x[1] or 999) + (x[2] or 999))):
        print(row, flush=True)


if __name__ == "__main__":
    main()
