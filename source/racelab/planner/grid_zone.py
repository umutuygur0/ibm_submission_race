"""Tek bir bölge için ızgara taraması (hız × fren × apeks kaydırma), det + delay1 (9 paralel).

    python racelab/planner/grid_zone.py --params runs/snap_z12_final.json --zone 9
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
    ap.add_argument("--zone", type=int, required=True)
    ap.add_argument("--fs", default="0.97,0.985,1.0")
    ap.add_argument("--fb", default="0.6,0.8,1.0")
    ap.add_argument("--dn", default="0,-0.5,-1.0")
    a = ap.parse_args()
    d = json.load(open(a.params))
    base = {k: v for k, v in d.items() if not k.startswith("_")}
    base.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]),
                _delay1=True, _poslim=9.0, _poslim_rob=9.0, ctrl="st")
    cands = []
    for fs, fb, dn in itertools.product(*[list(map(float, x.split(","))) for x in (a.fs, a.fb, a.dn)]):
        c = copy.deepcopy(base)
        c["_z"][a.zone][2] *= fs
        c["_bz"][a.zone] *= fb
        c["_ns"][a.zone] += dn
        c["_tag"] = (fs, fb, dn)
        cands.append(c)
    q = mp.Queue()
    for i in range(1, 10):
        q.put(i)
    pool = mp.Pool(9, initializer=search._init, initargs=(q,))
    res = pool.map(search.evaluate, [to_params(c) for c in cands])
    rows = []
    for c, (_, r) in zip(cands, res):
        rows.append((c["_tag"], r.get("det_lap"), r.get("delay1_lap"), round(r["max_pos"], 4), round(r.get("delay1_pos", 9), 4), r["damage"], r["reason"]))
    for row in sorted(rows, key=lambda x: (x[4], x[1] or 999)):
        print(row, flush=True)


if __name__ == "__main__":
    main()
