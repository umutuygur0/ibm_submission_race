"""Tek bir viraj için kontrolcü kazanç çarpanı × hız çarpanı ızgarası (det; geçerlilik = o bölgede |pos| ≤ 1, hasarsız).

    python racelab/planner/zone_ctrl_probe.py --params runs/x.json --apex 1927 --idx 9
"""
import argparse
import copy
import itertools
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from greedy_corners import lap_tele, local  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--apex", type=float, required=True)
    ap.add_argument("--idx", type=int, default=9)
    ap.add_argument("--set", nargs="*", default=[])
    a = ap.parse_args()
    d = json.load(open(a.params))
    cur = {k: v for k, v in d.items() if not k.startswith("_")}
    for it in a.set:
        k, v = it.split("=", 1)
        cur[k] = json.loads(v)
    cur.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]), ctrl="st",
               _kz=d.get("_kz") or [[1.0, 1.0, 1.0] for _ in d["_zones"]])
    zi = min(range(len(cur["_z"])), key=lambda i: abs(cur["_z"][i][3] - a.apex))
    lo, hi = cur["_z"][zi][0], cur["_z"][zi][1] + 150
    rows = []
    for sm, fh, fe, ff in itertools.product([1.0, 1.1, 1.2], [0.5, 0.75, 1.0, 1.5], [0.5, 1.0, 2.0], [1.0, 1.3]):
        c = copy.deepcopy(cur)
        c["_z"][zi][2] *= sm
        c["_kz"][zi] = [fh, fe, ff]
        r, t = lap_tele(a.idx, c, False)
        pos, dmg, tt = local(r, t, lo, hi)
        m = (t[:, 0] > cur["_z"][zi][3] - 60) & (t[:, 0] < cur["_z"][zi][3] + 60)
        vmin = float(t[m, 1].min()) if m.any() else 0
        rows.append((sm, fh, fe, ff, round(pos, 3), dmg, round(tt, 3), round(vmin), r["lap"]))
        print(rows[-1], flush=True)
    ok = [x for x in rows if x[4] <= 1.0 and x[5] == 0]
    print("== en iyi geçerli (bölge sonu süresine göre):")
    for x in sorted(ok, key=lambda x: x[6])[:8]:
        print(x)


if __name__ == "__main__":
    main()
