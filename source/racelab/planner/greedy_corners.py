"""Virajları sırayla tek tek sınırına kadar zorla (açgözlü, tur başından sona).

Her bölge (viraj) için hız çarpanı (sonra fren çarpanı) aday değerleri denenir. Yerel geçerlilik: o bölge + sonraki
150 m içinde det turda |pos| ≤ lim ve hasarsız; delay1 turda |pos| ≤ lim_rob ve hasarsız. Geçerliler arasında bölge
sonu + 150 m'deki süreye göre en hızlısı seçilir, sonraki bölgeye geçilir. Sonunda tam tur doğrulaması.

    python racelab/planner/greedy_corners.py --start runs/g2_best.json --name gc1 --idx 9 --passes 2
"""
import argparse
import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sim import run_lap  # noqa: E402
from try_one import make_driver  # noqa: E402
from zone_search2 import to_params  # noqa: E402

L = 3608.0


def lap_tele(idx, c, delay1):
    drv, _ = make_driver(to_params(c))
    r = run_lap(idx, drv, 1000000, record=True, delay1=delay1, offtrack_stop=1.3)
    return r, r.pop("tele")


def local(r, tele, lo, hi):
    """Bölge [lo, hi) içinde max |pos|, hasar artışı ve hi'ye varış süresi."""
    s = tele[:, 0]
    started = tele[:, 8] > 0.5
    if lo < 0:
        m = started & ((s >= lo + L) | (s < hi))
    else:
        m = started & (s >= lo) & (s < hi)
    if not m.any():
        return 9.0, 999.0, 999.0
    pos = float(np.abs(tele[m, 2]).max())
    dmg = float(tele[m, 7].max() - tele[started, 7].min()) if m.any() else 0.0
    idx = np.where(m)[0][-1]
    return pos, dmg, float(tele[idx, 8])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--idx", type=int, default=9)
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--lim", type=float, default=0.995)
    ap.add_argument("--lim-rob", type=float, default=1.06)
    a = ap.parse_args()
    out = os.path.join(HERE, "runs", f"{a.name}_best.json")
    assert not os.path.exists(out), out
    d = json.load(open(a.start))
    cur = {k: v for k, v in d.items() if not k.startswith("_")}
    cur.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]), ctrl="st")
    nz = len(cur["_z"])

    def evaluate(c, lo, hi):
        r1, t1 = lap_tele(a.idx, c, False)
        p1, d1, tt = local(r1, t1, lo, hi)
        r2, t2 = lap_tele(a.idx, c, True)
        p2, d2, _ = local(r2, t2, lo, hi)
        ok = p1 <= a.lim and d1 == 0 and p2 <= a.lim_rob and d2 == 0
        return ok, tt, (p1, p2, d1, d2), r1

    for ps in range(a.passes):
        for zi in range(nz):
            lo, hi = cur["_z"][zi][0], cur["_z"][zi][1] + 150
            for key, mults in (("_z", [0.97, 1.0, 1.03, 1.06, 1.1, 1.15, 1.2]), ("_bz", [0.9, 1.0, 1.1, 1.25, 1.45, 1.7])):
                best = None
                for mlt in mults:
                    c = copy.deepcopy(cur)
                    if key == "_z":
                        c["_z"][zi][2] = round(cur["_z"][zi][2] * mlt, 4)
                    else:
                        c["_bz"][zi] = round(cur["_bz"][zi] * mlt, 4)
                    ok, tt, info, r1 = evaluate(c, lo, hi)
                    if ok and (best is None or tt < best[0] - 1e-4):
                        best = (tt, c, mlt, info, r1)
                if best is not None:
                    cur = best[1]
                print(f"geçiş {ps} bölge {zi} ({cur['_z'][zi][3]:.0f}) {key}: seçilen ×{best[2] if best else '-'} "
                      f"bölge sonu t={best[0] if best else '-'} pos det/delay1={best[3][:2] if best else '-'} tur {best[4]['lap'] if best else '-'}",
                      flush=True)
        # tam tur kontrolü
        r1, t1 = lap_tele(a.idx, cur, False)
        r2, t2 = lap_tele(a.idx, cur, True)
        print(f"== geçiş {ps} sonu: det {r1['lap']} |pos| {r1['max_pos']:.3f} hasar {r1['damage']} | delay1 {r2['lap']} "
              f"|pos| {r2['max_pos']:.3f} hasar {r2['damage']}", flush=True)
        json.dump(dict({k: v for k, v in cur.items() if not k.startswith("_")}, _zones=cur["_z"], _bz=cur["_bz"],
                       _ns=cur["_ns"], _result=r1, _delay1=r2), open(out, "w"), indent=1)


if __name__ == "__main__":
    main()
