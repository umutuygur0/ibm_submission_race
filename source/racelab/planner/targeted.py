"""Hedefli koordinat araması (toplam tur süresi; stres geçerliliğiyle). 9 paralel.

Anahtar adları: global parametre (ör. g2_v, tcs_l, cl_top) ya da bölge geni:
  z<i>  hız çarpanı, bz<i> fren çarpanı, ns<i> apeks kaydırma (m, toplamsal), kh<i>/ke<i>/kf<i> kazanç çarpanları.
Her anahtar için mevcut değer etrafında 9 değer (çarpımsal; ns için toplamsal) denenir; aday geçerliyse
(det |pos| ≤ lim, hasarsız; delay1 ve 2 rastgele kaçırma turu hasarsız, |pos| ≤ lim_rob) ve det tur daha kısaysa kabul.

    python racelab/planner/targeted.py --start runs/snap_gr3L.json --name t1 --keys g2_v tcs_l z1 bz1 --passes 2
"""
import argparse
import copy
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import search  # noqa: E402
from zone_search2 import to_params  # noqa: E402

MULTS = [0.92, 0.95, 0.97, 0.985, 1.015, 1.03, 1.05, 1.08, 1.12]
ADDS = [-1.0, -0.6, -0.3, -0.15, 0.15, 0.3, 0.6, 1.0, 1.5]


def get_set(c, key, val=None):
    import re
    m = re.fullmatch(r"(z|bz|ns|kh|ke|kf)(\d+)", key)
    if not m:
        if val is None:
            return c.get(key, 0.0)
        c[key] = val
        return val
    kind, i = m.group(1), int(m.group(2))
    if kind == "z":
        if val is None:
            return c["_z"][i][2]
        c["_z"][i][2] = val
    elif kind == "bz":
        if val is None:
            return c["_bz"][i]
        c["_bz"][i] = val
    elif kind == "ns":
        if val is None:
            return c["_ns"][i]
        c["_ns"][i] = val
    else:
        j = {"kh": 0, "ke": 1, "kf": 2}[kind]
        if val is None:
            return c["_kz"][i][j]
        c["_kz"][i][j] = val
    return val


def valid(r, lim):
    return r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= lim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--keys", nargs="+", required=True)
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--lim", type=float, default=1.0)
    ap.add_argument("--lim-rob", type=float, default=1.06)
    ap.add_argument("--robust", type=int, default=2)
    ap.add_argument("--offstop", type=float, default=1.15, help="|trackPos| bu değeri aşınca tur durdurulur")
    ap.add_argument("--lagrob", type=int, default=0, help="ek gecikme+kaçırma turu sayısı (hepsi hasarsız olmalı)")
    a = ap.parse_args()
    out = os.path.join(HERE, "runs", f"{a.name}_best.json")
    assert not os.path.exists(out), out
    d = json.load(open(a.start))
    cur = {k: v for k, v in d.items() if not k.startswith("_")}
    cur.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]), ctrl="st",
               _kz=d.get("_kz") or [[1.0, 1.0, 1.0] for _ in d["_zones"]],
               _delay1=True, _robust=a.robust, _poslim=a.lim, _poslim_rob=a.lim_rob, _offstop=a.offstop, _lagrob=a.lagrob)
    q = mp.Queue()
    for i in range(1, 10):
        q.put(i)
    pool = mp.Pool(9, initializer=search._init, initargs=(q,))
    (_, r0), = pool.map(search.evaluate, [to_params(cur)])
    best_lap = r0.get("det_lap", r0["lap"]) if valid(r0, a.lim) else 999.0
    print(f"başlangıç det {best_lap} (delay1 {r0.get('delay1_lap')}, geçerli {valid(r0, a.lim)})", flush=True)

    def save(r):
        json.dump(dict({k: v for k, v in cur.items() if not k.startswith("_")}, _zones=cur["_z"], _bz=cur["_bz"],
                       _ns=cur["_ns"], _kz=cur["_kz"], _result=r), open(out, "w"), indent=1)
    save(r0)
    for ps in range(a.passes):
        for key in a.keys:
            v0 = get_set(cur, key)
            add = key.startswith("ns")
            cands = []
            for m in (ADDS if add else MULTS):
                c = copy.deepcopy(cur)
                get_set(c, key, round(v0 + m if add else v0 * m, 6))
                cands.append(c)
            res = pool.map(search.evaluate, [to_params(c) for c in cands])
            ok = [(r.get("det_lap", r["lap"]), c, r) for c, (_, r) in zip(cands, res) if valid(r, a.lim)]
            if ok:
                lap, c, r = min(ok, key=lambda x: x[0])
                if lap < best_lap - 1e-6:
                    best_lap, cur = lap, c
                    save(r)
            print(f"geçiş {ps} {key}: {v0:.4g} → {get_set(cur, key):.4g} | det {best_lap} ({len(ok)}/{len(cands)} geçerli)",
                  flush=True)
        print(f"== geçiş {ps} bitti: det {best_lap}", flush=True)
    pool.close()


if __name__ == "__main__":
    main()
