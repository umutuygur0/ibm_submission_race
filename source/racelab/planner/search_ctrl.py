"""Kontrolcü parametrelerinin yerel evrimsel araması (bölge çarpanları sabit). 8 paralel tur (idx 1–8).

    python racelab/planner/search_ctrl.py --start runs/z3stb_best.json --name c1 --minutes 30

Başlangıç JSON'daki _zones/_bz korunur; SPACE'teki parametreler (alt, üst, adım) etrafında Gauss pertürbasyonu.
"""
import argparse
import csv
import json
import multiprocessing as mp
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import search  # noqa: E402
from zone_search2 import score_of, to_params  # noqa: E402

SPACE = {
    "st_kh": (0.2, 2.0, 0.15), "st_ke": (0.3, 5.0, 0.4), "st_kff": (0.5, 2.0, 0.1), "st_kd": (0.0, 2.0, 0.2),
    "st_ah": (0.0, 15.0, 1.5), "st_aff": (0.0, 15.0, 1.5), "st_kus": (0.0, 0.003, 0.0003), "st_lp": (0.2, 1.0, 0.1),
    "look_t": (0.0, 0.4, 0.03), "k_brake": (0.0, 0.4, 0.03), "band": (0.0, 2.5, 0.2),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--minutes", type=float, default=30)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--idx-from", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    runs = os.path.join(HERE, "runs")
    out = os.path.join(runs, f"{a.name}.csv")
    assert not os.path.exists(out), out
    best_f = os.path.join(runs, f"{a.name}_best.json")
    d = json.load(open(a.start))
    base = {k: v for k, v in d.items() if not k.startswith("_") or k in ("_zones", "_bz")}
    base["_z"] = base.pop("_zones")
    base.setdefault("_bz", [1.0] * len(base["_z"]))
    for k in SPACE:
        base.setdefault(k, {"st_kus": 0.0, "st_lp": 1.0}.get(k, 0.0))
    base["ctrl"] = "st"

    q = mp.Queue()
    for i in range(a.idx_from, a.idx_from + a.procs):
        q.put(i)
    pool = mp.Pool(a.procs, initializer=search._init, initargs=(q,))
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["n", "time", "score", "ok", "lap", "damage", "max_pos", "params"])
    pop = []
    (_, r0), = pool.map(search.evaluate, [to_params(base)])
    best_sc, _ = score_of(r0)
    best = (base, best_sc, r0)
    pop.append((best_sc, base))
    print(f"başlangıç {best_sc:.3f}", flush=True)
    t_end = time.time() + a.minutes * 60
    n = 0
    while time.time() < t_end:
        cands = []
        top = sorted(pop, key=lambda x: x[0])[:6]
        for _ in range(a.procs * 2):
            c = dict(rng.choice(top)[1])
            k_mut = rng.sample(list(SPACE), rng.randint(1, 4))
            for k in k_mut:
                lo, hi, st = SPACE[k]
                c[k] = round(min(hi, max(lo, c[k] + rng.gauss(0, st))), 5)
            cands.append(c)
        res = pool.map(search.evaluate, [to_params(c) for c in cands])
        for c, (_, r) in zip(cands, res):
            sc, ok = score_of(r)
            n += 1
            w.writerow([n, time.strftime("%H:%M:%S"), round(sc, 3), ok, r["lap"], r["damage"], round(r["max_pos"], 3),
                        json.dumps({k: c[k] for k in SPACE})])
            pop.append((sc, c))
            if sc < best[1] - 1e-6:
                best = (c, sc, r)
                json.dump(dict({kk: vv for kk, vv in c.items() if not kk.startswith("_")}, _zones=c["_z"], _bz=c["_bz"],
                               _score=sc, _result=r), open(best_f, "w"), indent=1)
                print(f"[{n}] yeni en iyi {sc:.3f} ({r['reason']}, pos {r['max_pos']:.2f})", flush=True)
        f.flush()
    pool.close()
    f.close()
    print("bitti", best[1])


if __name__ == "__main__":
    main()
