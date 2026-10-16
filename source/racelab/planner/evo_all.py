"""Tüm parametrelerin (bölge hız/fren/apeks çarpanları + kontrolcü) evrimsel araması: aynı anda 1–4 boyut değişir
(koordinat inişinin takıldığı yerel optimumdan çıkmak için). Değerlendirme: det + N bozulmuş tur (adım kaçırma).

    python racelab/planner/evo_all.py --start runs/snap_z12_final.json --name e1 --minutes 150
"""
import argparse
import copy
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
import zone_search2  # noqa: E402
from zone_search2 import to_params  # noqa: E402

CTRL = {"st_kh": 0.08, "st_ke": 0.15, "st_kff": 0.06, "st_kd": 0.08, "st_ah": 0.8, "st_aff": 0.8, "st_kus": 0.0001,
        "st_lp": 0.04, "look_t": 0.02, "k_brake": 0.01, "band": 0.15, "acc_scale": 0.02, "mu_b": 0.04, "upshift": 150.0}


def mutate(rng, c, nz):
    c = copy.deepcopy(c)
    for _ in range(rng.randint(1, 4)):
        kind = rng.random()
        i = rng.randrange(nz)
        if kind < 0.3:
            c["_z"][i][2] = round(c["_z"][i][2] * (1 + rng.gauss(0, 0.02)), 4)
        elif kind < 0.55:
            c["_bz"][i] = round(max(0.3, c["_bz"][i] * (1 + rng.gauss(0, 0.06))), 4)
        elif kind < 0.7:
            c["_ns"][i] = round(c["_ns"][i] + rng.gauss(0, 0.4), 3)
        else:
            k = rng.choice(list(CTRL))
            c[k] = round(max(0.0, c.get(k, 0.0) + rng.gauss(0, CTRL[k])), 5)
            if k == "st_lp":
                c[k] = min(1.0, c[k])
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--minutes", type=float, default=150)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--robust", type=int, default=1)
    ap.add_argument("--pos-lim", type=float, default=1.0)
    ap.add_argument("--pos-lim-rob", type=float, default=1.06)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--delay1", action="store_true", help="ek deterministik tur: kalıcı 1 adım gecikme (hasarsız, |pos| ≤ pos-lim-rob)")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    zone_search2.POS_LIM = a.pos_lim
    runs = os.path.join(HERE, "runs")
    out = os.path.join(runs, f"{a.name}.csv")
    assert not os.path.exists(out), out
    best_f = os.path.join(runs, f"{a.name}_best.json")
    d = json.load(open(a.start))
    base = {k: v for k, v in d.items() if not k.startswith("_")}
    base.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]),
                _robust=a.robust, _poslim=a.pos_lim, _poslim_rob=a.pos_lim_rob, ctrl="st", _delay1=a.delay1)
    nz = len(base["_z"])
    q = mp.Queue()
    for i in range(1, a.procs + 1):
        q.put(i)
    pool = mp.Pool(a.procs, initializer=search._init, initargs=(q,))
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["n", "time", "score", "ok", "lap", "det_lap", "max_pos", "damage", "reason"])
    (_, r0), = pool.map(search.evaluate, [to_params(base)])
    sc0, _ = zone_search2.score_of(r0)
    pop = [(sc0, base)]
    best = (sc0, base, r0)
    print(f"başlangıç {sc0:.3f} {r0}", flush=True)
    t_end = time.time() + a.minutes * 60
    n = 0
    while time.time() < t_end:
        top = sorted(pop, key=lambda x: x[0])[:5]
        cands = [mutate(rng, rng.choice(top)[1], nz) for _ in range(a.procs * 2)]
        res = pool.map(search.evaluate, [to_params(c) for c in cands])
        for c, (_, r) in zip(cands, res):
            sc, ok = zone_search2.score_of(r)
            n += 1
            w.writerow([n, time.strftime("%H:%M:%S"), round(sc, 3), ok, r["lap"], r.get("det_lap"), round(r["max_pos"], 4),
                        r["damage"], r["reason"]])
            pop.append((sc, c))
            if sc < best[0] - 1e-6:
                best = (sc, c, r)
                json.dump(dict({k: v for k, v in c.items() if not k.startswith("_")}, _zones=c["_z"], _bz=c["_bz"],
                               _ns=c["_ns"], _score=sc, _result=r), open(best_f, "w"), indent=1)
                print(f"[{n}] yeni en iyi {sc:.3f} (det {r.get('det_lap')}, pos {r['max_pos']:.3f})", flush=True)
        pop = sorted(pop, key=lambda x: x[0])[:40]
        f.flush()
    pool.close()
    f.close()
    print("bitti", best[0])


if __name__ == "__main__":
    main()
