"""Çeşitlilik odaklı evrimsel arama: adalar (farklı başlangıçlar), büyük sıçramalar, adalar arası çaprazlama.

Her ada kendi popülasyonunu tutar; her nesilde her adadan aday üretilir (çoğu küçük mutasyon, bazıları büyük
sıçrama, bazıları başka adanın en iyisiyle çaprazlama). Değerlendirme: det + delay1 (ikisi de hasarsız; det |pos| ≤ lim,
delay1 |pos| ≤ lim_rob). Her adanın ve genelin en iyisi kaydedilir.

    python racelab/planner/evo2.py --starts runs/a.json runs/b.json ... --name g1 --minutes 300
    (başlangıca ek ayar: --set-island 3 st_course=1 st_slipmax=0.05)
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
        "st_lp": 0.04, "look_t": 0.02, "k_brake": 0.01, "band": 0.15, "acc_scale": 0.02, "mu_b": 0.04, "upshift": 150.0,
        "mu": 0.04, "aero": 0.0002, "aero_b": 0.0001, "st_course": 0.1, "st_slipmax": 0.015, "tcs": 0.4,
        "g2_v": 8.0, "dn_off": 300.0, "cl_max": 0.05, "cl_top": 400.0, "cl_band": 200.0, "cl_shift": 1.0, "st_lowv": 1.0}
LIMITS = {"st_lp": (0.05, 1.0), "st_course": (0.0, 1.0), "st_slipmax": (0.0, 0.4), "upshift": (16000, 19700),
          "tcs": (2.0, 15.0), "g2_v": (0.0, 120.0), "dn_off": (0.0, 5000.0), "cl_max": (0.0, 0.667),
          "cl_top": (12000.0, 19600.0), "cl_band": (100.0, 6000.0), "cl_shift": (0.0, 10.0), "st_lowv": (0.0, 10.0)}


def clamp(k, v):
    lo, hi = LIMITS.get(k, (0.0, 1e9))
    return min(hi, max(lo, v))


def mutate(rng, c, nz, big=False):
    c = copy.deepcopy(c)
    m = 5.0 if big else 1.0
    for _ in range(rng.randint(1, 6 if big else 4)):
        kind = rng.random()
        i = rng.randrange(nz)
        if kind < 0.3:
            c["_z"][i][2] = round(c["_z"][i][2] * (1 + rng.gauss(0, 0.02 * m)), 4)
        elif kind < 0.5:
            c["_bz"][i] = round(max(0.3, c["_bz"][i] * (1 + rng.gauss(0, 0.06 * m))), 4)
        elif kind < 0.62:
            c["_ns"][i] = round(c["_ns"][i] + rng.gauss(0, 0.4 * m), 3)
        elif kind < 0.74 and c.get("_kz"):
            j = rng.randrange(3)
            c["_kz"][i][j] = round(max(0.02, c["_kz"][i][j] * (1 + rng.gauss(0, 0.12 * m))), 4)
        else:
            k = rng.choice(list(CTRL))
            c[k] = round(clamp(k, c.get(k, 0.0) + rng.gauss(0, CTRL[k] * m)), 6)
    return c


def crossover(rng, a, b, nz):
    c = copy.deepcopy(a)
    for i in range(nz):  # bölge bazında genleri karıştır
        if rng.random() < 0.5:
            c["_z"][i][2], c["_bz"][i], c["_ns"][i] = b["_z"][i][2], b["_bz"][i], b["_ns"][i]
            if c.get("_kz") and b.get("_kz"):
                c["_kz"][i] = list(b["_kz"][i])
    for k in CTRL:
        if k in b and rng.random() < 0.3:
            c[k] = b[k]
    return c


def load_start(path, extra, lims):
    d = json.load(open(path))
    base = {k: v for k, v in d.items() if not k.startswith("_")}
    base.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]), ctrl="st",
                _kz=d.get("_kz") or [[1.0, 1.0, 1.0] for _ in d["_zones"]],
                _delay1=lims[2], _poslim=lims[0], _poslim_rob=lims[1], _offstop=lims[3])
    base.update(extra)
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--starts", nargs="+", required=True)
    ap.add_argument("--set-island", nargs="+", action="append", default=[], help="ADA_NO ad=json ...")
    ap.add_argument("--name", required=True)
    ap.add_argument("--minutes", type=float, default=300)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--pos-lim", type=float, default=1.0)
    ap.add_argument("--pos-lim-rob", type=float, default=1.06)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--no-delay1", action="store_true", help="sadece det tur (gecikme turu yok)")
    ap.add_argument("--offstop", type=float, default=1.15)
    ap.add_argument("--prob", type=int, default=0, help="parametre sağlamlığı: aday başına ±prel oynatılmış ek det tur")
    ap.add_argument("--prel", type=float, default=0.007)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    zone_search2.POS_LIM = a.pos_lim
    runs = os.path.join(HERE, "runs")
    out = os.path.join(runs, f"{a.name}.csv")
    assert not os.path.exists(out), out
    extras = {}
    for item in a.set_island:
        isl = int(item[0])
        extras.setdefault(isl, {}).update({k: json.loads(v) for k, v in (x.split("=", 1) for x in item[1:])})
    islands = [load_start(p, extras.get(i, {}), (a.pos_lim, a.pos_lim_rob, not a.no_delay1, a.offstop))
               for i, p in enumerate(a.starts)]
    for isl in islands:
        isl["_prob"], isl["_prel"] = a.prob, a.prel
    nz = len(islands[0]["_z"])
    assert all(len(s["_z"]) == nz for s in islands), "tüm başlangıçlar aynı bölge sayısında olmalı"

    q = mp.Queue()
    for i in range(1, a.procs + 1):
        q.put(i)
    pool = mp.Pool(a.procs, initializer=search._init, initargs=(q,))
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["n", "time", "island", "op", "score", "ok", "det_lap", "delay1_lap", "max_pos", "delay1_pos", "damage", "reason"])

    def save(tag, sc, c, r):
        json.dump(dict({k: v for k, v in c.items() if not k.startswith("_")}, _zones=c["_z"], _bz=c["_bz"],
                       _ns=c["_ns"], _kz=c.get("_kz"), _score=sc, _result=r),
                  open(os.path.join(runs, f"{a.name}_{tag}.json"), "w"), indent=1)

    res0 = pool.map(search.evaluate, [to_params(s) for s in islands])
    pops = []
    best = None
    for i, (s, (_, r)) in enumerate(zip(islands, res0)):
        sc, _ = zone_search2.score_of(r)
        pops.append([(sc, s)])
        print(f"ada {i}: başlangıç {sc:.3f} det {r.get('det_lap')} delay1 {r.get('delay1_lap')}", flush=True)
        if best is None or sc < best[0]:
            best = (sc, s, r)
    t_end = time.time() + a.minutes * 60
    n = 0
    per = max(2, a.procs * 2 // len(islands))
    while time.time() < t_end:
        cands = []
        for i, pop in enumerate(pops):
            top = sorted(pop, key=lambda x: x[0])[:5]
            for _ in range(per):
                u = rng.random()
                if u < 0.12:
                    other = rng.choice([j for j in range(len(pops)) if j != i] or [i])
                    c = crossover(rng, rng.choice(top)[1], min(pops[other], key=lambda x: x[0])[1], nz)
                    op = "cross"
                elif u < 0.27:
                    c, op = mutate(rng, rng.choice(top)[1], nz, big=True), "big"
                else:
                    c, op = mutate(rng, rng.choice(top)[1], nz), "small"
                cands.append((i, op, c))
        res = pool.map(search.evaluate, [to_params(c) for _, _, c in cands])
        for (i, op, c), (_, r) in zip(cands, res):
            sc, ok = zone_search2.score_of(r)
            n += 1
            w.writerow([n, time.strftime("%H:%M:%S"), i, op, round(sc, 3), ok, r.get("det_lap"), r.get("delay1_lap"),
                        round(r["max_pos"], 4), round(r.get("delay1_pos", 9), 4), r["damage"], r["reason"]])
            if sc < min(p[0] for p in pops[i]) - 1e-6:
                save(f"island{i}", sc, c, r)
                print(f"[{n}] ada {i} ({op}) yeni en iyi {sc:.3f} (det {r.get('det_lap')}, delay1 {r.get('delay1_lap')})", flush=True)
            pops[i].append((sc, c))
            if sc < best[0] - 1e-6:
                best = (sc, c, r)
                save("best", sc, c, r)
                print(f"[{n}] GENEL en iyi {sc:.3f} (ada {i}, {op})", flush=True)
        for i in range(len(pops)):
            pops[i] = sorted(pops[i], key=lambda x: x[0])[:30]
        f.flush()
    pool.close()
    f.close()
    print("bitti", best[0])


if __name__ == "__main__":
    main()
