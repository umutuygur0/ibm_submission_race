"""Viraj bazlı hız çarpanı araması (koordinat inişi, 9 paralel deterministik tur).

Virajlar: yanal sınırlı hızın yerel minimumları; bölge sınırları ardışık minimumların ortası.
Her turda her bölge için 9 çarpan denenir (mevcut çarpan etrafında), en hızlı geçerli (hasarsız, |pos|<=lim) tur seçilir.
Ayrıca global parametreler (ld0, ld_v, k_steer, look_t, k_brake, band) aynı şekilde taranır.

    python racelab/planner/zone_search.py --start runs/p1_best.json --name z1 --minutes 60
"""
import argparse
import copy
import csv
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import search  # noqa: E402

POS_LIM = 1.0


def find_zones(params):
    from try_one import make_driver
    _, plan = make_driver(dict(params, zones=None))
    v, s, L = plan["v"], plan["s"], plan["L"]
    m = len(v)
    # yerel minimumlar (±40 m pencerede), hızı tepe hızından belirgin düşük olanlar
    w = 20
    mins = [i for i in range(m) if v[i] == min(v[(i + j) % m] for j in range(-w, w + 1)) and v[i] < 0.93 * v.max()]
    ded = []
    for i in mins:
        if not ded or s[i] - s[ded[-1]] > 60:
            ded.append(i)
    apex = [float(s[i]) for i in ded]
    bounds = []
    for a, b in zip(apex, apex[1:] + [apex[0] + L]):
        bounds.append((a + b) / 2)
    zones = []
    for k, a in enumerate(apex):
        lo = bounds[k - 1] if k > 0 else bounds[-1] - L
        zones.append([round(lo, 1), round(bounds[k], 1), 1.0, round(a, 1)])
    return zones


def zones_param(zones):
    out = []
    for lo, hi, f, _ in zones:
        if lo < 0:
            out += [[lo + 3608.0, 3608.0, f], [0.0, hi, f]]
        else:
            out.append([lo, hi, f])
    return out


def score_of(r):
    ok = r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= POS_LIM
    return (r["lap"] if ok else 300 - r["dist"] / 20 + (50 if r["lap"] else 0)), ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--minutes", type=float, default=60)
    ap.add_argument("--procs", type=int, default=9)
    ap.add_argument("--idx-from", type=int, default=1)
    ap.add_argument("--ncand", type=int, default=9)
    ap.add_argument("--zones", help="önceki zone JSON (devam)")
    ap.add_argument("--stanley", action="store_true", help="Stanley kontrolcüsü + parametreleri de taranır")
    ap.add_argument("--set", nargs="*", help="başlangıç değerlerini değiştir: ad=json")
    a = ap.parse_args()
    runs = os.path.join(HERE, "runs")
    out = os.path.join(runs, f"{a.name}.csv")
    assert not os.path.exists(out), out
    best_f = os.path.join(runs, f"{a.name}_best.json")
    base = {k: v for k, v in json.load(open(a.start)).items() if not k.startswith("_")}
    zones = json.load(open(a.zones))["_zones"] if a.zones else (base.pop("_zones", None) or find_zones(base))
    base.pop("zones", None)
    for it in a.set or []:
        k_, v_ = it.split("=", 1)
        base[k_] = json.loads(v_)
    if a.stanley:
        base["ctrl"] = "st"
        for k_, v_ in dict(st_kh=1.0, st_ke=2.0, st_kff=1.0, st_kd=0.0, st_ah=4.0, st_aff=6.0).items():
            base.setdefault(k_, v_)
    print(f"{len(zones)} bölge: " + ", ".join(f"{z[3]:.0f}" for z in zones), flush=True)

    q = mp.Queue()
    for i in range(a.idx_from, a.idx_from + a.procs):
        assert 1 <= i <= 9
        q.put(i)
    pool = mp.Pool(a.procs, initializer=search._init, initargs=(q,))
    f = open(out, "w", newline="", encoding="utf-8")
    wr = csv.writer(f)
    wr.writerow(["n", "time", "what", "value", "score", "ok", "lap", "damage", "max_pos", "dist"])
    n = 0

    def run_many(cands):
        nonlocal n
        res = pool.map(search.evaluate, [dict(c, zones=zones_param(c["_z"])) if "_z" in c else c for c in cands])
        outl = []
        for c, (_, r) in zip(cands, res):
            sc, ok = score_of(r)
            n += 1
            wr.writerow([n, time.strftime("%H:%M:%S"), c.get("_what"), c.get("_val"), round(sc, 3), ok, r["lap"],
                         r["damage"], round(r["max_pos"], 3), round(r["dist"], 1)])
            outl.append((sc, ok, r))
        f.flush()
        return outl

    cur = dict(base, _z=copy.deepcopy(zones))
    (best_sc, _, best_r), = run_many([cur])
    print(f"başlangıç {best_sc:.3f}", flush=True)
    t_end = time.time() + a.minutes * 60
    step = 0.06
    glob_keys = {"ld0": 1.0, "ld_v": 0.03, "k_steer": 0.15, "look_t": 0.04, "k_brake": 0.03, "band": 0.2,
                 "mu_b": 0.1, "aero_b": 0.0003, "acc_scale": 0.03, "vert": 0.3, "upshift": 400.0, "tcs": 1.5, "margin": 0.4}
    if base.get("ctrl") == "st":
        for k_ in ("ld0", "ld_v", "k_steer"):
            glob_keys.pop(k_)
        glob_keys.update(st_kh=0.15, st_ke=0.5, st_kff=0.15, st_kd=0.3, st_ah=2.0, st_aff=2.0)
    sweep = 0
    while time.time() < t_end:
        sweep += 1
        improved = 0
        for zi in range(len(cur["_z"])):
            if time.time() > t_end:
                break
            cands = []
            f0 = cur["_z"][zi][2]
            for d in np.linspace(-step, step * 1.5, a.ncand):
                c = copy.deepcopy(cur)
                c["_z"][zi][2] = round(f0 * (1 + d), 4)
                c["_what"], c["_val"] = f"zone{zi}@{cur['_z'][zi][3]:.0f}", c["_z"][zi][2]
                cands.append(c)
            res = run_many(cands)
            k = int(np.argmin([r[0] for r in res]))
            if res[k][0] < best_sc - 1e-6:
                best_sc, cur = res[k][0], cands[k]
                improved += 1
                json.dump(dict({kk: vv for kk, vv in cur.items() if not kk.startswith("_")}, _zones=cur["_z"],
                               _score=best_sc, _result=res[k][2]), open(best_f, "w"), indent=1)
        for gk, gs in glob_keys.items():
            if time.time() > t_end:
                break
            cands = []
            g0 = cur.get(gk, 0.0)
            for d in np.linspace(-1, 1, a.ncand):
                c = copy.deepcopy(cur)
                c[gk] = round(max(0.0, g0 + d * gs), 5)
                if gk == "margin":
                    c[gk] = round(min(2.2, max(0.5, c[gk])), 1)
                c["_what"], c["_val"] = gk, c[gk]
                cands.append(c)
            res = run_many(cands)
            k = int(np.argmin([r[0] for r in res]))
            if res[k][0] < best_sc - 1e-6:
                best_sc, cur = res[k][0], cands[k]
                improved += 1
                json.dump(dict({kk: vv for kk, vv in cur.items() if not kk.startswith("_")}, _zones=cur["_z"],
                               _score=best_sc, _result=res[k][2]), open(best_f, "w"), indent=1)
        print(f"tur {sweep}: en iyi {best_sc:.3f}, iyileşme {improved}, adım {step:.3f} ({n} deneme)", flush=True)
        if improved == 0:
            step *= 0.6
            if step < 0.005:
                step = 0.06
    pool.close()
    f.close()
    print("bitti", best_sc)


if __name__ == "__main__":
    main()
