"""Sürücü parametrelerinin otomatik araması (9 paralel hızlı simülasyon, idx 1–9).

Amaç: hasarsız ve pist içinde (|trackPos| <= pos_lim) en hızlı tur. Bitiremeyen: 300 − mesafe/20 cezası.
Strateji: önce rastgele örnekler, sonra en iyilerin etrafında daralan pertürbasyon.
Her deneme planner/runs/<ad>.csv'ye eklenir; en iyi parametreler planner/runs/<ad>_best.json.

    python racelab/planner/search.py --name s1 --minutes 60 [--start best.json] [--procs 9]
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

SPACE = {  # ad: (alt, üst, tür)
    "margin": (0.8, 2.2, "margin"),
    "mu": (1.4, 3.2, "f"), "aero": (0.0, 0.004, "f"),
    "mu_b": (1.6, 3.6, "f"), "aero_b": (0.0, 0.004, "f"),
    "ld0": (2.0, 14.0, "f"), "ld_v": (0.05, 0.5, "f"),
    "k_steer": (0.7, 2.5, "f"), "look_t": (0.0, 0.5, "f"),
    "k_brake": (0.02, 0.4, "f"), "band": (0.0, 2.0, "f"),
    "acc_scale": (0.8, 1.3, "f"),
}
MARGINS = [0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2]
POS_LIM = 1.0

_idx = None


def _init(q):
    global _idx
    _idx = q.get()


def evaluate(params):
    from try_one import make_driver
    from sim import run_lap
    t0 = time.time()
    lim = float(params.get("_poslim", POS_LIM))
    try:
        drv, _ = make_driver(params)
        r = run_lap(_idx, drv, 1000000, offtrack_stop=float(params.get("_offstop", 1.15)))
        # _robust=N: ek N tur 10 ms bekleme ile; hepsi geçerli olmalı, en kötüsü raporlanır
        laps = [r["lap"]]
        nprob = int(params.get("_prob", 0) or 0)
        if nprob and r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= lim:
            # parametre sağlamlığı: tüm sayısal parametreler ve bölge listeleri ±_prel oynatılmış det turlar hasarsız bitmeli
            import random as _rnd
            rng_ = _rnd.Random(hash((_idx, time.time())) & 0xFFFF)
            prel = float(params.get("_prel", 0.007))
            skip = {"margin", "max_gear", "launch", "offset", "v_max", "st_csign"}
            for _ in range(nprob):
                q = {}
                for k, v in params.items():
                    if k.startswith("_") or k in skip or isinstance(v, bool) or not isinstance(v, (int, float)):
                        q[k] = v
                    else:
                        q[k] = v * (1 + rng_.uniform(-prel, prel))
                for key in ("zones", "bzones"):
                    if params.get(key):
                        q[key] = [[a_, b_, f * (1 + rng_.uniform(-prel, prel))] for a_, b_, f in params[key]]
                if params.get("kzones"):
                    q["kzones"] = [[a_, b_] + [f * (1 + rng_.uniform(-prel, prel)) for f in fs]
                                   for a_, b_, *fs in params["kzones"]]
                drv, _ = make_driver(q)
                rp = run_lap(_idx, drv, 1000000, offtrack_stop=float(params.get("_offstop", 1.15)))
                if not (rp["lap"] is not None and rp["damage"] == 0):
                    r = dict(rp, reason="prob_" + rp["reason"], lap=rp["lap"])
                    r["max_pos"] = max(rp["max_pos"], lim + 0.02)
                    break
        if params.get("_delay1") and not r["reason"].startswith("prob_") and r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= lim:
            # kalıcı 1 adım gecikme (run_benchmark 10 ms koşullarını birebir taklit eder), deterministik
            drv, _ = make_driver(params)
            r3 = run_lap(_idx, drv, 1000000, offtrack_stop=float(params.get("_offstop", 1.15)), delay1=True)
            r["delay1_pos"] = r3["max_pos"]
            lim_d = float(params.get("_poslim_rob", lim))
            if not (r3["lap"] is not None and r3["damage"] == 0 and r3["max_pos"] <= lim_d):
                r = dict(r3, reason="delay1_" + r3["reason"])
                if r3["lap"] is not None and r3["damage"] == 0:
                    r["max_pos"] = max(r3["max_pos"], lim + 0.01)
            else:
                laps.append(r3["lap"])
                r["delay1_lap"] = r3["lap"]
        for _ in range(int(params.get("_robust", 0) or 0)):
            if not (r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= lim) or r["reason"].startswith("delay1_"):
                break
            drv, _ = make_driver(params)
            r2 = run_lap(_idx, drv, 10000, offtrack_stop=float(params.get("_offstop", 1.15)), drop_prob=float(params.get("_drop", 0.002)),
                         seed=hash((_idx, time.time())) & 0xFFFF, drop_start=True)
            lim_r = float(params.get("_poslim_rob", lim))  # bozulmuş turlar için sınır
            if not (r2["lap"] is not None and r2["damage"] == 0 and r2["max_pos"] <= lim_r) or r2["lap"] > r["lap"] + 5:
                r = dict(r2, lap=r2["lap"], reason="robust_" + r2["reason"])
                if r2["lap"] is not None and r2["damage"] == 0:
                    r["max_pos"] = max(r2["max_pos"], lim + 0.01)
                break
            laps.append(r2["lap"])
        else:
            # _lagrob=N: ek N tur kalıcı 1 adım gecikme + başta kaçırma + rastgele kaçırma (gerçek 10 ms koşuları)
            lim_r = float(params.get("_poslim_rob", lim))
            for j in range(int(params.get("_lagrob", 0) or 0)):
                drv, _ = make_driver(params)
                r4 = run_lap(_idx, drv, 10000, offtrack_stop=float(params.get("_offstop", 1.15)), delay1=True,
                             drop_prob=float(params.get("_drop", 0.002)), seed=(hash((_idx, time.time(), j)) & 0xFFFF),
                             drop_start=True)
                if not (r4["lap"] is not None and r4["damage"] == 0 and r4["max_pos"] <= lim_r):
                    r = dict(r4, reason="lagrob_" + r4["reason"])
                    if r4["lap"] is not None and r4["damage"] == 0:
                        r["max_pos"] = max(r4["max_pos"], lim + 0.01)
                    laps = []
                    break
                laps.append(r4["lap"])
            if len(laps) > 1:  # hepsi geçerli: puan = ortalama tur (det + 10 ms turları)
                r["det_lap"] = laps[0]
                r["lap"] = round(sum(laps) / len(laps), 3)
    except Exception as e:  # noqa: BLE001
        r = dict(lap=None, damage=0, max_pos=9, dist=0, reason=f"err {e}")
    ok = r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= lim
    score = r["lap"] if ok else 300 - r["dist"] / 20 + (50 if r["lap"] else 0)
    r.update(score=score, ok=ok, secs=round(time.time() - t0, 1), idx=_idx)
    return params, r


def sample(rng, center=None, sigma=1.0):
    p = {}
    for k, (lo, hi, t) in SPACE.items():
        if center is None:
            x = rng.uniform(lo, hi)
        else:
            x = center[k] + rng.gauss(0, sigma * (hi - lo) * 0.15)
            x = min(hi, max(lo, x))
        if t == "margin":
            x = min(MARGINS, key=lambda m: abs(m - x)) if center is None or rng.random() < 0.3 else center[k]
        p[k] = round(x, 5)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--minutes", type=float, default=60)
    ap.add_argument("--procs", type=int, default=9)
    ap.add_argument("--start", help="başlangıç parametre JSON (merkez)")
    ap.add_argument("--n-random", type=int, default=45)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    runs = os.path.join(HERE, "runs")
    os.makedirs(runs, exist_ok=True)
    out = os.path.join(runs, f"{a.name}.csv")
    assert not os.path.exists(out), f"var olan dosyanın üzerine yazılmaz: {out}"
    best_f = os.path.join(runs, f"{a.name}_best.json")
    from line_driver import DEFAULTS
    base = {k: DEFAULTS[k] for k in SPACE}
    if a.start:
        base.update({k: v for k, v in json.load(open(a.start)).items() if k in SPACE})

    q = mp.Queue()
    for i in range(1, a.procs + 1):
        q.put(i)
    pool = mp.Pool(a.procs, initializer=_init, initargs=(q,))
    results = []
    t_end = time.time() + a.minutes * 60
    pending = []
    n_sent = 0
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["n", "time", "score", "ok", "lap", "damage", "max_pos", "dist", "reason", "secs", "params"])

    def next_params():
        nonlocal n_sent
        n_sent += 1
        if n_sent == 1:
            return dict(base)
        if n_sent <= a.n_random and not a.start:
            return sample(rng)
        top = sorted(results, key=lambda x: x[1]["score"])[:5]
        if not top:
            return sample(rng, base, 1.0)
        c = rng.choice(top)[0]
        sig = 1.0 if len(results) < 100 else 0.6 if len(results) < 300 else 0.35
        return sample(rng, c, sig)

    for _ in range(a.procs * 2):
        pending.append(pool.apply_async(evaluate, (next_params(),)))
    best = None
    while pending:
        done = [p for p in pending if p.ready()]
        if not done:
            time.sleep(0.2)
            continue
        for d in done:
            pending.remove(d)
            params, r = d.get()
            results.append((params, r))
            w.writerow([len(results), time.strftime("%H:%M:%S"), round(r["score"], 3), r["ok"], r["lap"], r["damage"],
                        round(r["max_pos"], 3), round(r["dist"], 1), r["reason"], r["secs"], json.dumps(params)])
            f.flush()
            if best is None or r["score"] < best[1]["score"]:
                best = (params, r)
                json.dump(dict(params, _result=r), open(best_f, "w"), indent=1)
                print(f"[{len(results)}] yeni en iyi {r['score']:.3f} ({r['reason']}, pos {r['max_pos']:.2f})", flush=True)
            if time.time() < t_end:
                pending.append(pool.apply_async(evaluate, (next_params(),)))
        if len(results) % 50 == 0 and done:
            print(f"  {len(results)} deneme, en iyi {best[1]['score']:.3f}", flush=True)
    pool.close()
    pool.join()
    f.close()
    print("bitti", best[1])


if __name__ == "__main__":
    main()
