"""Zorla gecikme stres testi: her aday için N tur, kalıcı 1 adım gecikme + yarış başında kaçırma + rastgele adım kaçırma
(run_benchmark 10 ms koşullarındaki en kötü durum; adaydan bağımsız olarak her birine aynı şekilde uygulanır). 9 paralel.

    python racelab/planner/lag_stress.py --params runs/a.json runs/b.json --n 30
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

_idx = None


def _init(q):
    global _idx
    _idx = q.get()


def job(args):
    path, seed, mode = args
    from build_line_submission import load_params
    from sim import run_lap
    from try_one import make_driver
    p = load_params(path)
    p["ctrl"] = "st"
    drv, _ = make_driver(p)
    if mode == "lag":
        r = run_lap(_idx, drv, 10000, delay1=True, drop_prob=0.002, seed=seed, drop_start=True, offtrack_stop=1.6)
    else:
        r = run_lap(_idx, drv, 10000, drop_prob=0.002, seed=seed, drop_start=True, offtrack_stop=1.6)
    return path, mode, r["lap"], r["damage"], r["max_pos"], r["reason"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", nargs="+", required=True)
    ap.add_argument("--n", type=int, default=30)
    a = ap.parse_args()
    q = mp.Queue()
    for i in range(1, 10):
        q.put(i)
    pool = mp.Pool(9, initializer=_init, initargs=(q,))
    jobs = [(p, s, m) for p in a.params for m in ("lag", "drop") for s in range(a.n)]
    res = pool.map(job, jobs)
    for p in a.params:
        for m in ("lag", "drop"):
            rr = [r for r in res if r[0] == p and r[1] == m]
            ok = [r for r in rr if r[2] is not None and r[3] == 0]
            on = [r for r in ok if r[4] <= 1.0]
            laps = [r[2] for r in ok]
            print(f"{os.path.basename(p):28s} {m:4s}: hasarsız {len(ok)}/{len(rr)}, pist içi {len(on)}/{len(rr)}, "
                  f"tur ort {sum(laps)/max(len(laps),1):.3f}, max|pos| {max(r[4] for r in rr):.3f}", flush=True)
    pool.close()


if __name__ == "__main__":
    main()
