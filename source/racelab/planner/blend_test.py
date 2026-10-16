"""İki parametre seti arasında doğrusal karıştırma; det + delay1 testi (8 paralel).

    python racelab/planner/blend_test.py runs/A.json runs/B.json
"""
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import search  # noqa: E402
from zone_search2 import to_params  # noqa: E402


def blend(A, B, t):
    c = {}
    for k, v in A.items():
        if k.startswith("_"):
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool) and isinstance(B.get(k), (int, float)):
            c[k] = v * (1 - t) + B[k] * t
        else:
            c[k] = v
    c["_z"] = [[za[0], za[1], za[2] * (1 - t) + zb[2] * t, za[3]] for za, zb in zip(A["_zones"], B["_zones"])]
    c["_bz"] = [a * (1 - t) + b * t for a, b in zip(A["_bz"], B["_bz"])]
    c["_ns"] = [a * (1 - t) + b * t for a, b in zip(A["_ns"], B["_ns"])]
    c.update(_delay1=True, _poslim=1.0, _poslim_rob=9.0, ctrl="st", _tag=t)
    return c


def main():
    A = json.load(open(sys.argv[1]))
    B = json.load(open(sys.argv[2]))
    ts = [0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.0]
    cands = [blend(A, B, t) for t in ts]
    q = mp.Queue()
    for i in range(1, 9):
        q.put(i)
    pool = mp.Pool(8, initializer=search._init, initargs=(q,))
    res = pool.map(search.evaluate, [to_params(c) for c in cands])
    for c, (_, r) in zip(cands, res):
        print(c["_tag"], "det", r.get("det_lap", r["lap"]), "pos", round(r["max_pos"], 3), "| delay1", r.get("delay1_lap"),
              "pos", round(r.get("delay1_pos", 9), 3), "hasar", r["damage"], r["reason"], flush=True)
        out = {k: v for k, v in c.items() if not k.startswith("_")}
        out.update(_zones=c["_z"], _bz=c["_bz"], _ns=c["_ns"])
        json.dump(out, open(os.path.join(HERE, "runs", f"blend_{c['_tag']:.2f}.json"), "w"), indent=1)
    pool.close()


if __name__ == "__main__":
    main()
