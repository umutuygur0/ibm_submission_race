"""Paralel viraj viraj ızgara araması: her bölge için (hız çarpanı × kazanç çarpanları [yön, yanal, ön besleme]) ve
ardından fren çarpanı. Yerel geçerlilik (bölge + 150 m): det |pos| ≤ lim ve hasarsız, delay1 |pos| ≤ lim_rob ve hasarsız.
Geçerliler arasında bölge sonundaki süreye göre en hızlısı seçilir. 9 paralel (idx 1–9).

    python racelab/planner/greedy2.py --start runs/x.json --name gr1 --passes 2 [--set cl_max=0.66 ...]
"""
import argparse
import copy
import itertools
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
    c, lo, hi, lim, lim_rob = args[:5]
    nprob, prel = (args[5], args[6]) if len(args) > 5 else (0, 0.0)
    from greedy_corners import lap_tele, local
    try:
        r1, t1 = lap_tele(_idx, c, False)
        p1, d1, tt = local(r1, t1, lo, hi)
        r2, t2 = lap_tele(_idx, c, True)
        p2, d2, _ = local(r2, t2, lo, hi)
    except Exception as e:  # noqa: BLE001
        return False, 999.0, (9, 9, 9, 9), {"lap": None, "err": str(e)}
    ok = p1 <= lim and d1 == 0 and p2 <= lim_rob and d2 == 0
    # tüm tur da bitmeli ve hasarsız olmalı (yerel iyi ama sonra pistten çıkaran adayları ele)
    ok = ok and r1["lap"] is not None and r1["damage"] == 0 and r2["lap"] is not None and r2["damage"] == 0
    nstress = args[7] if len(args) > 7 else 0
    if ok and nstress:  # stres: rastgele adım kaçırma (+ bir tur kalıcı gecikmeyle), tüm tur hasarsız ve |pos| ≤ lim_rob
        from sim import run_lap
        from try_one import make_driver
        from zone_search2 import to_params
        import random
        base_seed = random.randrange(1 << 30)
        for j in range(nstress + 1):
            drv, _ = make_driver(to_params(c))
            rs = run_lap(_idx, drv, 10000, drop_prob=0.002, seed=base_seed + j, drop_start=True, delay1=(j == nstress),
                         offtrack_stop=1.2)
            if not (rs["lap"] is not None and rs["damage"] == 0 and rs["max_pos"] <= lim_rob):
                ok = False
                break
    if ok and nprob:  # parametre sağlamlığı: ±prel oynatılmış det turlar da (yerel) hasarsız olmalı
        import random
        rng = random.Random(hash((lo, tt)) & 0xFFFF)
        for _ in range(nprob):
            q = copy.deepcopy(c)
            for k, v in c.items():
                if (not k.startswith("_") and isinstance(v, (int, float)) and not isinstance(v, bool)
                        and k not in ("margin", "max_gear", "launch", "offset", "v_max", "st_csign")):
                    q[k] = v * (1 + rng.uniform(-prel, prel))
            q["_z"] = [[z[0], z[1], z[2] * (1 + rng.uniform(-prel, prel)), z[3]] for z in c["_z"]]
            q["_bz"] = [b * (1 + rng.uniform(-prel, prel)) for b in c["_bz"]]
            q["_kz"] = [[f * (1 + rng.uniform(-prel, prel)) for f in k3] for k3 in c["_kz"]]
            try:
                r3, t3 = lap_tele(_idx, q, False)
                p3, d3, _ = local(r3, t3, lo, hi)
            except Exception:  # noqa: BLE001
                return False, 999.0, (9, 9, 9, 9), {"lap": None}
            if not (p3 <= lim_rob and d3 == 0):
                ok = False
                break
    return ok, tt, (p1, p2, d1, d2), {"lap": r1["lap"], "delay1": r2["lap"], "pos": r1["max_pos"], "pos_d1": r2["max_pos"],
                                      "dmg": r1["damage"], "dmg_d1": r2["damage"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--lim", type=float, default=0.995)
    ap.add_argument("--lim-rob", type=float, default=1.06)
    ap.add_argument("--procs", type=int, default=9)
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--shift", action="store_true", help="apeks kaydırma aşaması da")
    ap.add_argument("--prob", type=int, default=0, help="parametre sağlamlığı: aday başına ek oynatılmış tur sayısı")
    ap.add_argument("--prel", type=float, default=0.007)
    ap.add_argument("--stress", type=int, default=0, help="aday başına rastgele kaçırma turu (+1 gecikmeli tur)")
    a = ap.parse_args()
    out = os.path.join(HERE, "runs", f"{a.name}_best.json")
    assert not os.path.exists(out), out
    d = json.load(open(a.start))
    cur = {k: v for k, v in d.items() if not k.startswith("_")}
    for it in a.set:
        k, v = it.split("=", 1)
        cur[k] = json.loads(v)
    cur.update(_z=d["_zones"], _bz=d["_bz"], _ns=d.get("_ns") or [0.0] * len(d["_zones"]), ctrl="st",
               _kz=d.get("_kz") or [[1.0, 1.0, 1.0] for _ in d["_zones"]])
    nz = len(cur["_z"])
    q = mp.Queue()
    for i in range(1, a.procs + 1):
        q.put(i)
    pool = mp.Pool(a.procs, initializer=_init, initargs=(q,))

    def save(info):
        json.dump(dict({k: v for k, v in cur.items() if not k.startswith("_")}, _zones=cur["_z"], _bz=cur["_bz"],
                       _ns=cur["_ns"], _kz=cur["_kz"], _result=info), open(out, "w"), indent=1)

    for ps in range(a.passes):
        for zi in range(nz):
            lo, hi = cur["_z"][zi][0], cur["_z"][zi][1] + 150
            f0, (h0, e0, ff0) = cur["_z"][zi][2], cur["_kz"][zi]
            cands = []
            for sm, fh, fe, ff in itertools.product([0.97, 1.0, 1.06, 1.12, 1.2, 1.3], [0.6, 0.8, 1.0, 1.25],
                                                    [0.6, 1.0, 1.5], [0.9, 1.0, 1.15, 1.3]):
                c = copy.deepcopy(cur)
                c["_z"][zi][2] = round(f0 * sm, 4)
                c["_kz"][zi] = [round(h0 * fh, 4), round(e0 * fe, 4), round(ff0 * ff, 4)]
                cands.append(c)
            res = pool.map(job, [(c, lo, hi, a.lim, a.lim_rob, a.prob, a.prel, a.stress) for c in cands])
            ok = [(tt, c, info) for c, (o, tt, _, info) in zip(cands, res) if o]
            if ok:
                tt, cur, info = min(ok, key=lambda x: x[0])
                save(info)
                print(f"geçiş {ps} bölge {zi} ({cur['_z'][zi][3]:.0f}) hız/kazanç: f={cur['_z'][zi][2]} kz={cur['_kz'][zi]} "
                      f"t={tt:.3f} tur {info['lap']} delay1 {info['delay1']} ({len(ok)}/{len(cands)} geçerli)", flush=True)
            # apeks kaydırma (m, sol +)
            if a.shift:
                n0 = cur["_ns"][zi]
                cands = []
                for dn in [-1.5, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 1.5]:
                    c = copy.deepcopy(cur)
                    c["_ns"][zi] = round(n0 + dn, 3)
                    cands.append(c)
                res = pool.map(job, [(c, lo, hi, a.lim, a.lim_rob, a.prob, a.prel, a.stress) for c in cands])
                ok = [(tt, c, info) for c, (o, tt, _, info) in zip(cands, res) if o]
                if ok:
                    tt, cur, info = min(ok, key=lambda x: x[0])
                    save(info)
                    print(f"geçiş {ps} bölge {zi} apeks: ns={cur['_ns'][zi]} t={tt:.3f} tur {info['lap']} delay1 {info['delay1']}",
                          flush=True)
            # fren çarpanı
            b0 = cur["_bz"][zi]
            cands = []
            for bm in [0.85, 0.95, 1.0, 1.1, 1.25, 1.45, 1.7, 2.0]:
                c = copy.deepcopy(cur)
                c["_bz"][zi] = round(b0 * bm, 4)
                cands.append(c)
            res = pool.map(job, [(c, lo, hi, a.lim, a.lim_rob, a.prob, a.prel, a.stress) for c in cands])
            ok = [(tt, c, info) for c, (o, tt, _, info) in zip(cands, res) if o]
            if ok:
                tt, cur, info = min(ok, key=lambda x: x[0])
                save(info)
                print(f"geçiş {ps} bölge {zi} fren: bz={cur['_bz'][zi]} t={tt:.3f} tur {info['lap']} delay1 {info['delay1']}",
                      flush=True)
        print(f"== geçiş {ps} bitti", flush=True)
    pool.close()


if __name__ == "__main__":
    main()
