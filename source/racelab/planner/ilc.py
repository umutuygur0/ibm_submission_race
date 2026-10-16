"""Yinelemeli öğrenen kontrol (ILC): konuma bağlı direksiyon düzeltme tablosu.

Her yinelemede tur sürülür, plan ızgarasında yanal hata e(s) = n_hedef − n_araç (m) ölçülür,
düzeltme ilc(s − öne_alma) += γ · süzülmüş e(s). Deterministik simülasyonda hata turdan tura azalır.
En iyi (geçerli, en hızlı; eşitlikte en düşük hata) tablo kaydedilir.

    python racelab/planner/ilc.py --params runs/z5irls_best.json --name i1 --iters 12 --idx 9
Çıktı: planner/ilc/<ad>_best.npy, planner/runs/<ad>_best.json (ilc_file ile)
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build_line_submission import load_params  # noqa: E402
from sim import run_lap  # noqa: E402
from try_one import make_driver  # noqa: E402

POS_LIM = 1.0


def lateral_error(tele, plan):
    """Plan ızgarasında (2 m) ortalama yanal hata (m); örneği olmayan noktalar 0."""
    s = tele[:, 0] % plan["L"]
    half_w = np.interp(s, plan["s"], plan["w"]) / 2
    e = np.interp(s, plan["s"], plan["n"]) - tele[:, 2] * half_w
    m = len(plan["s"])
    idx = np.minimum((s / 2.0).astype(int), m - 1)
    acc = np.bincount(idx, weights=e, minlength=m)
    cnt = np.bincount(idx, minlength=m)
    out = np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0)
    return out, cnt > 0


def smooth(x, k=9):
    pad = k // 2
    return np.convolve(np.concatenate([x[-pad:], x, x[:pad]]), np.ones(k) / k, "valid")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--iters", type=int, default=12)
    ap.add_argument("--idx", type=int, default=9)
    ap.add_argument("--gamma", type=float, default=0.008, help="rad / m")
    ap.add_argument("--lead", type=float, default=8.0, help="düzeltmeyi kaç m önce uygula")
    ap.add_argument("--init", help="başlangıç ilc .npy")
    a = ap.parse_args()
    os.makedirs(os.path.join(HERE, "ilc"), exist_ok=True)
    out_best = os.path.join(HERE, "ilc", f"{a.name}_best.npy")
    assert not os.path.exists(out_best), out_best
    p = load_params(a.params)
    p["ctrl"] = "st"
    drv, plan = make_driver(p)
    m = len(plan["s"])
    ilc = np.load(a.init) if a.init else np.zeros(m)
    lead = int(round(a.lead / 2.0))
    best = None
    for it in range(a.iters + 1):
        f = os.path.join(HERE, "ilc", f"{a.name}_it{it:02d}.npy")
        np.save(f, ilc)
        p["ilc_file"] = f
        drv, plan = make_driver(p)
        r = run_lap(a.idx, drv, 1000000, record=True, offtrack_stop=1.3)
        tele = r.pop("tele")
        e, seen = lateral_error(tele[tele[:, 0] < 3700], plan)
        rms = float(np.sqrt(np.mean(e[seen] ** 2)))
        ok = r["lap"] is not None and r["damage"] == 0 and r["max_pos"] <= POS_LIM
        print(f"it {it:2d}: tur {r['lap']} hasar {r['damage']} |pos| {r['max_pos']:.3f} hata rms {rms:.3f} m "
              f"max {np.abs(e).max():.2f} ({r['reason']})", flush=True)
        key = (0 if ok else 1, r["lap"] or 999, rms)
        if best is None or key < best[0]:
            best = (key, ilc.copy(), r)
        es = smooth(np.where(seen, e, 0.0))
        ilc = np.clip(ilc + a.gamma * np.roll(es, -lead), -0.12, 0.12)
    np.save(out_best, best[1])
    dp = json.load(open(a.params))
    dp["ilc_file"] = out_best
    dp["_result"] = best[2]
    json.dump(dp, open(os.path.join(HERE, "runs", f"{a.name}_best.json"), "w"), indent=1)
    print("en iyi:", best[0], "→", out_best)


if __name__ == "__main__":
    main()
