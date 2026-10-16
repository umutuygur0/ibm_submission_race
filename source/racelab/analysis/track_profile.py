"""Corkscrew eğrilik haritası + bir koşunun telemetrisiyle viraj viraj karşılaştırma.

    python racelab/analysis/track_profile.py --run <results/runs/... klasörü>

Pist XML'i (torcs/tracks/road/corkscrew/corkscrew.xml) sadece OKUNUR.
Çıktı: her viraj için giriş/en düşük/çıkış hızı, kullanılan yanal ivme (v²/R), fren başlangıcı.
"""
import argparse
import csv
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TRACK = os.path.join(ROOT, "torcs", "tracks", "road", "corkscrew", "corkscrew.xml")


def load_segments(path=TRACK):
    txt = open(path, encoding="utf-8", errors="replace").read()
    txt = re.sub(r"<!DOCTYPE.*?]>", "", txt, flags=re.S)  # çok satırlı iç DTD
    txt = re.sub(r"&[\w.-]+;", "", txt)  # &default-surfaces; gibi dış varlıklar
    root = ET.fromstring(txt)
    segs_node = None
    for sec in root.iter("section"):
        if sec.get("name") == "Track Segments":
            segs_node = sec
            break
    segs = []
    for s in segs_node.findall("section"):
        a = {e.get("name"): e.get("val") for e in s if e.tag in ("attnum", "attstr")}
        typ = a.get("type")
        if typ == "str":
            L, r0, r1 = float(a["lg"]), math.inf, math.inf
        else:
            arc = math.radians(float(a["arc"]))
            r0 = float(a["radius"])
            r1 = float(a.get("end radius", r0))
            L = arc * (r0 + r1) / 2
        segs.append({"name": s.get("name"), "type": typ, "len": L, "r0": r0, "r1": r1})
    return segs


def curvature_map(segs, step=1.0):
    """Her metre için işaretli eğrilik (sol +, sağ -)."""
    ds, ks = [], []
    pos = 0.0
    for s in segs:
        n = max(int(s["len"] / step), 1)
        for i in range(n):
            t = (i + 0.5) / n
            if s["type"] == "str":
                k = 0.0
            else:
                r = s["r0"] + (s["r1"] - s["r0"]) * t
                k = (1.0 / r) * (1 if s["type"] == "lft" else -1)
            ds.append(pos + (i + 0.5) * s["len"] / n)
            ks.append(k)
        pos += s["len"]
    return np.array(ds), np.array(ks), pos


def corners(ds, ks, min_k=1 / 400.0, merge_gap=15):
    """Eğriliği belirgin bölgeleri viraj olarak grupla."""
    on = np.abs(ks) > min_k
    out, i = [], 0
    while i < len(on):
        if on[i]:
            j = i
            while j + 1 < len(on) and (on[j + 1] or (j + merge_gap < len(on) and on[j + 1:j + merge_gap].any())):
                j += 1
            out.append((ds[i], ds[j], float(np.max(np.abs(ks[i:j + 1])))))
            i = j + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="results/runs/<run_id> klasörü (telemetry.csv içeren)")
    args = ap.parse_args()
    segs = load_segments()
    ds, ks, total = curvature_map(segs)
    print(f"pist: {len(segs)} segment, merkez hattı uzunluğu {total:.1f} m")

    rows = list(csv.DictReader(open(os.path.join(args.run, "telemetry.csv"), encoding="utf-8")))
    dist = np.array([float(r["distRaced"]) for r in rows])
    dfs = np.array([float(r["distFromStart"]) for r in rows])
    v = np.array([float(r["speedX"]) for r in rows])
    acc = np.array([float(r["act_accel"] or 0) for r in rows])
    brk = np.array([float(r["act_brake"] or 0) for r in rows])
    pos = np.array([float(r["trackPos"]) for r in rows])
    m = dist > 5  # start çizgisinden sonrası (dfs pist üzerindeki konum)
    dfs, v, acc, brk, pos = dfs[m], v[m], acc[m], brk[m], pos[m]
    kk = np.interp(dfs, ds, np.abs(ks))
    alat = (v / 3.6) ** 2 * kk / 9.81  # g cinsinden (merkez hattı yarıçapıyla, yaklaşık)

    print(f"\n{'viraj':>5} {'aralık (m)':>13} {'min R':>6} {'giriş':>6} {'min v':>6} {'çıkış':>6} "
          f"{'maks lat g':>10} {'fren>0.3 başl.':>14} {'süre s':>7}")
    for n, (a, b, kmax) in enumerate(corners(ds, ks), 1):
        sel = (dfs >= a - 5) & (dfs <= b + 5)
        if not sel.any():
            continue
        pre = (dfs >= a - 250) & (dfs < a)
        bstart = dfs[pre][brk[pre] > 0.3]
        bs = f"{a - bstart.min():.0f} m önce" if len(bstart) else "-"
        ent = v[np.argmin(np.abs(dfs - a))]
        ext = v[np.argmin(np.abs(dfs - b))]
        print(f"{n:>5} {a:6.0f}-{b:<6.0f} {1 / kmax:6.0f} {ent:6.0f} {v[sel].min():6.0f} {ext:6.0f} "
              f"{alat[sel].max():10.2f} {bs:>14} {sel.sum() * 0.02:7.2f}")
    print(f"\nyanal ivme (g): ort {alat[kk > 1 / 400].mean():.2f}, %95 {np.percentile(alat[kk > 1 / 400], 95):.2f}, "
          f"maks {alat.max():.2f}")
    print(f"tam gaz (>0.95) oranı {np.mean(acc > 0.95):.2f}, fren oranı {np.mean(brk > 0):.2f}, "
          f"ne gaz ne fren (<0.3, 0) {np.mean((acc < 0.3) & (brk == 0)):.2f}")


if __name__ == "__main__":
    main()
