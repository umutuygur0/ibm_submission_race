"""Corkscrew pistinin gerçek geometrisi, TORCS'un 3B modelinden (corkscrew.acc, sadece OKUNUR).

Yol yüzeyi nesneleri (TKMN*) üçgen şeritleri: köşe noktaları (sol, sağ) çiftleri halinde.
Çıktı: racelab/planner/track_center.npz — orta çizgi (x, y, z), genişlik, yay uzunluğu s, eğrilik,
s=0 başlangıç çizgisine hizalanmış (XML segment eğriliğiyle çapraz korelasyon).

    python racelab/planner/track_mesh.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
ACC = os.path.join(ROOT, "torcs", "tracks", "road", "corkscrew", "corkscrew.acc")
OUT = os.path.join(HERE, "track_center.npz")
TRACK_LEN = 3608.2


def read_road_pairs(path=ACC):
    lines = open(path, encoding="latin-1").read().split("\n")
    pairs = []
    i = 0
    while i < len(lines):
        if lines[i].startswith("OBJECT poly"):
            name, verts = None, []
            j = i + 1
            while j < len(lines) and not lines[j].startswith("OBJECT"):
                if lines[j].startswith("name"):
                    name = lines[j].split('"')[1]
                if lines[j].startswith("numvert"):
                    n = int(lines[j].split()[1])
                    verts = [tuple(map(float, lines[j + 1 + k].split()[:3])) for k in range(n)]
                j += 1
            if name and name.startswith("TKMN"):
                for k in range(0, len(verts) - 1, 2):
                    a, b = np.array(verts[k]), np.array(verts[k + 1])
                    # AC3D: (x, yukarı, -y) → TORCS (x, y, z)
                    pa = np.array([a[0], -a[2], a[1]])
                    pb = np.array([b[0], -b[2], b[1]])
                    pairs.append((pa, pb))
            i = j
        else:
            i += 1
    return pairs


def chain(points):
    """Noktaları en yakın komşu ile kapalı bir döngüye diz (tekrarları at)."""
    pts = np.unique(np.round(points, 3), axis=0)
    n = len(pts)
    used = np.zeros(n, bool)
    order = [0]
    used[0] = True
    for _ in range(n - 1):
        d = np.linalg.norm(pts[:, :2] - pts[order[-1], :2], axis=1)
        d[used] = np.inf
        k = int(np.argmin(d))
        if d[k] > 30:  # kopukluk: dur
            break
        order.append(k)
        used[k] = True
    return pts[order]


def resample(path, ds=1.0):
    seg = np.linalg.norm(np.diff(path[:, :2], axis=0, append=path[:1, :2]), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    L = s[-1]
    closed = np.vstack([path, path[:1]])
    t = np.arange(0, L, ds)
    out = np.stack([np.interp(t, s, closed[:, c]) for c in range(closed.shape[1])], 1)
    return out, L


def curvature(xy, ds):
    d1 = (np.roll(xy, -1, 0) - np.roll(xy, 1, 0)) / (2 * ds)
    d2 = (np.roll(xy, -1, 0) - 2 * xy + np.roll(xy, 1, 0)) / ds ** 2
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-9)


def main():
    pairs = read_road_pairs()
    mids = np.array([(a + b) / 2 for a, b in pairs])
    widths = np.array([np.linalg.norm((a - b)[:2]) for a, b in pairs])
    print(f"yol şeridi çifti: {len(pairs)}, genişlik min/ort/max {widths.min():.2f}/{widths.mean():.2f}/{widths.max():.2f}")
    path = chain(mids)
    print(f"zincir: {len(path)} / {len(np.unique(np.round(mids, 3), axis=0))} nokta")
    c, L = resample(path, 1.0)
    print(f"orta çizgi uzunluğu {L:.1f} m (TORCS {TRACK_LEN})")
    k = curvature(c[:, :2], 1.0)
    # yön: XML'deki toplam dönüş +360 (sol pozitif) → saat yönü tersine olmalı
    turn = np.sum(k) * 1.0
    if turn < 0:
        c, k = c[::-1].copy(), -curvature(c[::-1, :2], 1.0)
        k = curvature(c[:, :2], 1.0)
    print(f"toplam dönüş {math.degrees(np.sum(k)):.1f}°")

    # s=0 hizalaması: XML eğriliği (sabit yarıçaplı segmentler doğru) ile çapraz korelasyon
    sys.path.insert(0, os.path.join(ROOT, "racelab", "analysis"))
    from track_profile import load_segments, curvature_map
    ds_x, ks_x, _ = curvature_map(load_segments(), 1.0)
    n = len(k)
    kx = np.interp(np.arange(n) * (ds_x[-1] / n), ds_x, ks_x)
    ksm = np.convolve(np.concatenate([k[-10:], k, k[:10]]), np.ones(21) / 21, "valid")
    best = max(range(n), key=lambda sh: float(np.dot(np.roll(ksm, -sh)[:600], kx[:600])))
    c = np.roll(c, -best, 0)
    k = np.roll(ksm, -best)
    # genişlik: her orta çizgi noktasına en yakın şerit çiftinin genişliği
    w = np.array([widths[np.argmin(np.linalg.norm(mids[:, :2] - p[:2], axis=1))] for p in c])
    np.savez(OUT, xyz=c, k=k, w=w, L=L, s=np.arange(len(c)) * L / len(c))
    print(f"kaydedildi {OUT}; kaydırma {best} m; s=0 noktası {c[0]}")
    for s0 in range(0, n, 300):
        print(f"  s={s0:5d}  k={k[s0]:+.4f} (xml {kx[s0]:+.4f})  z={c[s0, 2]:7.2f}  w={w[s0]:.2f}")


if __name__ == "__main__":
    main()
