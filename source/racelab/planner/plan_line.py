"""Yarış çizgisi + hız profili (çevrimdışı, pistin gerçek geometrisinden).

Yarış çizgisi: orta çizgiden yanal kayma n(s) (sol +, metre). Minimum eğrilik:
    min Σ ||P[i+1] - 2 P[i] + P[i-1]||²,  P[i] = C[i] + n[i] N[i],  |n[i]| <= w/2 - pay
P, n'ye göre doğrusal → sınırlı doğrusal en küçük kareler (scipy lsq_linear), tam çözüm.

Hız profili: yanal tutunma (aerodinamik baskıyla hızla artar) → viraj hız sınırı; ileri geçiş
(motor ivmesi), geri geçiş (fren). Parametreler simülasyonda otomatik ayarlanır.
"""
import os

import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import diags, vstack, csr_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81


def load_track(ds=2.0):
    d = np.load(os.path.join(HERE, "track_center.npz"))
    xyz, L = d["xyz"], float(d["L"])
    n0 = len(xyz)
    s0 = np.arange(n0) * L / n0
    m = int(round(L / ds))
    s = np.arange(m) * L / m
    closed = np.vstack([xyz, xyz[:1]])
    sc = np.append(s0, L)
    c = np.stack([np.interp(s, sc, closed[:, i]) for i in range(3)], 1)
    w = np.interp(s, sc, np.append(d["w"], d["w"][0]))
    return s, c, w, L


def normals(xy):
    t = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    return np.stack([-t[:, 1], t[:, 0]], 1)  # sola dik


def curvature(xy):
    d1 = (np.roll(xy, -1, 0) - np.roll(xy, 1, 0)) / 2
    d2 = np.roll(xy, -1, 0) - 2 * xy + np.roll(xy, 1, 0)
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-12)


def min_curvature_line(xy, w, margin=1.5, smooth_iters=1, reg=1e-3):
    """n(s) [m]; margin: kenardan pay (aracın yarı genişliği + güvenlik)."""
    m = len(xy)
    N = normals(xy)
    # ikinci fark operatörü (dairesel)
    D = diags([np.ones(m), -2 * np.ones(m), np.ones(m)], [-1, 0, 1], shape=(m, m), format="lil")
    D[0, m - 1] = 1
    D[m - 1, 0] = 1
    D = csr_matrix(D)
    # D @ (C + n N) = D C + D diag(Nx) n , D diag(Ny) n
    Ax = D @ diags(N[:, 0])
    Ay = D @ diags(N[:, 1])
    A = vstack([Ax, Ay, diags(np.full(m, reg))]).tocsr()
    b = -np.concatenate([D @ xy[:, 0], D @ xy[:, 1], np.zeros(m)])
    lim = w / 2 - margin
    res = lsq_linear(A, b, bounds=(-lim, lim), lsmr_tol="auto", max_iter=5000)
    n = res.x
    return n, xy + n[:, None] * N


def speed_profile(k, ds, z=None, mu=1.6, aero=0.0035, mu_brake=1.4, aero_brake=0.003,
                  acc_fn=None, v_max=90.0, scale=None, kv=None, vert=0.0, bscale=None):
    """k: yolun eğriliği (1/m). Döner: hedef hız (m/s) her noktada.
    Yanal: v²|k| <= mu (g + aero v²)  → v² <= mu g / (|k| - mu aero).
    scale: noktasal çarpan (bölge bazlı ayar)."""
    ak = np.abs(k)
    if kv is not None and vert:  # dikey eğrilik: tepede (kv<0) yük azalır → aero terimine eklenir
        aero = aero + vert * kv
        aero_brake = aero_brake + vert * kv
    aero_a = np.broadcast_to(np.asarray(aero, dtype=float), ak.shape)
    aerob_a = np.broadcast_to(np.asarray(aero_brake, dtype=float), ak.shape)
    denom = ak - mu * aero_a
    v_lat = np.where(denom > 1e-6, np.sqrt(mu * G / np.maximum(denom, 1e-6)), v_max)
    v_lat = np.minimum(v_lat, v_max)
    if scale is not None:
        v_lat = v_lat * scale
    m = len(k)
    if acc_fn is None:
        def acc_fn(v):  # yaklaşık motor ivmesi (m/s²), telemetriyle ayarlanacak
            return max(0.5, 9.0 - 0.09 * v)
    # dairesel profil: iki tur dolaş
    v = v_lat.copy()
    for _ in range(2):  # ileri (ivmelenme)
        for i in range(1, 2 * m):
            a, b = (i - 1) % m, i % m
            lat = (v[a] ** 2 * ak[a]) / (mu * max(G + aero_a[a] * v[a] ** 2, 1.0))
            acc = acc_fn(v[a]) * np.sqrt(max(0.0, 1 - min(lat, 1.0) ** 2))
            v[b] = min(v[b], np.sqrt(v[a] ** 2 + 2 * acc * ds))
    for _ in range(2):  # geri (fren)
        for i in range(2 * m - 1, 0, -1):
            a, b = i % m, (i - 1) % m
            lat = (v[a] ** 2 * ak[a]) / (mu * max(G + aero_a[a] * v[a] ** 2, 1.0))
            dec = mu_brake * max(G + aerob_a[a] * v[a] ** 2, 1.0) * np.sqrt(max(0.0, 1 - min(lat, 1.0) ** 2))
            if bscale is not None:
                dec *= bscale[a]
            v[b] = min(v[b], np.sqrt(v[a] ** 2 + 2 * dec * ds))
    return v


if __name__ == "__main__":
    s, c, w, L = load_track(2.0)
    n, P = min_curvature_line(c[:, :2], w)
    seg = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    print(f"çizgi uzunluğu {seg.sum():.1f} m (orta çizgi {L:.1f}); |n| max {np.abs(n).max():.2f} m")
    kc = curvature(c[:, :2])
    kp = curvature(P)
    print(f"eğrilik max: orta {np.abs(kc).max():.4f}, çizgi {np.abs(kp).max():.4f}")
    v = speed_profile(kp, 2.0)
    print(f"hız profili min {v.min()*3.6:.0f} km/h, ort {v.mean()*3.6:.0f}; tahmini tur {np.sum(seg / v):.2f} s")
