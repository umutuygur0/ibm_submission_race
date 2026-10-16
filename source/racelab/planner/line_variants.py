"""Yarış çizgisi varyantları (çevrimdışı karşılaştırma + önbelleğe yazma).

Varyantlar:
  mc   : minimum eğrilik (Σ ikinci fark²) + küçük düzenleme (plan_line.min_curvature_line)
  mc0  : düzenlemesiz
  irls : yinelemeli yeniden ağırlıklandırma — yüksek eğrilikli noktalara daha çok ağırlık (minimax'a yaklaşır)
Her biri için: tahmini tur süresi (sabit fizik parametreleriyle hız profili), viraj apeks kullanımları.

    python racelab/planner/line_variants.py
"""
import os
import sys

import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import csr_matrix, diags, vstack

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from plan_line import curvature, load_track, normals, speed_profile  # noqa: E402

CACHE = os.path.join(HERE, "cache")


def solve(xy, w, margin, reg=1e-3, weights=None):
    m = len(xy)
    N = normals(xy)
    D = diags([np.ones(m), -2 * np.ones(m), np.ones(m)], [-1, 0, 1], shape=(m, m), format="lil")
    D[0, m - 1] = 1
    D[m - 1, 0] = 1
    D = csr_matrix(D)
    W = diags(np.ones(m) if weights is None else weights)
    Ax = W @ D @ diags(N[:, 0])
    Ay = W @ D @ diags(N[:, 1])
    rows = [Ax, Ay]
    b = [-(W @ (D @ xy[:, 0])), -(W @ (D @ xy[:, 1]))]
    if reg > 0:
        rows.append(diags(np.full(m, reg)))
        b.append(np.zeros(m))
    A = vstack(rows).tocsr()
    bb = np.concatenate(b)
    lim = w / 2 - margin
    from scipy.optimize import minimize

    def fg(x):
        r = A @ x - bb
        return float(r @ r), 2 * (A.T @ r)
    x0 = np.zeros(m)
    res = minimize(fg, x0, jac=True, method="L-BFGS-B", bounds=list(zip(-lim, lim)),
                   options=dict(maxiter=20000, maxfun=40000, ftol=1e-15, gtol=1e-12))
    n = res.x
    return n, xy + n[:, None] * N


def est_lap(P, ds=2.0):
    k = curvature(P)
    k = np.convolve(np.concatenate([k[-3:], k, k[:3]]), np.ones(7) / 7, "valid")
    seg = np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)
    v = speed_profile(k, ds, mu=1.9, aero=0.0030, mu_brake=2.3, aero_brake=0.0012,
                      acc_fn=lambda v: float(np.interp(v, [0, 10, 30, 45, 55, 65, 75, 82, 90],
                                                       [12, 11.8, 11, 11.5, 8.8, 7.2, 4.2, 1, 0])) * 1.3)
    return float(np.sum(seg / v)), k


def main():
    s, c, w, L = load_track(2.0)
    xy = c[:, :2]
    os.makedirs(CACHE, exist_ok=True)
    for margin in (1.5, 1.0):
        out = {}
        n, P = solve(xy, w, margin, 1e-3)
        out["mc"] = (n, P)
        n, P = solve(xy, w, margin, 0.0)
        out["mc0"] = (n, P)
        wts = np.ones(len(xy))
        for it in range(6):
            n, P = solve(xy, w, margin, 0.0, wts)
            k = np.abs(curvature(P))
            ks = np.convolve(np.concatenate([k[-10:], k, k[:10]]), np.ones(21) / 21, "valid")
            wts = 1.0 + 4.0 * (ks / ks.max())
        out["irls"] = (n, P)
        for name, (n, P) in out.items():
            t, k = est_lap(P)
            i = np.searchsorted(s, 1922)
            print(f"pay {margin}: {name:5s} tahmini tur {t:6.2f} s  uzunluk {np.sum(np.linalg.norm(np.roll(P,-1,0)-P,axis=1)):7.1f}"
                  f"  max|k| {np.abs(k).max():.4f}  1922'de n={n[i]:+.2f} k={k[i]:+.4f}")
            np.savez(os.path.join(CACHE, f"line_{name}_m{margin:.2f}.npz"), n=n, P=P)


if __name__ == "__main__":
    main()
