"""Hibrit teslim dosyası: yarış çizgisi sürücüsü (build_line_submission) + artık RL düzeltmesi (SAC aktör ağırlıkları gömülü).

    python racelab/planner/build_res_submission.py --params racelab/planner/runs/snap_gr3L.json \
        --actor racelab/rl/runs/res3/candidates/vXXXX.pt --run-config racelab/rl/runs/res3/config.json \
        --out racelab/submission/torcs_jm_par_hybrid_res3.py

Gözlem ve düzeltme rl/residual_env.py (res_obs, apply_residual) ile birebir aynıdır; eşdeğerlik run_benchmark ile doğrulanır.
Aktör ileri geçişi numpy varsa numpy, yoksa saf Python.
"""
import argparse
import base64
import json
import os
import subprocess
import sys
import zlib

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

RES_BLOCK = r'''
# =====================================================================
# ARTIK RL DÜZELTMESİ (racelab/planner/build_res_submission.py ile üretildi)
# Aktör: __ACTOR__   (gözlem 28 → 256 → 256 → 4, deterministik tanh(mu))
# Temel komut yukarıdaki yarış çizgisi sürücüsünden; RL direksiyona ±__DS__, pedala ±__DP__ ekler.
# =====================================================================
try:
    import numpy as _rnp
except ImportError:
    _rnp = None

_RES_SHAPES = __SHAPES__
_RES_BLOB = (
__BLOB__
)
_RES_DS, _RES_DP = __DS__, __DP__
_RES_TCS = __TCS__


def _res_load():
    raw = _zlib.decompress(_b64.b64decode("".join(_RES_BLOB)))
    flat = _array("f")
    flat.frombytes(raw)
    out, i = [], 0
    for rows, cols in _RES_SHAPES:
        n = rows * cols
        part = flat[i:i + n]
        i += n
        if _rnp is not None:
            out.append(_rnp.array(part, dtype=_rnp.float32).reshape(rows, cols) if cols > 1
                       else _rnp.array(part, dtype=_rnp.float32))
        else:
            out.append([list(part[r * cols:(r + 1) * cols]) for r in range(rows)] if cols > 1 else list(part))
    return out


_RES_W = _res_load()
_RES_PREV = [0.0, 0.0]


def _res_policy(obs):
    Ws, bs = _RES_W[0::2], _RES_W[1::2]
    if _rnp is not None:
        h = _rnp.asarray(obs, dtype=_rnp.float32)
        for W, b in zip(Ws[:-1], bs[:-1]):
            h = _rnp.maximum(W @ h + b, 0)
        y = Ws[-1] @ h + bs[-1]
        return [float(_rnp.tanh(y[0])), float(_rnp.tanh(y[1]))]
    h = obs
    for W, b in zip(Ws[:-1], bs[:-1]):
        h = [max(sum(x * y for x, y in zip(row, h)) + c, 0.0) for row, c in zip(W, b)]
    y = [sum(x * y for x, y in zip(row, h)) + c for row, c in zip(Ws[-1][:2], bs[-1][:2])]
    return [_math.tanh(y[0]), _math.tanh(y[1])]


def _res_obs(S, base_steer, base_pedal, prev):
    p = _LP
    s = S['distFromStart'] + p["offset"]
    v = max(S['speedX'] / 3.6, 0.0)
    half_w = _lp_interp("w", s) / 2
    e = _lp_interp("n", s) - S['trackPos'] * half_w
    heading = _lp_theta("theta", s) - S['angle']
    dh = (_lp_theta("theta_p", s + 4.0) - heading + _math.pi) % (2 * _math.pi) - _math.pi
    vt = _lp_interp("v", s)
    ks = [_lp_interp("k", s + d) * 40.0 for d in (5.0, 20.0, 45.0, 80.0)]
    w = S['wheelSpinVel']
    spin = ((w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) - (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1])) / 2
    ph = 2 * _math.pi * (S['distFromStart'] % _LL) / _LL
    harm = []
    for k in (1, 2, 4, 8, 16):
        harm += [_math.sin(k * ph), _math.cos(k * ph)]
    o = [v / 80.0, S['trackPos'], S['angle'] * 3.0, e / 3.0, dh * 3.0, (vt - v) / 20.0] + ks + [
        base_steer, base_pedal, S.get('speedY', 0.0) / 30.0, spin / 10.0, S['rpm'] / 20000.0, S['gear'] / 6.0,
        prev[0], prev[1]] + harm
    return [max(-5.0, min(5.0, x)) for x in o]


def res_drive(c):
    global _RES_PREV
    S, R = c.S.d, c.R.d
    line_drive(c)  # temel komut
    b_steer, b_acc, b_brk = R['steer'], R['accel'], R['brake']
    a = _res_policy(_res_obs(S, b_steer, b_acc - b_brk, _RES_PREV))
    a = [max(-1.0, min(1.0, a[0])), max(-1.0, min(1.0, a[1]))]
    steer = max(-1.0, min(1.0, b_steer + a[0] * _RES_DS))
    pedal = max(-1.0, min(1.0, b_acc - b_brk + a[1] * _RES_DP))
    accel, brake = max(pedal, 0.0), max(-pedal, 0.0)
    w = S['wheelSpinVel']
    if accel > 0 and a[1] > 0:  # artırılan gaz için patinaj kontrolü
        spin = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) - (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1])
        tc = accel
        if spin > _RES_TCS:
            tc = max(0.0, accel - min(accel, (spin - _RES_TCS) * 0.1))
        accel = min(accel, max(b_acc, tc))
    if brake > 0 and S['speedX'] >= 10:  # ABS
        slip = S['speedX'] / 3.6 - sum(x * r for x, r in zip(w, _WHEEL_R)) / 4.0
        if slip > 2.0:
            brake = brake - min(brake, (slip - 2.0) * 0.1)
    R['steer'], R['accel'], R['brake'] = steer, accel, brake
    _RES_PREV = a

'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--actor", required=True)
    ap.add_argument("--run-config", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    assert not os.path.exists(a.out), f"var olan dosyanın üzerine yazılmaz: {a.out}"
    cfg = json.load(open(a.run_config))
    tmp = a.out + ".line_tmp.py"
    assert not os.path.exists(tmp)
    subprocess.run([sys.executable, os.path.join(HERE, "build_line_submission.py"), "--params", a.params, "--out", tmp],
                   check=True)
    src = open(tmp, encoding="utf-8").read()
    os.remove(tmp)
    ck = torch.load(a.actor, map_location="cpu", weights_only=False)
    sd = ck["actor"]
    n_lin = len([k for k in sd if k.endswith(".weight")])
    keys = [f"net.{2 * i}.{t}" for i in range(n_lin) for t in ("weight", "bias")]
    arrays = [sd[k].numpy().astype(np.float32) for k in keys]
    shapes = [(x.shape[0], x.shape[1]) if x.ndim == 2 else (x.shape[0], 1) for x in arrays]
    blob = base64.b64encode(zlib.compress(np.concatenate([x.ravel() for x in arrays]).tobytes(), 9)).decode()
    lines = "\n".join(f'    "{blob[i:i + 100]}"' for i in range(0, len(blob), 100))
    from build_line_submission import load_params
    p = load_params(a.params)
    block = (RES_BLOCK.replace("__ACTOR__", os.path.relpath(os.path.abspath(a.actor), ROOT).replace("\\", "/"))
             .replace("__SHAPES__", repr(shapes)).replace("__BLOB__", lines)
             .replace("__DS__", repr(float(cfg["d_steer"]))).replace("__DP__", repr(float(cfg["d_pedal"])))
             .replace("__TCS__", repr(float(p.get("tcs", 6.75)))))
    old = "# ================= MAIN DRIVE FUNCTION ================="
    assert src.count(old) == 1
    src = src.replace(old, block + old)
    old = "    line_drive(c)\n    return"
    assert src.count(old) == 1
    src = src.replace(old, "    res_drive(c)  # yarış çizgisi + artık RL düzeltmesi\n    return")
    src = src.replace("# Bu dosya racelab/planner/build_line_submission.py tarafından üretildi.",
                      "# Bu dosya racelab/planner/build_res_submission.py tarafından üretildi (hibrit: çizgi + artık RL).", 1)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(src)
    print("yazıldı:", a.out, f"({os.path.getsize(a.out) // 1024} KB)")


if __name__ == "__main__":
    main()
