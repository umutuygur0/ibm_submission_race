"""Teslim dosyası üretici: gym_torcs/torcs_jm_par.py iskeleti + gömülü RL politikası.

    python racelab/submission/build_submission.py --model racelab/rl/snapshots/v2_ft_v05581_best.pt
    → racelab/submission/torcs_jm_par_rl.py

Orijinal torcs_jm_par.py sadece OKUNUR; hiçbir mevcut dosya değişmez. Üretilen dosyada:
- REQUIRED bloklar (port, --parameters, get_servers_input) aynen korunur.
- init açıları modelin eğitildiği 19 mesafe sensörü açısına çevrilir.
- drive_modular → RL politikası (ağırlıklar dosyaya gömülü; numpy varsa numpy, yoksa saf Python).
- Ana döngüye kuyruk boşaltma eklenir (geç kalınan adımdan sonra kalıcı gecikmeyi önler).
- Vites/ABS/patinaj kuralları rl/common.py ile aynı; vites ≤ 6 (DriverAction 7'yi kabul etmez).
"""
import argparse
import base64
import os
import sys
import zlib

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "racelab", "rl"))
from common import TRACK_ANGLES  # noqa: E402

SRC = os.path.join(ROOT, "gym_torcs", "torcs_jm_par.py")

RL_BLOCK = r'''
# =====================================================================
# RL SÜRÜCÜ (racelab/submission/build_submission.py ile üretildi)
# Model: __MODEL__
# Politika: SAC actor (32 → 256 → 256 → 2), deterministik (tanh(mu)).
# Agent sadece [direksiyon, pedal] üretir; vites/ABS/patinaj kontrolü kuraldır.
# =====================================================================
import base64 as _b64
import math as _math
import zlib as _zlib
from array import array as _array
from operator import mul as _mul

try:
    import numpy as _np
except ImportError:  # standart Python ile de çalışır (biraz daha yavaş, ~2-4 ms/adım)
    _np = None

_RL_SHAPES = __SHAPES__
_POS_OBS = _RL_SHAPES[0][1] > 32  # model giriş boyutu 44/46 ise konum bilen (v10 / D1)
_TRACK_LEN = 3608.2
_POS_HARMONICS = (1, 2, 4, 8, 16, 32, 64) if _RL_SHAPES[0][1] == 46 else (1, 2, 4, 8, 16, 32)
_RL_BLOB = (
__BLOB__
)


def _rl_load():
    raw = _zlib.decompress(_b64.b64decode("".join(_RL_BLOB)))
    flat = _array("f")
    flat.frombytes(raw)
    out, i = [], 0
    for rows, cols in _RL_SHAPES:
        n = rows * cols
        part = flat[i:i + n]
        i += n
        if _np is not None:
            out.append(_np.array(part, dtype=_np.float32).reshape(rows, cols) if cols > 1
                       else _np.array(part, dtype=_np.float32))
        else:
            out.append([list(part[r * cols:(r + 1) * cols]) for r in range(rows)] if cols > 1 else list(part))
    return out


_RL_W = _rl_load()  # [W0, b0, W1, b1, ..., Wn, bn] (gizli katman sayısı modele göre)


def _rl_policy(obs):
    Ws, bs = _RL_W[0::2], _RL_W[1::2]
    if _np is not None:
        h = _np.asarray(obs, dtype=_np.float32)
        for W, b in zip(Ws[:-1], bs[:-1]):
            h = _np.maximum(W @ h + b, 0)
        y = Ws[-1] @ h + bs[-1]
        return [float(_np.tanh(y[0])), float(_np.tanh(y[1]))]
    h = obs
    for W, b in zip(Ws[:-1], bs[:-1]):
        h = [max(sum(map(_mul, row, h)) + c, 0.0) for row, c in zip(W, b)]
    y = [sum(map(_mul, row, h)) + c for row, c in zip(Ws[-1][:2], bs[-1][:2])]
    return [_math.tanh(y[0]), _math.tanh(y[1])]


def _clipf(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _rl_obs(S, prev):
    o = [S['angle'] / _math.pi, _clipf(S['trackPos'], -3, 3), S['speedX'] / 200.0,
         S['speedY'] / 50.0, S['speedZ'] / 50.0]
    o += [t / 200.0 for t in S['track']]
    o += [w / 200.0 for w in S['wheelSpinVel']]
    o += [S['rpm'] / 20000.0, S['gear'] / 7.0]
    o += list(prev)
    if _POS_OBS:  # v10: pistteki konum harmonikleri
        ph = 2.0 * _math.pi * (S['distFromStart'] % _TRACK_LEN) / _TRACK_LEN
        o += [f(k * ph) for k in _POS_HARMONICS for f in (_math.sin, _math.cos)]
    return o


_GEAR_RATIOS = [3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
_UPSHIFT_RPM = __UPSHIFT__
_ASSIST_FRONT = __ASSIST__  # düzlük gaz yardımcısı (m), 0 = kapalı
_ASSIST_DEC, _ASSIST_MARGIN = __ASSIST_DIST__  # mesafe tabanlı yardımcı: (fren ivmesi g, pay m); dec 0 = kapalı
_ASSIST_REACT = __ASSIST_REACT__  # tepki payı (s): gerekli mesafeye hız×t eklenir
_MAX_GEAR = 6  # DriverAction.clip_to_limits 7'yi kabul etmez (Corkscrew'da 6. vites bile gerekmiyor)
_WHEEL_R = [0.3179, 0.3179, 0.3307, 0.3307]
_TCS_TH = __TCS_TH__  # patinaj kontrolü eşiği (m/s), modelin eğitildiği değer
_PEDAL_GAIN, _PEDAL_BIAS = __PEDAL__  # pedal eşlemesi (v7: 4/3, 1/3; öncesi 1, 0)


def _rl_gear(gear, rpm, vx):
    gear = int(gear)
    if gear < 1 or vx < 5:
        return 1
    if gear < _MAX_GEAR and rpm > _UPSHIFT_RPM:
        return gear + 1
    if gear > 1 and rpm < _UPSHIFT_RPM * _GEAR_RATIOS[gear - 1] / _GEAR_RATIOS[gear - 2] - 2000:
        return gear - 1
    return gear


def _rl_command(a, S):
    steer = _clipf(a[0], -1, 1)
    pedal = _clipf(_clipf(a[1], -1, 1) * _PEDAL_GAIN + _PEDAL_BIAS, -1, 1)
    accel, brake = max(pedal, 0.0), max(-pedal, 0.0)
    w = S['wheelSpinVel']
    spin = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) - (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1])
    if spin > _TCS_TH:  # patinaj kontrolü
        accel = max(0.0, accel - min(accel, (spin - _TCS_TH) * 0.1))
    if brake > 0 and S['speedX'] >= 10:  # ABS
        slip = S['speedX'] / 3.6 - sum(x * r for x, r in zip(w, _WHEEL_R)) / 4.0
        if slip > 2.0:
            brake = brake - min(brake, (slip - 2.0) * 0.1)
    if S['speedX'] < 10 and pedal > 0:
        accel = max(accel, 0.5)
    return steer, accel, brake, _rl_gear(S['gear'], S['rpm'], S['speedX'])


_RL_PREV = [0.0, 0.0]


def rl_drive(c):
    global _RL_PREV
    S, R = c.S.d, c.R.d
    a = _rl_policy(_rl_obs(S, _RL_PREV))
    if (_ASSIST_FRONT and S['track'][9] >= _ASSIST_FRONT and abs(S['angle']) < 0.05
            and abs(S['trackPos']) < 0.7 and a[1] > 0):
        a = [a[0], 1.0]  # düzlük gaz yardımcısı
    elif _ASSIST_DEC and abs(S['angle']) < 0.05 and abs(S['trackPos']) < 0.7 and a[1] > 0:
        _vms = max(S['speedX'], 0.0) / 3.6  # önü frenleme mesafesinden uzunsa tam gaz
        if S['track'][9] >= _vms * _vms / (2 * _ASSIST_DEC * 9.81) + _ASSIST_MARGIN + _vms * _ASSIST_REACT:
            a = [a[0], 1.0]
    R['steer'], R['accel'], R['brake'], R['gear'] = _rl_command(a, S)
    R['clutch'] = 0
    _RL_PREV = a


def drain_latest(c):
    """Sokette birikmiş sensör mesajı varsa en yenisine geç (gecikme birikmesini önler)."""
    if not c.so:
        return
    latest = None
    c.so.setblocking(False)
    try:
        while True:
            data, _ = c.so.recvfrom(data_size)
            latest = data.decode('utf-8')
            if '***' in latest:
                break
    except (BlockingIOError, OSError):
        pass
    if c.so:
        c.so.settimeout(1)
    if latest is None or '***identified***' in latest:
        return
    if '***shutdown***' in latest or '***restart***' in latest:
        print("Server has stopped the race on %d." % c.port)
        c.shutdown()
        return
    c.S.parse_server_str(latest)

'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", required=True, help="çıktı dosyası (mevcut teslim dosyasının üzerine yazmamak için zorunlu)")
    ap.add_argument("--tcs-th", type=float, default=2.0, help="modelin eğitildiği TCS eşiği (v1-v4: 2.0, v6: 5.0)")
    ap.add_argument("--pedal-gain", type=float, default=1.0)
    ap.add_argument("--pedal-bias", type=float, default=0.0)
    ap.add_argument("--assist-front", type=float, default=0.0, help="düzlük gaz yardımcısı (m), 0=kapalı")
    ap.add_argument("--upshift", type=float, default=18000)
    ap.add_argument("--assist-dec", type=float, default=0.0, help="mesafe tabanlı yardımcı fren ivmesi (g), 0=kapalı")
    ap.add_argument("--assist-margin", type=float, default=40.0)
    ap.add_argument("--assist-react", type=float, default=0.0, help="tepki payı (s)")
    args = ap.parse_args()

    ck = torch.load(args.model, map_location="cpu", weights_only=False)
    sd = ck["actor"]
    n_lin = len([k for k in sd if k.endswith(".weight")])
    keys = [f"net.{2 * i}.{t}" for i in range(n_lin) for t in ("weight", "bias")]
    arrays = [sd[k].numpy().astype(np.float32) for k in keys]
    shapes = [(a.shape[0], a.shape[1]) if a.ndim == 2 else (a.shape[0], 1) for a in arrays]
    blob = base64.b64encode(zlib.compress(np.concatenate([a.ravel() for a in arrays]).tobytes(), 9)).decode()
    lines = "\n".join(f'    "{blob[i:i + 100]}"' for i in range(0, len(blob), 100))
    model_name = os.path.relpath(os.path.abspath(args.model), ROOT).replace("\\", "/")
    rl = (RL_BLOCK.replace("__MODEL__", model_name).replace("__SHAPES__", repr(shapes))
          .replace("__BLOB__", lines).replace("__TCS_TH__", repr(float(args.tcs_th)))
          .replace("__PEDAL__", repr((float(args.pedal_gain), float(args.pedal_bias))))
          .replace("__UPSHIFT__", repr(float(args.upshift))).replace("__ASSIST__", repr(float(args.assist_front)))
          .replace("__ASSIST_DIST__", repr((float(args.assist_dec), float(args.assist_margin))))
          .replace("__ASSIST_REACT__", repr(float(args.assist_react))))

    with open(SRC, encoding="utf-8") as f:
        src = f.read()

    def sub(old, new):
        nonlocal src
        assert src.count(old) == 1, f"beklenen metin bulunamadı/çoklu: {old[:60]!r}"
        src = src.replace(old, new)

    angles = " ".join(str(a) for a in TRACK_ANGLES)
    sub('a= "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"',
        f'a= "{angles}"  # RL modelinin eğitildiği sensör açıları')
    sub("# ================= MAIN DRIVE FUNCTION =================",
        rl + "# ================= MAIN DRIVE FUNCTION =================")
    sub('''def drive_modular(c):
    S, R = c.S.d, c.R.d
    R['steer'] = calculate_steering(S)
    R['accel'] = calculate_throttle(S, R)
    R['brake'] = apply_brakes(S)
    R['accel'] = traction_control(S, R['accel'])
    R['gear'] = shift_gears(S)
    return''', '''def drive_modular(c):
    # RL politikası (yukarıdaki RL SÜRÜCÜ bölümü). Kural tabanlı eski mantık ve
    # --parameters değerleri bu sürümde sürüşte kullanılmaz (sözleşme için korunur).
    rl_drive(c)
    return''')
    sub('''        C.get_servers_input()
        drive_modular(C)
        C.respond_to_server()''', '''        C.get_servers_input()
        drain_latest(C)
        if not C.so:
            break
        drive_modular(C)
        C.respond_to_server()''')
    header = ('# Bu dosya racelab/submission/build_submission.py tarafından üretildi.\n'
              f'# Kaynak iskelet: gym_torcs/torcs_jm_par.py   Model: {model_name}\n')
    assert not os.path.exists(args.out), f"var olan dosyanın üzerine yazılmaz: {args.out}"
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(header + src)
    print("yazıldı:", args.out, f"({os.path.getsize(args.out) // 1024} KB)")


if __name__ == "__main__":
    main()
