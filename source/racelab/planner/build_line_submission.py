"""Yarış çizgisi sürücüsü için teslim dosyası: gym_torcs/torcs_jm_par.py iskeleti + gömülü plan (saf Python, numpy gerekmez).

    python racelab/planner/build_line_submission.py --params racelab/planner/runs/z1_best.json \
        --out racelab/submission/torcs_jm_par_line_v1.py

Orijinal iskelet sadece OKUNUR; var olan dosyanın üzerine yazılmaz. REQUIRED bloklar korunur.
Sürüş mantığı planner/line_driver.py ile birebir aynıdır (eşdeğerlik run_benchmark ile doğrulanır).
"""
import argparse
import base64
import json
import os
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
SRC = os.path.join(ROOT, "gym_torcs", "torcs_jm_par.py")

BLOCK = r'''
# =====================================================================
# YARIŞ ÇİZGİSİ SÜRÜCÜSÜ (racelab/planner/build_line_submission.py ile üretildi)
# Plan: pistin geometrisinden önceden hesaplanmış yarış çizgisi + hız profili (2 m aralık).
# Canlı: konum (distFromStart, trackPos, angle) → direksiyon; hız hedefi → gaz/fren; ABS, patinaj, vites kuraldır.
# Parametreler: __PNAME__
# =====================================================================
import base64 as _b64
import math as _math
import zlib as _zlib
from array import array as _array

_LP = __PARAMS__
_LN = __NPTS__
_LL = __LEN__
_DS = 2.0
_WB = 2.95
_SLOCK = _math.radians(21.0)
_WHEEL_R = [0.3179, 0.3179, 0.3307, 0.3307]
_GEAR_RATIOS = [3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
_PLAN_B64 = (
__BLOB__
)


def _plan_load():
    raw = _zlib.decompress(_b64.b64decode("".join(_PLAN_B64)))
    a = _array("d")
    a.frombytes(raw)
    cols = {}
    names = __COLS__
    for i, nm in enumerate(names):
        cols[nm] = list(a[i * _LN:(i + 1) * _LN])
    return cols


_PL = _plan_load()


def _lp_interp(key, s):
    x = (s % _LL) / _DS
    i = int(x) % _LN
    f = x - int(x)
    j = (i + 1) % _LN
    arr = _PL[key]
    return arr[i] * (1 - f) + arr[j] * f


def _lp_theta(key, s):
    x = (s % _LL) / _DS
    i = int(x) % _LN
    f = x - int(x)
    j = (i + 1) % _LN
    a, b = _PL[key][i], _PL[key][j]
    d = (b - a + _math.pi) % (2 * _math.pi) - _math.pi
    return a + d * f


def _lp_gear(gear, rpm, vx):
    up = _LP["upshift"]
    gear = int(gear)
    if gear < 1 or vx < 5:
        g = 1
    elif gear < 7 and rpm > up:
        g = gear + 1
    elif gear > 1 and rpm < up * _GEAR_RATIOS[gear - 1] / _GEAR_RATIOS[gear - 2] - _LP["dn_off"]:
        g = gear - 1
    else:
        g = gear
    if _LP["g2_v"] and vx > _LP["g2_v"]:
        g = max(g, 2)
    return min(g, int(_LP["max_gear"]))


_LP_STATE = {"prev_heading": None, "prev_steer": 0.0, "since_up": 99, "clutch": 0.0}


def _lp_wheel_rpm(S, g):
    w = S['wheelSpinVel']
    if g < 1:
        return 0.0
    return (w[2] + w[3]) / 2 * _GEAR_RATIOS[g - 1] * 4.5 * 60 / (2 * _math.pi)


def line_drive(c):
    S, R = c.S.d, c.R.d
    p = _LP
    s = S['distFromStart'] + p["offset"]
    v = max(S['speedX'] / 3.6, 0.0)
    half_w = _lp_interp("w", s) / 2
    cx, cy = _lp_interp("Cx", s), _lp_interp("Cy", s)
    nx, ny = _lp_interp("Nx", s), _lp_interp("Ny", s)
    carx, cary = cx + S['trackPos'] * half_w * nx, cy + S['trackPos'] * half_w * ny
    heading = _lp_theta("theta", s) - S['angle']
    yaw_rate = 0.0
    t_now = S.get('curLapTime', 0.0)
    dt = 0.02
    if p["dt_real"] and _LP_STATE.get("prev_t") is not None and 0.0 < t_now - _LP_STATE["prev_t"] < 0.1:
        dt = t_now - _LP_STATE["prev_t"]
    _LP_STATE["prev_t"] = t_now
    if _LP_STATE["prev_heading"] is not None:
        yaw_rate = ((heading - _LP_STATE["prev_heading"] + _math.pi) % (2 * _math.pi) - _math.pi) / dt
    _LP_STATE["prev_heading"] = heading
    if p["ctrl"] == "st":
        n_car = S['trackPos'] * half_w
        if p["pred"]:  # gecikme telafisi: durumu pred saniye ileri tahmin et
            s = s + v * p["pred"]
            n_car = n_car + v * _math.sin(-S['angle']) * p["pred"]
            heading = heading + yaw_rate * p["pred"]
        e = _lp_interp("n", s) - n_car
        th = _lp_theta("theta_p", s + p["st_ah"])
        beta = _math.atan2(p["st_csign"] * S.get('speedY', 0.0), max(S['speedX'], 5.0))  # kayma açısı
        beta = max(-p["st_slipmax"], min(p["st_slipmax"], beta))
        dh = (th - (heading + p["st_course"] * beta) + _math.pi) % (2 * _math.pi) - _math.pi
        kp = _lp_interp("k", s + p["st_aff"])
        fkh, fke, fff = _lp_interp("kh_s", s), _lp_interp("ke_s", s), _lp_interp("ff_s", s)
        delta = (p["st_kh"] * fkh * dh + _math.atan2(p["st_ke"] * fke * e, v + 5.0)
                 + p["st_kff"] * fff * _math.atan(_WB * kp) - p["st_kd"] * (yaw_rate - v * kp) * 0.1
                 + p["st_kus"] * v * v * kp + _lp_interp("ilc", s))
    else:
        ld = p["ld0"] + p["ld_v"] * v
        tx, ty = _lp_interp("Px", s + ld), _lp_interp("Py", s + ld)
        dx, dy = tx - carx, ty - cary
        lx = _math.cos(heading) * dx + _math.sin(heading) * dy
        ly = -_math.sin(heading) * dx + _math.cos(heading) * dy
        alpha = _math.atan2(ly, max(lx, 0.1))
        delta = _math.atan(2 * _WB * _math.sin(alpha) / max(_math.hypot(lx, ly), 1.0)) * p["k_steer"]
        k_ahead = _lp_interp("k", s + 0.5 * ld)
        delta += (p["k_ff"] - 1.0) * _math.atan(_WB * k_ahead)
    steer = max(-1.0, min(1.0, delta / _SLOCK))
    if p["ctrl"] == "st":
        steer = _LP_STATE["prev_steer"] + p["st_lp"] * (steer - _LP_STATE["prev_steer"])
        _LP_STATE["prev_steer"] = steer
    vt = min(_lp_interp("v", s), _lp_interp("v", s + v * p["look_t"]))
    err = v - vt
    if err > p["band"]:
        accel, brake = 0.0, min(1.0, p["k_brake"] * (err - p["band"]) + 0.3)
    else:
        accel, brake = (1.0 if err < -0.5 else max(0.0, min(1.0, 0.5 - err))), 0.0
    w = S['wheelSpinVel']
    spin = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) - (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1])
    if p["tcs_lam"] > 0:  # kayma oranı tabanlı patinaj kontrolü
        vr = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) / 2
        vf = (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1]) / 2
        lam = (vr - vf) / max(vf, 3.0)
        if lam > p["tcs_lam"]:
            accel = max(0.0, accel * (1.0 - p["tcs_k"] * (lam - p["tcs_lam"])))
    else:  # patinaj kontrolü (eşik); kalkışta daha yüksek eşik
        th = p["tcs"]
        if p["tcs_lv"] and S.get('distRaced', 1e9) < 500 and S['speedX'] < p["tcs_lv"]:
            th = p["tcs_l"]
        if spin > th:
            accel = max(0.0, accel - min(accel, (spin - th) * 0.1))
    if brake > 0 and S['speedX'] >= 10:  # ABS
        slip = S['speedX'] / 3.6 - sum(x * r for x, r in zip(w, _WHEEL_R)) / 4.0
        if slip > 2.0:
            brake = brake - min(brake, (slip - 2.0) * 0.1)
    if S['speedX'] < 10 and accel > 0:
        accel = max(accel, 0.5)
    rpm_eff = S['rpm']
    if _LP_STATE["clutch"] > 0 and int(S['gear']) >= 1:  # debriyaj kayarken tahrik tekerleği devri
        rpm_eff = _lp_wheel_rpm(S, int(S['gear']))
    gear = _lp_gear(S['gear'], rpm_eff, S['speedX'])
    clutch = 0.0
    if p["cl_max"] > 0 and accel > 0 and brake == 0 and gear >= 1:  # debriyaj kaydırma (pedal ≤ 2/3 tam tork)
        clutch = p["cl_max"] * min(1.0, max(0.0, (p["cl_top"] - _lp_wheel_rpm(S, gear)) / max(p["cl_band"], 1.0)))
        if _LP_STATE["since_up"] < p["cl_shift"]:
            clutch = max(clutch, p["cl_max"])
    _LP_STATE["since_up"] = 0 if gear > int(S['gear']) >= 1 else _LP_STATE["since_up"] + 1
    _LP_STATE["clutch"] = clutch
    if p["launch"]:  # geri sayımda boşta tam gaz, direksiyon düz; kalkışta erken 2. vites
        if S.get('curLapTime', 0.0) < 0:
            gear, accel, brake, steer = 0, 1.0, 0.0, 0.0
        elif S.get('distRaced', 1e9) < 300 and S['speedX'] > p["launch_up_v"]:
            gear = max(gear, 2)
    if p["st_lowv"] > 0 and v < p["st_lowv"]:  # düşük hızda direksiyon sınırı
        steer *= max(v / p["st_lowv"], p["st_lowv_min"])
    R['steer'], R['accel'], R['brake'] = steer, accel, brake
    R['gear'] = gear
    R['clutch'] = clutch


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

COLS = ["Cx", "Cy", "theta", "Nx", "Ny", "Px", "Py", "n", "k", "v", "w", "theta_p", "ilc", "kh_s", "ke_s", "ff_s"]


def load_params(path):
    d = json.load(open(path))
    z = d.get("_zones")
    p = {k: v for k, v in d.items() if not k.startswith("_")}
    if z:
        out = []
        for lo, hi, f, _ in z:
            out += [[lo + 3608.0, 3608.0, f], [0.0, hi, f]] if lo < 0 else [[lo, hi, f]]
        p["zones"] = out
        bz = d.get("_bz")
        if bz:
            outb = []
            for (lo, hi, _, _), f in zip(z, bz):
                outb += [[lo + 3608.0, 3608.0, f], [0.0, hi, f]] if lo < 0 else [[lo, hi, f]]
            p["bzones"] = outb
        kz = d.get("_kz")
        if kz:
            outk = []
            for (lo, hi, _, _), (fh, fe, ff) in zip(z, kz):
                outk += [[lo + 3608.0, 3608.0, fh, fe, ff], [0.0, hi, fh, fe, ff]] if lo < 0 else [[lo, hi, fh, fe, ff]]
            p["kzones"] = outk
        ns = d.get("_ns")
        if ns:
            p["nshift"] = [[zz[3], d.get("ns_width", 30.0), amp] for zz, amp in zip(z, ns) if amp]
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    assert not os.path.exists(a.out), f"var olan dosyanın üzerine yazılmaz: {a.out}"
    from line_driver import DEFAULTS
    from try_one import make_driver
    p = dict(DEFAULTS, **load_params(a.params))
    drv, pl = make_driver(p)
    arrs = dict(Cx=pl["C"][:, 0], Cy=pl["C"][:, 1], theta=pl["theta"], Nx=pl["N"][:, 0], Ny=pl["N"][:, 1],
                Px=pl["P"][:, 0], Py=pl["P"][:, 1], n=pl["n"], k=pl["k"], v=pl["v"], w=pl["w"], theta_p=pl["theta_p"], ilc=pl["ilc"], kh_s=pl["kh_s"], ke_s=pl["ke_s"], ff_s=pl["ff_s"])
    flat = np.concatenate([np.asarray(arrs[c], dtype=np.float64) for c in COLS])
    blob = base64.b64encode(zlib.compress(flat.tobytes(), 9)).decode()
    lines = "\n".join(f'    "{blob[i:i + 100]}"' for i in range(0, len(blob), 100))
    keep = {k: p[k] for k in ("offset", "ctrl", "st_kh", "st_ke", "st_kff", "st_kd", "st_ah", "st_aff", "st_kus", "st_lp", "dt_real", "ld0", "ld_v",
                              "k_steer", "k_ff", "look_t", "band", "k_brake", "tcs", "upshift", "max_gear", "tcs_lam", "tcs_k", "pred", "st_course", "st_csign", "st_slipmax", "g2_v", "dn_off", "launch", "launch_up_v", "st_lowv",
                              "st_lowv_min", "cl_max", "cl_top", "cl_band", "cl_shift", "tcs_lv", "tcs_l")}
    block = (BLOCK.replace("__PNAME__", os.path.relpath(os.path.abspath(a.params), ROOT).replace("\\", "/"))
             .replace("__PARAMS__", repr(keep)).replace("__NPTS__", str(len(pl["s"])))
             .replace("__LEN__", repr(float(pl["L"]))).replace("__BLOB__", lines).replace("__COLS__", repr(COLS)))
    src = open(SRC, encoding="utf-8").read()

    def sub(old, new):
        nonlocal src
        assert src.count(old) == 1, f"beklenen metin bulunamadı/çoklu: {old[:60]!r}"
        src = src.replace(old, new)

    sub("# ================= MAIN DRIVE FUNCTION =================",
        block + "# ================= MAIN DRIVE FUNCTION =================")
    sub('''def drive_modular(c):
    S, R = c.S.d, c.R.d
    R['steer'] = calculate_steering(S)
    R['accel'] = calculate_throttle(S, R)
    R['brake'] = apply_brakes(S)
    R['accel'] = traction_control(S, R['accel'])
    R['gear'] = shift_gears(S)
    return''', '''def drive_modular(c):
    # Yarış çizgisi sürücüsü (yukarıdaki bölüm). Kural tabanlı eski mantık ve
    # --parameters değerleri bu sürümde sürüşte kullanılmaz (sözleşme için korunur).
    line_drive(c)
    return''')
    sub('''        C.get_servers_input()
        drive_modular(C)
        C.respond_to_server()''', '''        C.get_servers_input()
        drain_latest(C)
        if not C.so:
            break
        drive_modular(C)
        C.respond_to_server()''')
    header = ('# Bu dosya racelab/planner/build_line_submission.py tarafından üretildi.\n'
              f'# Kaynak iskelet: gym_torcs/torcs_jm_par.py   Parametreler: {a.params}\n')
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(header + src)
    print("yazıldı:", a.out, f"({os.path.getsize(a.out) // 1024} KB)")


if __name__ == "__main__":
    main()
