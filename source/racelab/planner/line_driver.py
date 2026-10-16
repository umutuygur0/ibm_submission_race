"""Yarış çizgisi takipçisi (kural tabanlı, eğitimsiz): önceden hesaplanmış çizgi + hız profili, canlı düzeltme.

Plan (build_plan): 2 m aralıklı orta çizgi C(s), yönü θ(s), sola normal N(s), çizgi noktası P(s), hedef hız v(s).
Sürücü her adımda:
  - konum: s = distFromStart, araç = C(s) + trackPos·(w/2)·N(s), yön = θ(s) − angle
  - direksiyon: saf takip (pure pursuit) → P(s + Ld) hedefi, + eğrilik ön beslemesi
  - hız: v(s + bakış) hedefi; altında tam gaz, üstünde orantılı fren (ABS), patinaj kontrolü
  - vites: mevcut otomatik vites (common.auto_gear)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "rl"))
import common  # noqa: E402
from plan_line import curvature, load_track, min_curvature_line, normals, speed_profile  # noqa: E402

WHEELBASE = 2.95
STEER_LOCK = math.radians(21.0)
DS = 2.0

# telemetriden (şampiyon turu) ölçülen tam gaz ivmesi (m/s²) — hız (m/s) noktalarında
ACC_V = np.array([0, 10, 30, 45, 55, 65, 75, 82, 90])
ACC_A = np.array([12.0, 11.8, 11.0, 11.5, 8.8, 7.2, 4.2, 1.0, 0.0])

DEFAULTS = dict(
    margin=1.6,      # çizginin kenardan payı (m)
    mu=2.0, aero=0.0010,          # yanal tutunma
    mu_b=2.4, aero_b=0.0010,      # fren
    acc_scale=1.0, v_max=95.0,
    ld0=6.0, ld_v=0.25,           # bakış mesafesi = ld0 + ld_v·v (m)
    k_ff=1.0,                     # eğrilik ön besleme kazancı
    k_steer=1.0,                  # saf takip kazancı
    look_t=0.15,                  # hız hedefi için zaman bakışı (s)
    k_brake=0.08,                 # (v − hedef) [m/s] başına fren
    band=0.3,                     # hedefin bu kadar üstüne çıkınca frenle (m/s)
    tcs=5.0, upshift=18650.0,
    offset=0.0,                   # s hizalama düzeltmesi (m)
    zones=None,                   # [(s_başı, s_sonu, hız çarpanı), ...]
    max_gear=6,                   # teslim dosyası (DriverAction) 6'yı aşmaz → değerlendirmede 6 kullan
    ctrl="pp",                    # "pp": saf takip, "st": Stanley tipi (yön + yanal hata + ön besleme + sönüm)
    st_kh=1.0, st_ke=2.0, st_kff=1.0, st_kd=0.0, st_ah=4.0, st_aff=6.0,
    st_kus=0.0,                   # lastik kayması ön beslemesi: direksiyon += st_kus · v² · k (rad)
    st_lp=1.0,                    # direksiyon alçak geçiren süzgeç (1 = süzgeçsiz)
    dt_real=False,
    pred=0.0,                     # gecikme telafisi: durum tahmini (s); 0 = kapalı
    g2_v=0.0, dn_off=2000.0,
    launch=0, launch_up_v=30.0, st_lowv=0.0, st_lowv_min=0.15,
    cl_max=0.0, cl_top=18000.0, cl_band=1500.0, cl_shift=0,
    tcs_lv=0.0, tcs_l=30.0,       # kalkış: distRaced < 500 m ve hız < tcs_lv km/h iken patinaj eşiği tcs_l (0 = kapalı)  # debriyaj kaydırma (TORCS: pedal ≤ 2/3 tam tork); 0 = kapalı  # kalkış: boşta devir, erken 2. vites, düşük hız direksiyon sınırı      # vites: g2_v km/h üstünde en az 2. vites (0 = kapalı); küçültme payı (devir)
    st_course=0.0, st_csign=1.0, st_slipmax=1.0,  # st_slipmax: kayma açısı katkısının sınırı (rad)  # yön terimi: 0 = gövde yönü, 1 = gidiş (hız vektörü) yönü; csign = speedY işareti
    tcs_lam=0.0, tcs_k=5.0,       # >0: kayma oranı tabanlı patinaj kontrolü (hedef oran, kazanç); 0 = eski eşik yöntemi                # True: yaw hızı için gerçek geçen süre (curLapTime farkı)
)


def build_plan(p, track=None, line=None):
    """Parametrelere göre plan dizileri. track/line verilirse yeniden hesaplanmaz (hız için)."""
    if track is None:
        track = load_track(DS)
    s, c, w, L = track
    xy = c[:, :2]
    if line is None:
        line = min_curvature_line(xy, w, margin=p["margin"])
    n, P = line
    if p.get("nshift"):  # viraj bazlı apeks kaydırma: [(merkez s, genişlik m, genlik m), ...] (sol +)
        lim = w / 2 - p["margin"]
        n = n.copy()
        for cs, wd, amp in p["nshift"]:
            d = (s - cs + L / 2) % L - L / 2
            n += amp * np.exp(-(d / wd) ** 2)
        n = np.clip(n, -lim, lim)
        P = xy + n[:, None] * normals(xy)
    k = curvature(P)
    k = np.convolve(np.concatenate([k[-3:], k, k[:3]]), np.ones(7) / 7, "valid")  # gürültü süzgeci
    bscale = None
    if p.get("bzones"):  # bölge bazlı fren çarpanı: [(s_başı, s_sonu, çarpan), ...]
        bscale = np.ones(len(s))
        for a, b, f in p["bzones"]:
            bscale[(s >= a) & (s < b)] *= f
    scale = None
    if p.get("zones"):
        scale = np.ones(len(s))
        for a, b, f in p["zones"]:
            scale[(s >= a) & (s < b)] *= f

    def acc_fn(v):
        return p["acc_scale"] * float(np.interp(v, ACC_V, ACC_A))
    z = c[:, 2]
    zs = np.convolve(np.concatenate([z[-10:], z, z[:10]]), np.ones(21) / 21, "valid")
    kv = (np.roll(zs, -1) - 2 * zs + np.roll(zs, 1)) / DS ** 2  # dikey eğrilik (1/m)
    kv = np.convolve(np.concatenate([kv[-5:], kv, kv[:5]]), np.ones(11) / 11, "valid")
    v = speed_profile(k, DS, mu=p["mu"], aero=p["aero"], mu_brake=p["mu_b"], aero_brake=p["aero_b"],
                      acc_fn=acc_fn, v_max=p["v_max"], scale=scale, kv=kv, vert=p.get("vert", 0.0), bscale=bscale)
    t = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    theta = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    kh_s, ke_s, ff_s = np.ones(len(s)), np.ones(len(s)), np.ones(len(s))
    if p.get("kzones"):  # viraj bazlı kontrolcü kazanç çarpanları: [(s_başı, s_sonu, f_kh, f_ke, f_ff), ...]
        for a_, b_, fh, fe, ff in p["kzones"]:
            m_ = (s >= a_) & (s < b_)
            kh_s[m_] *= fh
            ke_s[m_] *= fe
            ff_s[m_] *= ff
    ilc = np.zeros(len(s))
    if p.get("ilc_file"):  # yinelemeli öğrenen kontrol: konuma bağlı direksiyon düzeltmesi (rad)
        ilc = np.load(os.path.join(HERE, p["ilc_file"]) if not os.path.isabs(p["ilc_file"]) else p["ilc_file"])
    tp = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    theta_p = np.unwrap(np.arctan2(tp[:, 1], tp[:, 0]))
    return dict(s=s, C=xy, theta=theta, N=normals(xy), P=P, n=n, k=k, v=v, w=w, L=L, theta_p=theta_p, ilc=ilc,
                kh_s=kh_s, ke_s=ke_s, ff_s=ff_s)


class LineDriver:
    def __init__(self, plan, params):
        self.pl = plan
        self.p = dict(DEFAULTS, **params)
        self.m = len(plan["s"])
        self.L = plan["L"]
        self.prev_heading = None
        self.prev_steer = 0.0
        self.since_up = 99
        self.clutch = 0.0

    def _idx(self, s):
        return (s % self.L) / DS

    def _interp(self, arr, s):
        x = self._idx(s)
        i = int(x) % self.m
        f = x - int(x)
        j = (i + 1) % self.m
        return arr[i] * (1 - f) + arr[j] * f

    def _theta(self, s, key="theta"):
        x = self._idx(s)
        i = int(x) % self.m
        f = x - int(x)
        j = (i + 1) % self.m
        a, b = self.pl[key][i], self.pl[key][j]
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        return a + d * f

    def act(self, S):
        cmd = self._act(S)
        p = self.p
        if p.get("launch", 0):  # geri sayımda boşta tam gaz, direksiyon düz (motor boğulmasın)
            if S.get("curLapTime", 0.0) < 0:
                cmd.update(gear=0, accel=1.0, brake=0.0, steer=0.0)
            elif S.get("distRaced", 1e9) < 300 and S["speedX"] > p.get("launch_up_v", 30.0):
                cmd["gear"] = max(cmd["gear"], 2)  # kalkışta erken 2. vites, 1'e geri düşmesin
        lv = p.get("st_lowv", 0.0)
        if lv > 0:  # düşük hızda direksiyonu sınırla (kalkışta tam kilit olmasın)
            v = max(S["speedX"] / 3.6, 0.0)
            if v < lv:
                cmd["steer"] *= max(v / lv, p.get("st_lowv_min", 0.15))
        return cmd

    def _act(self, S):
        p = self.p
        s = S["distFromStart"] + p["offset"]
        v = max(S["speedX"] / 3.6, 0.0)
        half_w = self._interp(self.pl["w"], s) / 2
        C = self._interp(self.pl["C"], s)
        N = self._interp(self.pl["N"], s)
        car = C + S["trackPos"] * half_w * N
        heading = self._theta(s) - S["angle"]
        yaw_rate = 0.0
        t_now = S.get("curLapTime", 0.0)
        dt = 0.02
        if p.get("dt_real") and getattr(self, "prev_t", None) is not None and 0.0 < t_now - self.prev_t < 0.1:
            dt = t_now - self.prev_t  # kaçan adımda gerçek geçen süre (yaw hızı sıçramasın)
        self.prev_t = t_now
        if self.prev_heading is not None:
            yaw_rate = ((heading - self.prev_heading + math.pi) % (2 * math.pi) - math.pi) / dt
        self.prev_heading = heading
        if p["ctrl"] == "st":
            n_car = S["trackPos"] * half_w
            pr = p.get("pred", 0.0)
            if pr:  # gecikme telafisi: durumu pr saniye ileri tahmin et
                s = s + v * pr
                n_car = n_car + v * math.sin(-S["angle"]) * pr
                heading = heading + yaw_rate * pr
            e = self._interp(self.pl["n"], s) - n_car
            th = self._theta(s + p["st_ah"], "theta_p")
            beta = math.atan2(p.get("st_csign", 1.0) * S.get("speedY", 0.0), max(S["speedX"], 5.0))  # kayma açısı
            smax = p.get("st_slipmax", 1.0)
            course = heading + p.get("st_course", 0.0) * max(-smax, min(smax, beta))
            dh = (th - course + math.pi) % (2 * math.pi) - math.pi
            kp = self._interp(self.pl["k"], s + p["st_aff"])
            fkh, fke, fff = (self._interp(self.pl["kh_s"], s), self._interp(self.pl["ke_s"], s),
                             self._interp(self.pl["ff_s"], s))
            delta = (p["st_kh"] * fkh * dh + math.atan2(p["st_ke"] * fke * e, v + 5.0)
                     + p["st_kff"] * fff * math.atan(WHEELBASE * kp) - p["st_kd"] * (yaw_rate - v * kp) * 0.1
                     + p.get("st_kus", 0.0) * v * v * kp + self._interp(self.pl["ilc"], s))
            st = max(-1.0, min(1.0, delta / STEER_LOCK))
            lp = p.get("st_lp", 1.0)
            st = self.prev_steer + lp * (st - self.prev_steer)
            self.prev_steer = st
            return self._speed(S, s, v, st)
        # saf takip
        ld = p["ld0"] + p["ld_v"] * v
        T = self._interp(self.pl["P"], s + ld)
        dx, dy = T - car
        lx = math.cos(heading) * dx + math.sin(heading) * dy
        ly = -math.sin(heading) * dx + math.cos(heading) * dy
        alpha = math.atan2(ly, max(lx, 0.1))
        delta = math.atan(2 * WHEELBASE * math.sin(alpha) / max(math.hypot(lx, ly), 1.0)) * p["k_steer"]
        k_ahead = self._interp(self.pl["k"], s + 0.5 * ld)
        delta += (p["k_ff"] - 1.0) * math.atan(WHEELBASE * k_ahead)  # k_ff=1 → saf takip zaten içerir
        steer = max(-1.0, min(1.0, delta / STEER_LOCK))
        return self._speed(S, s, v, steer)

    def _wheel_rpm(self, S, g):
        w = S["wheelSpinVel"]
        if g < 1:
            return 0.0
        return (w[2] + w[3]) / 2 * common.GEAR_RATIOS[g - 1] * 4.5 * 60 / (2 * math.pi)

    def _gear(self, S):
        p = self.p
        up, g, rpm, vx = p["upshift"], int(S["gear"]), S["rpm"], S["speedX"]
        if self.clutch > 0 and g >= 1:  # debriyaj kayarken motor devri yanıltıcı → tahrik tekerleği devri
            rpm = self._wheel_rpm(S, g)
        R = common.GEAR_RATIOS
        if g < 1 or vx < 5:
            g = 1
        elif g < 7 and rpm > up:
            g = g + 1
        elif g > 1 and rpm < up * R[g - 1] / R[g - 2] - p.get("dn_off", 2000.0):
            g = g - 1
        if p.get("g2_v", 0.0) and vx > p["g2_v"]:
            g = max(g, 2)
        return min(g, int(p["max_gear"]))

    def _speed(self, S, s, v, steer):
        p = self.p
        # hız
        vt = min(self._interp(self.pl["v"], s), self._interp(self.pl["v"], s + v * p["look_t"]))
        err = v - vt
        if err > p["band"]:
            accel, brake = 0.0, min(1.0, p["k_brake"] * (err - p["band"]) + 0.3)
        else:
            accel, brake = (1.0 if err < -0.5 else max(0.0, min(1.0, 0.5 - err))), 0.0
        if p.get("tcs_lam", 0.0) > 0:
            w = S["wheelSpinVel"]
            R = common.WHEEL_RADIUS
            vr = (w[2] * R[2] + w[3] * R[3]) / 2
            vf = (w[0] * R[0] + w[1] * R[1]) / 2
            lam = (vr - vf) / max(vf, 3.0)
            if lam > p["tcs_lam"]:
                accel = max(0.0, accel * (1.0 - p["tcs_k"] * (lam - p["tcs_lam"])))
        else:
            th = p["tcs"]
            if p.get("tcs_lv", 0.0) and S.get("distRaced", 1e9) < 500 and S["speedX"] < p["tcs_lv"]:
                th = p.get("tcs_l", 30.0)
            accel = common.traction_control(accel, S, th)
        brake = common.abs_brake(brake, S)
        if S["speedX"] < 10 and accel > 0:
            accel = max(accel, 0.5)
        gear = self._gear(S)
        clutch = 0.0
        if p.get("cl_max", 0.0) > 0 and accel > 0 and brake == 0 and gear >= 1:
            wr = self._wheel_rpm(S, gear)
            clutch = p["cl_max"] * min(1.0, max(0.0, (p["cl_top"] - wr) / max(p["cl_band"], 1.0)))
            if self.since_up < p.get("cl_shift", 0):
                clutch = max(clutch, p["cl_max"])
        self.since_up = 0 if gear > int(S["gear"]) >= 1 else self.since_up + 1
        self.clutch = clutch
        return {"accel": accel, "brake": brake, "gear": gear, "steer": steer, "clutch": clutch, "meta": 0}
