"""RL agent'ı için ortak parçalar: SCR protokolü, gözlem, sürüş yardımcıları.

Eğitim ortamı (torcs_env.py) ve yarış agent'ı (agents/sac_agent.py) bu modülü
paylaşır; böylece eğitimde öğrenilen politika yarışta birebir aynı girdileri görür.

Agent sadece 2 şey öğrenir:  steer ∈ [-1, 1]  ve  pedal ∈ [-1, 1]
(pedal > 0 gaz, pedal < 0 fren). Vites, ABS ve patinaj kontrolü kuraldır.
"""
import math
import re

import numpy as np

# 19 mesafe sensörünün açıları (derece). Virajı erken görmek için öne yoğun.
TRACK_ANGLES = [-90, -60, -45, -30, -20, -15, -10, -6, -3, 0, 3, 6, 10, 15, 20, 30, 45, 60, 90]
INIT_MSG = "SCR(init " + " ".join(str(a) for a in TRACK_ANGLES) + ")"

OBS_DIM = 32
ACT_DIM = 2

_PAIR = re.compile(r"\((\w+) ([^)]*)\)")


def parse(msg):
    out = {}
    for key, val in _PAIR.findall(msg):
        nums = [float(x) for x in val.split()]
        out[key] = nums[0] if len(nums) == 1 else nums
    return out


def make_obs(s, prev_action):
    """Ham sensör sözlüğü → normalize edilmiş 32 boyutlu vektör."""
    track = np.asarray(s["track"], dtype=np.float32)
    obs = np.concatenate([
        [s["angle"] / math.pi,
         np.clip(s["trackPos"], -3, 3),
         s["speedX"] / 200.0,
         s["speedY"] / 50.0,
         s["speedZ"] / 50.0],
        track / 200.0,                                        # 19 (pist dışında -1 → -0.005)
        np.asarray(s["wheelSpinVel"], dtype=np.float32) / 200.0,  # 4
        [s["rpm"] / 20000.0,
         s["gear"] / 7.0],
        prev_action,                                          # 2
    ]).astype(np.float32)
    return obs


# ----------------------------------------------------------------- v10: konum özellikleri
# Pistteki konum (distFromStart) → sin/cos harmonikleri. Corkscrew'a özgü: agent fren/gaz noktalarını
# konuma göre öğrenebilsin. k=32 → en kısa dalga boyu ~113 m.
TRACK_LEN = 3608.2
POS_HARMONICS = (1, 2, 4, 8, 16, 32)
OBS_DIM_POS = OBS_DIM + 2 * len(POS_HARMONICS)  # 44


# D1: bir harmonik daha (k=64 → en kısa dalga ~56 m), gözlem 46 boyut
POS_HARMONICS2 = POS_HARMONICS + (64,)
OBS_DIM_POS2 = OBS_DIM + 2 * len(POS_HARMONICS2)  # 46


def pos_features(s, harmonics=POS_HARMONICS):
    ph = 2.0 * math.pi * (s["distFromStart"] % TRACK_LEN) / TRACK_LEN
    return [f(k * ph) for k in harmonics for f in (math.sin, math.cos)]


def make_obs_pos(s, prev_action):
    return np.concatenate([make_obs(s, prev_action), np.asarray(pos_features(s), dtype=np.float32)])


def make_obs_pos2(s, prev_action):
    return np.concatenate([make_obs(s, prev_action), np.asarray(pos_features(s, POS_HARMONICS2), dtype=np.float32)])


def obs_fn_for(obs_dim):
    """Model giriş boyutuna göre gözlem fonksiyonu (32: klasik, 44: v10, 46: D1)."""
    return {OBS_DIM_POS: make_obs_pos, OBS_DIM_POS2: make_obs_pos2}.get(obs_dim, make_obs)


def pos_mode_for(obs_dim):
    """TorcsEnvV2(pos_obs=...) değeri: 32 → False, 44 → True, 46 → 2."""
    return {OBS_DIM_POS: True, OBS_DIM_POS2: 2}.get(obs_dim, False)


# ----------------------------------------------------------------- sürüş yardımcıları
# car1-ow1: tork 18000 rpm'e kadar artıyor, sınırlayıcı 18700 → geç vites değiştir.
GEAR_RATIOS = [3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
UPSHIFT_RPM = 18000
MAX_GEAR = 7
WHEEL_RADIUS = [0.3179, 0.3179, 0.3307, 0.3307]  # m (12" ön / 13" arka jant + lastik)


def auto_gear(gear, rpm, speed_x, upshift=None):
    up = UPSHIFT_RPM if upshift is None else upshift  # v8: 18500
    gear = int(gear)
    if gear < 1 or speed_x < 5:
        return 1
    if gear < MAX_GEAR and rpm > up:
        return gear + 1
    if gear > 1:
        down_rpm = up * GEAR_RATIOS[gear - 1] / GEAR_RATIOS[gear - 2] - 2000
        if rpm < down_rpm:
            return gear - 1
    return gear


def abs_brake(brake, s):
    """Tekerlek kilitleniyorsa freni azalt."""
    if brake <= 0 or s["speedX"] < 10:
        return brake
    v = s["speedX"] / 3.6
    wheel_v = np.mean([w * r for w, r in zip(s["wheelSpinVel"], WHEEL_RADIUS)])
    slip = v - wheel_v
    if slip > 2.0:
        brake = brake - min(brake, (slip - 2.0) * 0.1)
    return brake


def traction_control(accel, s, threshold=2.0):
    """Arka tekerler öndekilerden çok hızlı dönüyorsa gazı kes.
    threshold=2.0 v1-v4 modellerinin eğitildiği değer; v6'da 5.0 (2.0 düzlükte gazı gereksiz kesiyordu)."""
    w = s["wheelSpinVel"]
    spin = (w[2] * WHEEL_RADIUS[2] + w[3] * WHEEL_RADIUS[3]) - (w[0] * WHEEL_RADIUS[0] + w[1] * WHEEL_RADIUS[1])
    if spin > threshold:
        accel = max(0.0, accel - min(accel, (spin - threshold) * 0.1))
    return accel


def straight_assist(action, s, front=0.0, max_angle=0.05, max_pos=0.7):
    """v8 düzlük gaz yardımcısı: önü >= front m açık, araç düz ve pistin içindeyse,
    politika gaz istiyorsa gaz = 1. front=0 -> kapalı. Döner: (yeni aksiyon, uygulandı mı)."""
    if (front and s["track"][9] >= front and abs(s["angle"]) < max_angle
            and abs(s["trackPos"]) < max_pos and action[1] > 0):
        a = np.array(action, dtype=np.float32).copy()
        a[1] = 1.0
        return a, True
    return action, False


def to_command(action, s, tcs_threshold=2.0, pedal_gain=1.0, pedal_bias=0.0, upshift=None):
    """Politika çıktısı [steer, pedal] → SCR komut sözlüğü.
    v7: pedal' = clip(pedal*gain + bias); gain=4/3, bias=1/3 → politika 0.5 = tam gaz, -0.25 = sıfır."""
    steer = float(np.clip(action[0], -1, 1))
    pedal = float(np.clip(float(np.clip(action[1], -1, 1)) * pedal_gain + pedal_bias, -1, 1))
    accel = max(pedal, 0.0)
    brake = max(-pedal, 0.0)
    accel = traction_control(accel, s, tcs_threshold)
    brake = abs_brake(brake, s)
    if s["speedX"] < 10 and pedal > 0:  # kalkışta tam gaz desteği
        accel = max(accel, 0.5)
    gear = auto_gear(s["gear"], s["rpm"], s["speedX"], upshift)
    return {"accel": accel, "brake": brake, "gear": gear, "steer": steer, "clutch": 0.0, "meta": 0}


def command_msg(cmd):
    return ("(accel %.4f)(brake %.4f)(gear %d)(steer %.4f)(clutch %.3f)(focus 0)(meta %d)"
            % (cmd["accel"], cmd["brake"], cmd["gear"], cmd["steer"], cmd["clutch"], cmd["meta"]))
