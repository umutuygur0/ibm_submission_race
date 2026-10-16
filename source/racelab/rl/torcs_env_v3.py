"""TorcsEnv v3: v2 + direksiyon zikzak'ını önleme.

v2 modelleri düz yolda saniyede ~9-10 kez sağ-sol yapıyordu (ort |steer| ≈ 0.5, gaz ≈ %55-60).
v3:
- Direksiyon hız sınırı (smooth_steer): bir adımda direksiyon en fazla
  max_delta(v) kadar değişir; 200+ km/h'te `rate_hi`, duran araçta `rate_lo`.
  Agent (agents/sac_agent_v3.py) aynı fonksiyonu kullanır → eğitim = yarış.
- Ek ceza: |Δsteer| ve yüksek hızda steer² (düzde direksiyonu boşuna kullanma).

Gözlemdeki "önceki aksiyon", hız sınırı uygulanmış direksiyondur.
"""
import numpy as np

from torcs_env_v2 import TorcsEnvV2

RATE_HI = 0.04   # 200+ km/h'te adım başına en fazla direksiyon değişimi (≈ 2/s)
RATE_LO = 0.20   # 0 km/h'te


def smooth_steer(new, prev, speed_kmh, rate_hi=RATE_HI, rate_lo=RATE_LO):
    k = min(max(speed_kmh, 0.0) / 200.0, 1.0)
    max_delta = rate_lo + (rate_hi - rate_lo) * k
    return float(prev + np.clip(new - prev, -max_delta, max_delta))


class TorcsEnvV3(TorcsEnvV2):
    def __init__(self, *a, steer_change_pen=0.3, steer_mag_pen=0.05, rate_hi=RATE_HI, rate_lo=RATE_LO, **kw):
        super().__init__(*a, **kw)
        self.steer_change_pen = steer_change_pen
        self.steer_mag_pen = steer_mag_pen
        self.rate_hi, self.rate_lo = rate_hi, rate_lo

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1, 1).copy()
        prev_steer = float(self.prev_action[0])
        action[0] = smooth_steer(action[0], prev_steer, self.s["speedX"], self.rate_hi, self.rate_lo)
        obs, reward, done, info = super().step(action)
        v = max(self.s["speedX"], 0.0)
        reward -= self.steer_change_pen * abs(action[0] - prev_steer)
        reward -= self.steer_mag_pen * action[0] ** 2 * (v / 100.0)
        self.ep_return += -self.steer_change_pen * abs(action[0] - prev_steer) \
            - self.steer_mag_pen * action[0] ** 2 * (v / 100.0)
        return obs, float(reward), done, info
