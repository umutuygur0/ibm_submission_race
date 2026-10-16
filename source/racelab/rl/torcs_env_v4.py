"""TorcsEnv v4: v2 + zikzak'a karşı YUMUŞAK cezalar (sert direksiyon sınırı yok).

v3'teki sert hız sınırı, hızlı direksiyon gereken virajları (~450 m fren virajı,
~2470 m corkscrew S virajı) imkânsız kıldı. v4'te direksiyon serbesttir; zikzak pahalıdır:
- değişim cezası:   c_change × |Δsteer| × (v/100)
- yön değiştirme:   c_flip × (v/100)   (işaret değişiyor ve iki değer de |.|>0.05 ise)
- düzde direksiyon: c_mag × steer² × düzlük × (v/100),  düzlük = ön sensör (0°) / 200
  → virajda (önü kısa) neredeyse ceza yok, uzun düzlükte tam ceza.
- (isteğe bağlı) c_speed × ilerleme(m): hızlı gitmeye ek teşvik (v2 tabanı 0.5 × ilerleme).
Agent: agents/sac_agent_v2.py (aksiyon dönüşümü v2 ile aynı).
"""
import numpy as np

from torcs_env_v2 import TorcsEnvV2


class TorcsEnvV4(TorcsEnvV2):
    def __init__(self, *a, c_change=0.3, c_flip=0.2, c_mag=0.3, c_speed=0.0, **kw):
        super().__init__(*a, **kw)
        self.c_change, self.c_flip, self.c_mag, self.c_speed = c_change, c_flip, c_mag, c_speed

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        prev = float(self.prev_action[0])
        prev_dist = self.prev_dist
        front = float(self.s["track"][9]) if self.s["track"][9] > 0 else 0.0
        obs, reward, done, info = super().step(action)
        v = max(self.s["speedX"], 0.0) / 100.0
        st = float(action[0])
        pen = self.c_change * abs(st - prev) * v
        if st * prev < 0 and abs(st) > 0.05 and abs(prev) > 0.05:
            pen += self.c_flip * v
        pen += self.c_mag * st * st * min(front / 200.0, 1.0) * v
        bonus = self.c_speed * max(self.s["distRaced"] - prev_dist, 0.0)  # ek ilerleme ödülü
        self.ep_return += bonus - pen
        return obs, float(reward - pen + bonus), done, info
