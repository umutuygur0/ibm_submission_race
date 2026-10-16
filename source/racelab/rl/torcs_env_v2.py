"""TorcsEnv v2: çarpmama odaklı ödül + sağlamlık (robustness) eğitimi.

v1'den farklar
- Güvenlik payı: |trackPos| > SAFE_POS iken hızla orantılı kademeli ceza.
- Pist dışı / hasar cezası hıza göre büyür (yüksek hızda kenara gitmek daha pahalı).
- Pertürbasyon (sadece eğitimde): her adımda `drop_prob` olasılıkla agent'ın yeni
  komutu yerine bir önceki komut uygulanır (GUI'de TORCS'un 10 ms'de cevap alamayıp
  eski komutu kullanması gibi) + gözleme hafif Gauss gürültüsü.
- `race_mode=True`: erken bitirme yok, yarış TORCS kapatana kadar sürer (değerlendirme için).

Gözlem/aksiyon/sürüş yardımcıları v1 ile aynıdır (common.py) → agents/sac_agent.py
v2 ile eğitilen modelleri de değişiklik olmadan çalıştırır.
"""
import numpy as np

from common import ACT_DIM, command_msg, make_obs, make_obs_pos, make_obs_pos2, straight_assist, to_command
from torcs_env import TorcsEnv

SAFE_POS = 0.75


class TorcsEnvV2(TorcsEnv):
    def __init__(self, idx=1, track="corkscrew", max_steps=12000, udp_timeout_us=1000000,
                 drop_prob=0.0, obs_noise=0.0, race_mode=False, seed=0, tcs_threshold=2.0, safe_pos=SAFE_POS,
                 pedal_gain=1.0, pedal_bias=0.0, assist_front=0.0, upshift=None, pos_obs=False):
        super().__init__(idx=idx, track=track, max_steps=max_steps, udp_timeout_us=udp_timeout_us)
        self.drop_prob = drop_prob
        self.obs_noise = obs_noise
        self.race_mode = race_mode
        self.tcs_threshold = tcs_threshold  # v6: 5.0
        self.safe_pos = safe_pos            # v6b: 0.9 (yarış çizgisi için pist genişliği)
        self.pedal_gain, self.pedal_bias = pedal_gain, pedal_bias  # v7: 4/3, 1/3
        self.assist_front, self.upshift = assist_front, upshift      # v8: 170-180, 18500
        self.pos_obs = pos_obs                                        # v10: gözleme pist konumu
        self.rng = np.random.default_rng(seed)

    def _obs(self, s, action):
        if self.pos_obs == 2:  # D1 (46 boyut)
            o = make_obs_pos2(s, action)
        else:
            o = make_obs_pos(s, action) if self.pos_obs else make_obs(s, action)
        if self.obs_noise > 0:
            o = o + self.rng.normal(0, self.obs_noise, size=o.shape).astype(np.float32)
        return o

    def reset(self):
        super().reset()
        self.applied = np.zeros(ACT_DIM, dtype=np.float32)
        self.offtrack_steps = 0
        self.hits = 0
        return self._obs(self.s, self.prev_action)

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        if self.assist_front:  # v8: gözlemdeki önceki aksiyon = yardımcı sonrası aksiyon
            action, _ = straight_assist(action, self.s, self.assist_front)
        # pertürbasyon: bu adımda yeni komut "ulaşmasın", eskisi uygulansın
        applied = self.applied if (self.drop_prob > 0 and self.rng.random() < self.drop_prob) else action
        cmd = to_command(applied, self.s, self.tcs_threshold, self.pedal_gain, self.pedal_bias, self.upshift)
        self.sock.sendto(command_msg(cmd).encode(), ("127.0.0.1", self.port))
        s = self._recv()
        self.steps += 1
        info = {"lap_time": None, "crash": False, "offtrack": False, "stuck": False, "truncated": False}

        if s is None:  # TORCS yarışı kapattı (tur limiti / race_mode'da yarış sonu)
            self._kill()
            info["truncated"] = True
            info["distance"] = self.s["distRaced"] - self.start_dist
            info["damage"] = self.s["damage"]
            info["hits"] = self.hits
            info["offtrack_steps"] = self.offtrack_steps
            return self._obs(self.s, action), 0.0, True, info

        v = max(s["speedX"], 0.0)
        pos = abs(s["trackPos"])
        progress = s["distRaced"] - self.prev_dist
        d_damage = s["damage"] - self.prev_damage

        reward = progress * 0.5
        reward -= 0.05 * abs(action[0] - self.prev_action[0])
        reward -= 0.01
        if pos > self.safe_pos:  # kenara yaklaştıkça artan, hızla ölçeklenen ceza
            reward -= (pos - self.safe_pos) * (0.5 + v / 100.0)

        done = False
        if d_damage > 0:
            self.hits += 1
            reward -= 10.0 + v / 10.0
            info["crash"] = True
            done = not self.race_mode
        if pos > 1.0:
            self.offtrack_steps += 1
            if not self.race_mode:
                reward -= 10.0 + v / 10.0
                info["offtrack"] = True
                done = True
        if s["lastLapTime"] > 0 and s["lastLapTime"] != self.prev_last_lap:
            info["lap_time"] = s["lastLapTime"]
            reward += 20.0
            done = True
        self.slow_steps = self.slow_steps + 1 if s["speedX"] < 5 else 0
        if not self.race_mode and self.steps > 100 and self.slow_steps > self.stuck_steps:
            reward -= 5.0
            done = info["stuck"] = True
        if self.steps >= self.max_steps:
            done = info["truncated"] = True

        info["distance"] = s["distRaced"] - self.start_dist
        info["damage"] = s["damage"]
        info["hits"] = self.hits
        info["offtrack_steps"] = self.offtrack_steps
        self.prev_dist, self.prev_damage, self.prev_last_lap = s["distRaced"], s["damage"], s["lastLapTime"]
        self.prev_action = action
        self.applied = applied
        self.s = s
        self.ep_return += reward
        if done:
            self._kill()
        return self._obs(s, action), float(reward), done, info
