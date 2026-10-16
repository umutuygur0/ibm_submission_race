"""Artık (residual) RL ortamı: temel sürücü = planner/line_driver.LineDriver (ör. gr3L); RL yalnızca küçük düzeltme ekler.

Aksiyon a ∈ [-1, 1]²: direksiyon += a0·d_steer, pedal (gaz − fren) += a1·d_pedal; sonra patinaj kontrolü + ABS yeniden.
Vites ve debriyaj temel sürücüden. Gözlem (RES_OBS boyut) teslim dosyasında da hesaplanabilir büyüklüklerden oluşur.
Ödül: ilerleme (m) × 0,1 − kenar aşımı − hasar − düzeltme büyüklüğü cezası; tur bonusu.
Sağlamlık: bölümlerin p_delay oranında kalıcı 1 adım gecikme, drop_prob ile komut tekrarı (adım kaçırma taklidi).
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLANNER = os.path.join(os.path.dirname(HERE), "planner")
sys.path.insert(0, HERE)
sys.path.insert(0, PLANNER)
import common  # noqa: E402
from common import command_msg  # noqa: E402
from torcs_env import TorcsEnv  # noqa: E402

RES_OBS = 28
HARM = (1, 2, 4, 8, 16)


def res_obs(S, drv, base, prev_a):
    """drv: LineDriver (plan dizileri), base: temel komut sözlüğü."""
    pl, p = drv.pl, drv.p
    s = S["distFromStart"] + p["offset"]
    v = max(S["speedX"] / 3.6, 0.0)
    half_w = drv._interp(pl["w"], s) / 2
    n_car = S["trackPos"] * half_w
    e = drv._interp(pl["n"], s) - n_car
    heading = drv._theta(s) - S["angle"]
    dh = (drv._theta(s + 4.0, "theta_p") - heading + math.pi) % (2 * math.pi) - math.pi
    vt = drv._interp(pl["v"], s)
    ks = [drv._interp(pl["k"], s + d) * 40.0 for d in (5.0, 20.0, 45.0, 80.0)]
    w = S["wheelSpinVel"]
    R = common.WHEEL_RADIUS
    spin = ((w[2] * R[2] + w[3] * R[3]) - (w[0] * R[0] + w[1] * R[1])) / 2
    ph = 2 * math.pi * (S["distFromStart"] % drv.L) / drv.L
    harm = [f(k * ph) for k in HARM for f in (math.sin, math.cos)]
    o = [v / 80.0, S["trackPos"], S["angle"] * 3.0, e / 3.0, dh * 3.0, (vt - v) / 20.0, *ks,
         base["steer"], base["accel"] - base["brake"], S.get("speedY", 0.0) / 30.0, spin / 10.0,
         S["rpm"] / 20000.0, S["gear"] / 6.0, prev_a[0], prev_a[1], *harm]
    return np.clip(np.asarray(o, dtype=np.float32), -5, 5)


def apply_residual(base, a, S, d_steer, d_pedal, tcs):
    cmd = dict(base)
    cmd["steer"] = float(np.clip(base["steer"] + float(a[0]) * d_steer, -1, 1))
    pedal = float(np.clip(base["accel"] - base["brake"] + float(a[1]) * d_pedal, -1, 1))
    accel, brake = max(pedal, 0.0), max(-pedal, 0.0)
    if accel > 0 and a[1] > 0:  # artırılan gaz için patinaj kontrolü
        accel = min(accel, max(base["accel"], common.traction_control(accel, S, tcs)))
    if brake > 0:
        brake = common.abs_brake(brake, S)
    cmd["accel"], cmd["brake"] = accel, brake
    return cmd


class ResidualEnv:
    def __init__(self, idx, params_json, d_steer=0.08, d_pedal=0.15, drop_prob=0.0, p_delay=0.3, seed=0,
                 c_act=0.02, max_steps=6000, c_off=0.5, off_end=1.2, **_):
        from build_line_submission import load_params
        self.params = load_params(params_json)
        self.params["ctrl"] = "st"
        self.idx, self.d_steer, self.d_pedal = idx, d_steer, d_pedal
        self.drop_prob, self.p_delay, self.c_act, self.c_off = drop_prob, p_delay, c_act, c_off
        self.off_end = off_end
        self.obs_noise = 0.0
        self.max_steps = max_steps
        self.rng = np.random.default_rng(seed)
        self.env = TorcsEnv(idx=idx, udp_timeout_us=1000000)
        self.ep_return = 0.0

    def _send(self, cmd):
        from sim import skeleton_msg
        self.env.sock.sendto(skeleton_msg(cmd).encode(), ("127.0.0.1", self.env.port))

    def reset(self):
        from try_one import make_driver
        self.drv, _ = make_driver(self.params)
        for _ in range(3):
            try:
                self.env._launch()
                s = self.env._recv()
                if s is not None:
                    break
            except RuntimeError:
                pass
        self.s = s
        self.delay = self.rng.random() < self.p_delay
        self.pending = None
        self.last_sent = None
        self.prev_a = np.zeros(2, np.float32)
        self.steps, self.ep_return = 0, 0.0
        self.start = s["distRaced"]
        self.prev_dist = s["distRaced"]
        self.prev_dmg = s["damage"]
        self.last_lap = s["lastLapTime"]
        self.base = self.drv.act(s)
        return res_obs(s, self.drv, self.base, self.prev_a)

    def step(self, a):
        a = np.clip(np.asarray(a, np.float32), -1, 1)
        cmd = apply_residual(self.base, a, self.s, self.d_steer, self.d_pedal, self.params.get("tcs", 6.75))
        out = cmd
        if self.delay:  # kalıcı 1 adım gecikme
            out, self.pending = (self.pending or {"accel": 0.0, "brake": 0.0, "gear": 0, "steer": 0.0, "clutch": 0.0}), cmd
        if self.drop_prob and self.last_sent is not None and self.rng.random() < self.drop_prob:
            out = self.last_sent  # adım kaçırma taklidi: sunucu önceki komutu kullanır
        self._send(out)
        self.last_sent = out
        s = self.env._recv()
        self.steps += 1
        info = {"lap_time": None, "crash": False, "offtrack": False, "stuck": False, "truncated": False}
        if s is None:
            self.env._kill()
            info["truncated"] = True
            info["distance"] = self.prev_dist - self.start
            info["damage"] = self.prev_dmg
            return res_obs(self.s, self.drv, self.base, a), 0.0, True, info
        prog = s["distRaced"] - self.prev_dist
        r = 0.1 * prog - self.c_act * float(np.sum(a * a))
        pos = abs(s["trackPos"])
        if pos > 1.0:
            r -= self.c_off * (pos - 1.0)
        done = False
        if s["damage"] > self.prev_dmg:
            r -= 20.0
            done = info["crash"] = True
        if pos > self.off_end:
            r -= 20.0
            done = info["offtrack"] = True
        if s["lastLapTime"] > 0 and s["lastLapTime"] != self.last_lap:
            info["lap_time"] = s["lastLapTime"]
            r += 10.0
            done = True
        if self.steps > 300 and s["speedX"] < 5:
            done = info["stuck"] = True
        if self.steps >= self.max_steps:
            done = info["truncated"] = True
        info["distance"] = s["distRaced"] - self.start
        info["damage"] = s["damage"]
        self.prev_dist, self.prev_dmg, self.s = s["distRaced"], s["damage"], s
        self.prev_a = a
        self.ep_return += r
        if done:
            self.env._kill()
            o = res_obs(s, self.drv, self.base, a)
        else:
            self.base = self.drv.act(s)
            o = res_obs(s, self.drv, self.base, a)
        return o, float(r), done, info

    def close(self):
        self.env._kill()
