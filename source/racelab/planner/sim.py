"""Bir sürücüyle tek tur koştur (hızlı mod, eğitim portları idx 1–9). Kural tabanlı sürücüler için.

    from sim import run_lap
    r = run_lap(idx, driver, udp_timeout_us=1000000)   # deterministik (= GUI)
    r = run_lap(idx, driver, udp_timeout_us=10000)     # 10 ms bekleme (zorlamalı)
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "rl"))
from common import command_msg  # noqa: E402
from torcs_env import TorcsEnv  # noqa: E402


def skeleton_msg(cmd):
    """gym_torcs/torcs_jm_par.py DriverAction.__repr__ ile aynı biçim (%.3f, aynı sıra, focus listesi)."""
    steer = min(1.0, max(-1.0, cmd["steer"]))
    brake = min(1.0, max(0.0, cmd["brake"]))
    accel = min(1.0, max(0.0, cmd["accel"]))
    gear = cmd["gear"] if cmd["gear"] in (-1, 0, 1, 2, 3, 4, 5, 6) else 0
    clutch = min(1.0, max(0.0, cmd.get("clutch", 0.0)))
    return ("(accel %.3f)(brake %.3f)(clutch %.3f)(gear %.3f)(steer %.3f)(focus -90 -45 0 45 90)(meta %.3f)"
            % (accel, brake, clutch, gear, steer, 0))


def run_lap(idx, driver, udp_timeout_us=1000000, max_steps=9000, record=False, offtrack_stop=1.3, fmt="skeleton",
            drop_prob=0.0, seed=None, drop_start=False, delay1=False):
    """drop_prob > 0: o olasılıkla cevap gönderilmez (TORCS 10 ms bekleyip önceki komutu kullanır) → zaman aşımı taklidi."""
    import random
    rng = random.Random(seed)
    env = TorcsEnv(idx=idx, udp_timeout_us=udp_timeout_us)
    tele = []
    res = dict(lap=None, damage=0.0, max_pos=0.0, dist=0.0, steps=0, vmax=0.0, reason="")
    try:
        for _ in range(3):
            try:
                env._launch()
                s = env._recv()
                if s is not None:
                    break
            except RuntimeError:
                time.sleep(1)
        else:
            res["reason"] = "launch"
            return res
        last = s["lastLapTime"]
        start = s["distRaced"]
        prev_cmd = None
        for step in range(max_steps):
            cmd = driver.act(s)
            if delay1:  # kalıcı 1 adım gecikme: sunucuya bir önceki gözleme ait komut gider
                cmd, prev_cmd = (prev_cmd or {"accel": 0.0, "brake": 0.0, "gear": 0, "steer": 0.0}), cmd
            msg = skeleton_msg(cmd) if fmt == "skeleton" else command_msg(cmd)
            # drop_start: yarış başındaki ilk 10 adımda bir kaçırma (gerçek 10 ms koşularında sık görülür)
            start_drop = drop_start and step == getattr(rng, "_sd", None)
            if step == 0 and drop_start:
                rng._sd = rng.randint(1, 10)
            if start_drop or (drop_prob and step > 5 and rng.random() < drop_prob):
                continue_send = False
            else:
                continue_send = True
            if continue_send:
                env.sock.sendto(msg.encode(), ("127.0.0.1", env.port))
            s2 = env._recv()
            if s2 is None:
                res["reason"] = "closed"
                if res["dist"] > 3590 and s.get("curLapTime", 0) > 30:  # bitiş mesajı kapanışla birlikte geldi
                    res["lap"] = round(s["curLapTime"] + 0.02, 3)
                    res["reason"] = "lap_closed"
                break
            s = s2
            res["steps"] = step + 1
            res["damage"] = s["damage"]
            res["dist"] = s["distRaced"] - start
            res["vmax"] = max(res["vmax"], s["speedX"])
            if res["dist"] > 50:  # kalkış ızgarası pist ortasında değil
                res["max_pos"] = max(res["max_pos"], abs(s["trackPos"]))
            if record:
                tele.append((s["distFromStart"], s["speedX"], s["trackPos"], s["angle"], cmd["steer"],
                             cmd["accel"], cmd["brake"], s["damage"], s["curLapTime"]))
            if s["lastLapTime"] > 0 and s["lastLapTime"] != last:
                res["lap"] = s["lastLapTime"]
                res["reason"] = "lap"
                break
            if abs(s["trackPos"]) > offtrack_stop:
                res["reason"] = "offtrack"
                break
            if step > 200 and s["speedX"] < 3:
                res["reason"] = "stuck"
                break
    finally:
        env._kill()
    if record:
        res["tele"] = np.array(tele)
    return res
