"""Windows'ta çalışan, menüsüz TORCS RL ortamı.

Her episode için `wtorcs.exe -r <config> -t <timeout>` başlatır (grafiksiz, gerçek
zamandan çok daha hızlı, agent cevabını bekleyen senkron mod). Episode erken bitince
TORCS süreci öldürülür, reset'te yenisi açılır (~0.1-0.5 s).

Farklı `idx` değerleri farklı scr_server portlarını kullanır (idx i → port 3001+i),
böylece birden çok ortam paralel koşabilir. idx 0 (port 3001) GUI/benchmark için
boş bırakılır.
"""
import os
import socket
import subprocess
import time

import numpy as np

from common import INIT_MSG, OBS_DIM, ACT_DIM, command_msg, make_obs, parse, to_command

RACELAB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(RACELAB)
TORCS_DIR = os.path.join(ROOT, "torcs")
TEMPLATE = os.path.join(RACELAB, "configs", "corkscrew_1lap.xml")
GEN_DIR = os.path.join(RACELAB, "configs", "train")


def make_config(idx, track="corkscrew", laps=1):
    """scr_server idx'ine göre eğitim config'i üretir (racelab/configs/train altında)."""
    os.makedirs(GEN_DIR, exist_ok=True)
    with open(TEMPLATE, encoding="utf-8") as f:
        xml = f.read()
    xml = xml.replace('<attnum name="idx" val="0"/>', f'<attnum name="idx" val="{idx}"/>')
    xml = xml.replace('val="corkscrew"', f'val="{track}"')
    xml = xml.replace('<attnum name="laps" val="1"/>', f'<attnum name="laps" val="{laps}"/>')
    path = os.path.join(GEN_DIR, f"{track}_idx{idx}_{laps}lap.xml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(xml)
    return os.path.relpath(path, TORCS_DIR)


def kill_training_torcs(idxs=None):
    """SADECE eğitim config'iyle (-r ...configs\\train...) açılmış TORCS'ları kapatır.
    Elle açılmış GUI TORCS'a dokunmaz. `idxs` verilirse yalnızca o scr_server idx'lerine
    ait olanlar kapatılır (paralel çalışan başka bir eğitimin ortamlarına dokunulmaz)."""
    match = "$_.CommandLine -like '*configs\\train*'"
    if idxs is not None:
        ors = " -or ".join(f"$_.CommandLine -like '*_idx{i}_*'" for i in idxs)
        match = f"{match} -and ({ors})"
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='wtorcs.exe'\" | "
          f"Where-Object {{ {match} }} | "
          "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }")
    subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class TorcsEnv:
    def __init__(self, idx=1, track="corkscrew", max_steps=12000, udp_timeout_us=1000000,
                 offtrack_limit=1.0, stuck_steps=150):
        self.idx = idx
        self.port = 3001 + idx
        self.cfg = make_config(idx, track)
        self.max_steps = max_steps
        self.udp_timeout_us = udp_timeout_us
        self.offtrack_limit = offtrack_limit
        self.stuck_steps = stuck_steps
        self.proc = None
        self.sock = None
        self.observation_dim = OBS_DIM
        self.action_dim = ACT_DIM

    # ------------------------------------------------------------- süreç yönetimi
    def _kill(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        self.proc = None
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _launch(self):
        self._kill()
        self.proc = subprocess.Popen(
            [os.path.join(TORCS_DIR, "wtorcs.exe"), "-r", self.cfg, "-t", str(self.udp_timeout_us)],
            cwd=TORCS_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        t0 = time.time()
        while True:
            if time.time() - t0 > 20:
                raise RuntimeError(f"TORCS port {self.port} cevap vermedi")
            self.sock.sendto(INIT_MSG.encode(), ("127.0.0.1", self.port))
            try:
                data, _ = self.sock.recvfrom(4096)
            except OSError:
                continue
            if b"***identified***" in data:
                break
        self.sock.settimeout(5.0)

    def _recv(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(8192)
            except socket.timeout:
                return None
            except OSError:
                return None
            msg = data.decode("utf-8", "replace")
            if "***shutdown***" in msg or "***restart***" in msg:
                return None
            if "***identified***" in msg:
                continue
            return parse(msg)

    # ------------------------------------------------------------- RL arayüzü
    def reset(self):
        for _ in range(3):
            try:
                self._launch()
                s = self._recv()
                if s is not None:
                    break
            except RuntimeError:
                time.sleep(1)
        else:
            raise RuntimeError("TORCS başlatılamadı")
        self.s = s
        self.prev_action = np.zeros(ACT_DIM, dtype=np.float32)
        self.steps = 0
        self.slow_steps = 0
        self.start_dist = s["distRaced"]
        self.prev_dist = s["distRaced"]
        self.prev_damage = s["damage"]
        self.prev_last_lap = s["lastLapTime"]
        self.ep_return = 0.0
        return make_obs(s, self.prev_action)

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        cmd = to_command(action, self.s)
        self.sock.sendto(command_msg(cmd).encode(), ("127.0.0.1", self.port))
        s = self._recv()
        self.steps += 1
        info = {"lap_time": None, "crash": False, "offtrack": False, "stuck": False}

        if s is None:  # TORCS yarışı kapattı (tur limiti vb.)
            self._kill()
            info["truncated"] = True
            return make_obs(self.s, action), 0.0, True, info

        progress = s["distRaced"] - self.prev_dist
        d_damage = s["damage"] - self.prev_damage
        reward = progress * 0.5                       # ilerleme (m/adım; 200 km/h ≈ 1.1)
        reward -= 0.05 * abs(action[0] - self.prev_action[0])   # direksiyon titremesi cezası
        reward -= 0.01                                # zaman cezası: hızlı bitirmeye teşvik

        done, truncated = False, False
        if d_damage > 0:                              # yarışma hedefi: sıfır hasar
            reward -= 10.0
            done = info["crash"] = True
        if abs(s["trackPos"]) > self.offtrack_limit:
            reward -= 10.0
            done = info["offtrack"] = True
        if s["lastLapTime"] > 0 and s["lastLapTime"] != self.prev_last_lap:
            info["lap_time"] = s["lastLapTime"]
            reward += 20.0
            done = True
        self.slow_steps = self.slow_steps + 1 if s["speedX"] < 5 else 0
        if self.steps > 100 and self.slow_steps > self.stuck_steps:
            reward -= 5.0
            done = info["stuck"] = True
        if self.steps >= self.max_steps:
            done = truncated = True

        info["truncated"] = truncated and not (info["crash"] or info["offtrack"] or info["stuck"])
        info["distance"] = s["distRaced"] - self.start_dist
        info["damage"] = s["damage"]
        self.prev_dist, self.prev_damage, self.prev_last_lap = s["distRaced"], s["damage"], s["lastLapTime"]
        self.prev_action = action
        self.s = s
        self.ep_return += reward
        if done:
            self._kill()
        return make_obs(s, action), float(reward), done, info

    def close(self):
        self._kill()
