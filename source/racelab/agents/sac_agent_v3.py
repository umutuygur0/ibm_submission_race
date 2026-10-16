"""SAC yarış agent'ı v3 — v2 + direksiyon hız sınırı (torcs_env_v3 ile eğitilen modeller için).

Direksiyon her adımda en fazla smooth_steer() kadar değişir; eğitimdekiyle birebir aynı
fonksiyon (rl/torcs_env_v3.py). v2 modellerini bu agent ile değil sac_agent_v2.py ile koştur.

sac_agent_v2'den devralınanlar
1. Isınma: bağlanmadan önce model birkaç kez boş çalıştırılır. PyTorch'un ilk çağrısı
   yavaştır; v1 yarış başında bu yüzden TORCS'un 10 ms bekleme süresini kaçırıyordu.
2. Kuyruk boşaltma: her adımda soketteki tüm bekleyen sensör mesajları okunur, sadece
   en yenisine cevap verilir. scr_server her adımda bir cevap okur; agent geride kalıp
   her eski mesaja da cevap verirse TORCS bayat cevabı sonraki adımın cevabı sanır ve
   agent kalıcı olarak 1+ adım gecikmeli sürer. v2 bu birikmeyi engeller.

    python racelab/agents/sac_agent_v3.py --model racelab/rl/runs/v3_ft/best.pt
"""
import argparse
import os
import socket
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "rl"))
from common import ACT_DIM, INIT_MSG, OBS_DIM, command_msg, make_obs, parse, to_command  # noqa: E402
from sac import Actor  # noqa: E402
from torcs_env_v3 import smooth_steer  # noqa: E402

DEFAULT_MODEL = os.path.join(os.path.dirname(HERE), "rl", "runs", "v3_ft", "best.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", type=int, default=3001)
    ap.add_argument("-H", "--host", default="localhost")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    args, _ = ap.parse_known_args()

    torch.set_num_threads(1)
    ck = torch.load(args.model, map_location="cpu", weights_only=False)
    actor = Actor(OBS_DIM, ACT_DIM, tuple(ck.get("hidden", [256, 256])))
    actor.load_state_dict(ck["actor"])
    actor.eval()
    with torch.no_grad():  # ısınma
        for _ in range(50):
            actor(torch.zeros(1, OBS_DIM), deterministic=True, with_logp=False)
    print(f"Model: {args.model}", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    addr = (args.host, args.port)
    while True:
        sock.sendto(INIT_MSG.encode(), addr)
        try:
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            print(f"Waiting for server on {args.port}...", flush=True)
            continue
        if b"***identified***" in data:
            print(f"Client connected on {args.port}", flush=True)
            break

    prev = np.zeros(ACT_DIM, dtype=np.float32)
    skipped = 0
    while True:
        sock.settimeout(1.0)
        try:
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        # birikmiş mesaj varsa en yenisine geç
        sock.setblocking(False)
        while True:
            try:
                newer, _ = sock.recvfrom(8192)
            except (BlockingIOError, OSError):
                break
            if b"***" in newer:
                data = newer
                break
            data = newer
            skipped += 1
        msg = data.decode("utf-8", "replace")
        if "***shutdown***" in msg or "***restart***" in msg:
            print(f"Race over. (atlanan eski mesaj: {skipped})", flush=True)
            break
        if "***identified***" in msg:
            continue
        s = parse(msg)
        with torch.no_grad():
            a, _ = actor(torch.as_tensor(make_obs(s, prev)).unsqueeze(0), deterministic=True, with_logp=False)
        a = a.squeeze(0).numpy().copy()
        a[0] = smooth_steer(a[0], prev[0], s["speedX"])
        sock.sendto(command_msg(to_command(a, s)).encode(), addr)
        prev = a
    sock.close()


if __name__ == "__main__":
    main()
