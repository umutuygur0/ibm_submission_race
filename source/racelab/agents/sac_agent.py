"""Eğitilmiş SAC politikasıyla yarışan agent (snakeoil gibi tek başına çalışır).

    python racelab/agents/sac_agent.py --model racelab/rl/runs/sac_v1/best.pt
    python racelab/agents/sac_agent.py -p 3001 --model ...

GUI'de: TORCS → Quick Race → Corkscrew, sürücü `scr_server 1`, New Race (mavi ekran),
sonra bu script. Benchmark: run_benchmark.py --agent racelab/agents/sac_agent.py --model ...
Vites/ABS/patinaj kontrolü eğitimdekiyle aynı kurallardır (racelab/rl/common.py).
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

DEFAULT_MODEL = os.path.join(os.path.dirname(HERE), "rl", "runs", "sac_v1", "best.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", type=int, default=3001)
    ap.add_argument("-H", "--host", default="localhost")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    args, _ = ap.parse_known_args()  # snakeoil'e özgü diğer argümanları yok say

    torch.set_num_threads(1)
    # best.pt eval bilgisini numpy sayılarıyla tutar; kendi eğitimimizin dosyası → güvenli
    ck = torch.load(args.model, map_location="cpu", weights_only=False)
    actor = Actor(OBS_DIM, ACT_DIM, tuple(ck.get("hidden", [256, 256])))
    actor.load_state_dict(ck["actor"])
    actor.eval()
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
    while True:
        try:
            data, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        msg = data.decode("utf-8", "replace")
        if "***shutdown***" in msg or "***restart***" in msg:
            print("Race over.", flush=True)
            break
        if "***identified***" in msg:
            continue
        s = parse(msg)
        with torch.no_grad():
            a, _ = actor(torch.as_tensor(make_obs(s, prev)).unsqueeze(0), deterministic=True, with_logp=False)
        a = a.squeeze(0).numpy()
        sock.sendto(command_msg(to_command(a, s)).encode(), addr)
        prev = a
    sock.close()


if __name__ == "__main__":
    main()
