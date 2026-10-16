"""Hızlı deney aracı: eğitilmiş bir modeli sürüş yardımcısı / donanım ayarı varyantlarıyla koşturur.

    python racelab/rl/quick_exp.py --model snapshots/v6g_best.pt --tcs 5 --assist-front 180 --gui 10

- Deterministik (senkron, = gerçek GUI) 1 koşu + `--gui` kadar 10 ms bekleme süreli koşu.
- --assist-front F: önü ≥ F m açık, |açı| < assist_angle, |trackPos| < assist_pos iken gaz = 1
  (politika fren istiyorsa dokunulmaz). Düzlükte temkinli gazı test etmek için.
- --upshift R: vites büyütme devri (varsayılan 18000).
Sonuç satırları racelab/rl/runs/quick_exp.csv'ye eklenir (üzerine yazılmaz).
"""
import argparse
import csv
import datetime as dt
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import common  # noqa: E402
from common import ACT_DIM, OBS_DIM  # noqa: E402
from sac import Actor  # noqa: E402
from torcs_env_v2 import TorcsEnvV2  # noqa: E402


def run(actor, args, udp_timeout):
    env = TorcsEnvV2(idx=args.idx, race_mode=True, udp_timeout_us=udp_timeout, max_steps=30000,
                     tcs_threshold=args.tcs, pedal_gain=args.pedal_gain, pedal_bias=args.pedal_bias,
                     pos_obs=args.pos_obs)
    o = env.reset()
    assisted = steps = 0
    while True:
        with torch.no_grad():
            a, _ = actor(torch.as_tensor(o).unsqueeze(0), deterministic=True, with_logp=False)
        a = a.squeeze(0).numpy().copy()
        s = env.s
        if (args.assist_front and s["track"][9] >= args.assist_front and abs(s["angle"]) < args.assist_angle
                and abs(s["trackPos"]) < args.assist_pos and a[1] > 0):
            a[1] = 1.0
            assisted += 1
        elif args.assist_dec:  # mesafe tabanlı: önü frenleme mesafesinden (v²/2a + pay) uzunsa tam gaz
            vms = max(s["speedX"], 0) / 3.6
            need = vms * vms / (2 * args.assist_dec * 9.81) + args.assist_margin + vms * args.assist_react
            if (s["track"][9] >= need and abs(s["angle"]) < args.assist_angle
                    and abs(s["trackPos"]) < args.assist_pos and a[1] > args.assist_min_pedal):
                a[1] = 1.0
                assisted += 1
        steps += 1
        o, r, d, info = env.step(a)
        if d:
            break
    env.close()
    return {"lap": info.get("lap_time"), "damage": info.get("damage", 0), "dist": round(info.get("distance", 0)),
            "assist_frac": round(assisted / max(steps, 1), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--tcs", type=float, default=5.0)
    ap.add_argument("--upshift", type=float, default=18000)
    ap.add_argument("--assist-front", type=float, default=0.0)
    ap.add_argument("--assist-angle", type=float, default=0.05)
    ap.add_argument("--assist-pos", type=float, default=0.7)
    ap.add_argument("--gui", type=int, default=0, help="10 ms bekleme süreli ek koşu sayısı")
    ap.add_argument("--idx", type=int, default=9)
    ap.add_argument("--tag", default="")
    ap.add_argument("--abs-th", type=float, default=2.0, help="ABS kayma eşiği m/s (varsayılan 2.0); <=0 kapalı")
    ap.add_argument("--assist-dec", type=float, default=0.0, help="mesafe tabanlı yardımcı: varsayılan fren ivmesi (g); 0=kapalı")
    ap.add_argument("--assist-margin", type=float, default=40.0, help="mesafe tabanlı yardımcı payı (m)")
    ap.add_argument("--assist-min-pedal", type=float, default=0.0, help="politika pedalı bundan büyükse yardımcı devreye girer")
    ap.add_argument("--assist-react", type=float, default=0.0, help="tepki payı (s): gerekli mesafeye hız×t eklenir (gecikme dayanıklılığı)")
    ap.add_argument("--pedal-gain", type=float, default=1.0)
    ap.add_argument("--pedal-bias", type=float, default=0.0)
    args = ap.parse_args()

    common.UPSHIFT_RPM = args.upshift  # auto_gear modül sabitini okur
    if args.abs_th != 2.0:  # ABS varyantı (sadece bu deney sürecinde)
        th = args.abs_th

        def abs_brake(brake, s):
            if th <= 0 or brake <= 0 or s["speedX"] < 10:
                return brake
            v = s["speedX"] / 3.6
            wheel_v = np.mean([w * r for w, r in zip(s["wheelSpinVel"], common.WHEEL_RADIUS)])
            slip = v - wheel_v
            if slip > th:
                brake = brake - min(brake, (slip - th) * 0.1)
            return brake
        common.abs_brake = abs_brake
    torch.set_num_threads(1)
    ck = torch.load(args.model, map_location="cpu", weights_only=False)
    obs_dim = ck["actor"]["net.0.weight"].shape[1]  # 32 klasik, 44 konum bilen (v10)
    args.pos_obs = common.pos_mode_for(obs_dim)
    actor = Actor(obs_dim, ACT_DIM, tuple(ck.get("hidden", [256, 256])))
    actor.load_state_dict(ck["actor"])

    det = run(actor, args, 1000000)
    guis = [run(actor, args, 10000) for _ in range(args.gui)]
    clean = [g for g in guis if g["lap"] and g["damage"] == 0]
    row = {"time": dt.datetime.now().strftime("%m-%d %H:%M"), "tag": args.tag, "model": os.path.basename(args.model),
           "tcs": args.tcs, "upshift": args.upshift, "abs_th": args.abs_th, "assist_dec": args.assist_dec, "assist_margin": args.assist_margin, "assist_react": args.assist_react,
           "pedal_gain": args.pedal_gain, "assist_front": args.assist_front,
           "assist_angle": args.assist_angle, "assist_pos": args.assist_pos,
           "det_lap": det["lap"], "det_damage": det["damage"], "det_dist": det["dist"], "assist_frac": det["assist_frac"],
           "gui_n": args.gui, "gui_clean": len(clean),
           "gui_mean_lap": round(float(np.mean([g["lap"] for g in clean])), 3) if clean else None}
    print(row, flush=True)
    out = os.path.join(HERE, "runs", "quick_exp_v4.csv")  # abs_th sütunu eklendi (v1 dosyası korunur)
    new = not os.path.exists(out)
    with open(out, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)


if __name__ == "__main__":
    main()
