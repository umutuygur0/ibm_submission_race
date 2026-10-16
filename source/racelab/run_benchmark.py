"""Bir agent'ı yarışma koşulunda koşturur ve sonucu skor tablosuna kaydeder.

Agent dosyasına dokunmaz: agent'ı `-p <proxy_port>` ile başlatır, telemetri proxy'si
trafiği TORCS'a iletirken her adımı kaydeder.

Modlar
  fast : wtorcs.exe -r ile menüsüz/grafiksiz yarış (gerçek zamandan ~30x hızlı)
  gui  : TORCS'u sen elle açıp "New Race" ile mavi bekleme ekranına getirirsin,
         script sadece proxy + agent'ı çalıştırır ve kaydeder.

Örnekler
  python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py
  python racelab/run_benchmark.py --agent gym_torcs/torcs_jm_par.py --repeat 3
  python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py --mode gui
"""
import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "tools"))
from racelog import TelemetryProxy  # noqa: E402

TORCS_DIR = os.path.join(ROOT, "torcs")
RESULTS = os.path.join(HERE, "results")
LEADERBOARD = os.path.join(RESULTS, "leaderboard.csv")
LAP_LOG = os.path.join(RESULTS, "LAP_LOG.md")

COLUMNS = ["timestamp", "run_id", "agent", "label", "mode", "config", "finished", "lap_time_s",
           "damage", "hits", "offtrack_events", "offtrack_time_s", "top_speed_kmh",
           "avg_speed_kmh", "distance_m", "sim_time_s", "server_timeouts", "note"]


def run_once(agent, label, mode, config, torcs_port, proxy_port, timeout, note, agent_args,
             udp_timeout_us):
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = f"{stamp}_{label}"
    run_dir = os.path.join(RESULTS, "runs", run_id)
    os.makedirs(run_dir, exist_ok=True)

    torcs = None
    torcs_log = open(os.path.join(run_dir, "torcs_stdout.txt"), "wb")
    if mode == "fast":
        # -r yolu wtorcs.exe klasörüne göre verilmeli
        cfg_rel = os.path.relpath(os.path.join(HERE, "configs", config), TORCS_DIR)
        torcs = subprocess.Popen([os.path.join(TORCS_DIR, "wtorcs.exe"), "-r", cfg_rel, "-t", str(udp_timeout_us)],
                                 cwd=TORCS_DIR, stdout=torcs_log, stderr=subprocess.STDOUT)
        time.sleep(1.5)

    proxy = TelemetryProxy(proxy_port, torcs_port, live_csv=os.path.join(run_dir, "telemetry.csv"))
    proxy.start()

    agent_path = os.path.abspath(agent)
    agent_log = open(os.path.join(run_dir, "agent_stdout.txt"), "wb")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONWARNINGS="ignore")
    ap = subprocess.Popen([sys.executable, agent_path, "-p", str(proxy_port)] + agent_args,
                          cwd=os.path.dirname(agent_path), stdout=agent_log,
                          stderr=subprocess.STDOUT, env=env)

    t0 = time.time()
    while time.time() - t0 < timeout:
        if torcs is not None and torcs.poll() is not None:
            break
        if proxy.shutdown_seen:
            time.sleep(0.5)
            break
        if ap.poll() is not None and not proxy.rows:
            break  # agent çöktü
        time.sleep(0.2)
    wall = time.time() - t0

    for p in (ap, torcs):
        if p is not None and p.poll() is None:
            p.kill()
            p.wait()
    proxy.stop()
    torcs_log.close()
    agent_log.close()

    summ = proxy.summary(laps=1)
    timeouts = 0
    with open(os.path.join(run_dir, "torcs_stdout.txt"), "rb") as f:
        timeouts = f.read().count(b"Timeout for client answer")
    summ.update({"run_id": run_id, "agent": os.path.relpath(agent_path, ROOT).replace("\\", "/"),
                 "label": label, "mode": mode, "config": config if mode == "fast" else "gui (TORCS menüsü)", "wall_s": round(wall, 1),
                 "server_timeouts": timeouts, "note": note})
    proxy.write_csv(os.path.join(run_dir, "telemetry.csv"))
    with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summ, f, indent=2, ensure_ascii=False)

    if summ.get("steps", 0) == 0:
        print(f"!! agent bağlanamadı, skor tablosuna yazılmadı. Log: {run_dir}")
        return summ
    append_leaderboard(summ, stamp)
    write_lap_log()
    return summ


def append_leaderboard(s, stamp):
    new = not os.path.exists(LEADERBOARD)
    with open(LEADERBOARD, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        if new:
            w.writeheader()
        row = dict(s, timestamp=stamp)
        w.writerow(row)


def write_lap_log():
    with open(LEADERBOARD, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    def key(r):
        # bitirenler: önce hasarsız, sonra tur süresi; bitiremeyenler: en uzağa giden önce
        if r["finished"] == "True":
            return (0, int(r["damage"] or 0) > 0, float(r["lap_time_s"]), 0)
        return (1, 0, 0, -float(r["distance_m"] or 0))

    best, runs, done = {}, {}, {}
    for r in rows:
        lb = r["label"]
        runs[lb] = runs.get(lb, 0) + 1
        done[lb] = done.get(lb, 0) + (r["finished"] == "True")
        if lb not in best or key(r) < key(best[lb]):
            best[lb] = r

    lines = ["# Lap Log — skor tablosu",
             "",
             "Otomatik üretilir (`racelab/run_benchmark.py`). Ham veri: `leaderboard.csv`,",
             "her koşunun telemetrisi: `runs/<run_id>/telemetry.csv`.",
             "",
             "Sıralama: önce turu bitirenler (hasarsız olan önde, sonra tur süresi),",
             "sonra bitiremeyenler (en uzağa giden önde). Pist: Corkscrew ≈ 3608 m.",
             "`@gui-timing` = TORCS'un GUI'deki 10 ms bekleme süresiyle koşu (gerçek koşula yakın, rastgelelik var).",
             "",
             "## Her agent'ın en iyi koşusu",
             "",
             "| # | Agent (label) | Bitirme | Tur süresi (s) | Mesafe (m) | Hasar | Çarpma | Pist dışı (kez / s) | Max hız | Ort. hız | Koşu |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(sorted(best.values(), key=key), 1):
        lb = r["label"]
        lines.append(f"| {i} | {lb} | {done[lb]}/{runs[lb]} | {r['lap_time_s'] or 'DNF'} | {r['distance_m']} | "
                     f"{r['damage']} | {r['hits']} | {r['offtrack_events']} / {r['offtrack_time_s']} | "
                     f"{r['top_speed_kmh']} | {r['avg_speed_kmh']} | `{r['run_id']}` |")
    lines += ["", "## Tüm koşular (yeniden eskiye)", "",
              "| Zaman | Label | Tur (s) | Mesafe (m) | Hasar | Çarpma | Pist dışı | Max hız | Mod | Not |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in reversed(rows):
        lines.append(f"| {r['timestamp']} | {r['label']} | {r['lap_time_s'] or 'DNF'} | {r['distance_m']} | "
                     f"{r['damage']} | {r['hits']} | {r['offtrack_events']} | {r['top_speed_kmh']} | "
                     f"{r['mode']} | {r['note']} |")
    with open(LAP_LOG, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--agent", required=True, help="agent .py dosyası (-p <port> almalı)")
    ap.add_argument("--label", help="skor tablosundaki ad (varsayılan: dosya adı)")
    ap.add_argument("--mode", choices=["fast", "gui"], default="fast")
    ap.add_argument("--config", default="corkscrew_1lap.xml", help="racelab/configs altındaki yarış dosyası")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--torcs-port", type=int, default=3001)
    ap.add_argument("--proxy-port", type=int, default=3101)
    ap.add_argument("--timeout", type=float, default=None, help="saniye (fast:300, gui:900)")
    ap.add_argument("--note", default="")
    ap.add_argument("--udp-timeout-us", type=int, default=1000000,
                    help="fast modda TORCS'un agent cevabını bekleme süresi (mikrosaniye). "
                         "Büyük değer = adım kaçırma yok, tekrarlanabilir sonuç. GUI varsayılanı 10000")
    args, agent_args = ap.parse_known_args()
    # agent kendi klasöründe çalışır; bizim cwd'ye göre verilen dosya yollarını mutlak yap
    agent_args = [os.path.abspath(a) if os.path.exists(a) else a for a in agent_args]

    label = args.label or os.path.splitext(os.path.basename(args.agent))[0]
    timeout = args.timeout or (300 if args.mode == "fast" else 900)
    os.makedirs(os.path.join(RESULTS, "runs"), exist_ok=True)
    for i in range(args.repeat):
        s = run_once(args.agent, label, args.mode, args.config, args.torcs_port,
                     args.proxy_port, timeout, args.note, agent_args, args.udp_timeout_us)
        print(f"[{i + 1}/{args.repeat}] {label}: bitirdi={s['finished']} tur={s.get('lap_time_s')}s "
              f"hasar={s.get('damage')} çarpma={s.get('hits')} pist_dışı={s.get('offtrack_events')} "
              f"mesafe={s.get('distance_m')}m max={s.get('top_speed_kmh')}km/h timeouts={s.get('server_timeouts')} (duvar {s.get('wall_s')}s)")


if __name__ == "__main__":
    main()
