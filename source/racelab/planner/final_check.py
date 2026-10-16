"""Teslim dosyalarının final doğrulaması (sistem boşken çalıştır): run_benchmark det + N×10 ms.

    python racelab/planner/final_check.py --agents racelab/submission/a.py racelab/submission/b.py --repeat 30
Özet: racelab/planner/runs/final_check_<zaman>.txt (yeni dosya)
"""
import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PAT = re.compile(r"bitirdi=(\w+) tur=([\d.]+|None)s? hasar=([\d.]+).*?pist_dışı=(\d+).*?timeouts=(\d+)")


EXTRA = []


def bench(agent, label, repeat, timeout_us):
    cmd = [sys.executable, os.path.join(ROOT, "racelab", "run_benchmark.py"), "--agent", agent, "--mode", "fast",
           "--label", label, "--repeat", str(repeat)] + EXTRA
    if timeout_us:
        cmd += ["--udp-timeout-us", str(timeout_us)]
    out = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace",
                         env=dict(os.environ, PYTHONIOENCODING="utf-8")).stdout
    rows = []
    for m in PAT.finditer(out):
        fin, lap, dmg, off, to = m.groups()
        rows.append(dict(fin=fin == "True", lap=float(lap) if lap != "None" else None, dmg=float(dmg), off=int(off),
                         to=int(to)))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--agents", nargs="+", required=True)
    ap.add_argument("--repeat", type=int, default=30)
    ap.add_argument("--idx9", action="store_true", help="port 3010 (idx 9) yapılandırmasıyla (3001 GUI'ye ayrılı)")
    a = ap.parse_args()
    if a.idx9:
        EXTRA.extend(["--config", "train/corkscrew_idx9_1lap.xml", "--torcs-port", "3010", "--proxy-port", "3110"])
    out = os.path.join(HERE, "runs", f"final_check_{time.strftime('%Y%m%d-%H%M%S')}.txt")
    lines = []
    for ag in a.agents:
        name = os.path.basename(ag)
        det = bench(ag, f"FINAL_{name}_det", 1, 0)
        rr = bench(ag, f"FINAL_{name}_x{a.repeat}@gui-timing", a.repeat, 10000)
        clean = [r for r in rr if r["fin"] and r["dmg"] == 0]
        clean_on = [r for r in clean if r["off"] == 0]
        laps = [r["lap"] for r in clean if r["lap"]]
        d = det[0] if det else {}
        line = (f"{name}: det {d.get('lap')} hasar {d.get('dmg')} pist_dışı {d.get('off')} | 10ms ×{len(rr)}: "
                f"hasarsız {len(clean)}/{len(rr)}, hasarsız+pist içi {len(clean_on)}/{len(rr)}, "
                f"tur ort {sum(laps) / max(len(laps), 1):.3f} min {min(laps) if laps else None} max {max(laps) if laps else None}, "
                f"zaman aşımı ort {sum(r['to'] for r in rr) / max(len(rr), 1):.1f}")
        print(line, flush=True)
        lines.append(line)
        with open(out, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    print("özet:", out)


if __name__ == "__main__":
    main()
