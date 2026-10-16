"""Gece hattı: çok tohumlu kısa eğitim + seçim + sonradan yardımcı (post-hoc assist) değerlendirmesi.

    python racelab/rl/night_pipeline.py --prefix n3 --start-seed 40 --until 07:30

Her döngü:
  1) v6 tarifiyle eğitim (TCS 5, safe_pos 0.9, c_speed 0.25, v4d/final_full'dan) — --minutes, 8 worker
  2) select_models: en iyi --top aday × 10 (TCS 5, yardımcısız)
  3) seçimin en iyi 3'ü için yardımcı varyantları (deterministik + ×8 10 ms bekleme) — quick_exp.py
  4) sonuçlar runs/<prefix>_pipeline.csv'ye eklenir (üzerine yazılmaz)
--until saatinden sonra yeni döngü başlamaz. Durdurmak için runs/<prefix>_STOP dosyası oluştur.
"""
import argparse
import csv
import datetime as dt
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PY = sys.executable
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")
# (etiket, quick_exp argümanları) — sonradan uygulanan yardımcı varyantları
ASSISTS = [("a0_u18000", ["--upshift", "18000"]),
           ("a180_u18650", ["--assist-front", "180", "--upshift", "18650"]),
           ("d3.0m50_u18650", ["--assist-dec", "3.0", "--assist-margin", "50", "--upshift", "18650"]),
           ("d2.0m30_u18650", ["--assist-dec", "2.0", "--assist-margin", "30", "--upshift", "18650"])]


def sh(args, log):
    with open(log, "a", encoding="utf-8") as f:
        f.write(f"\n$ {' '.join(args)}\n")
        f.flush()
        return subprocess.run(args, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, env=ENV).returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", default="n3")
    ap.add_argument("--start-seed", type=int, default=40)
    ap.add_argument("--minutes", type=float, default=45)
    ap.add_argument("--until", default="07:30", help="bu saatten sonra yeni döngü başlamaz (HH:MM)")
    ap.add_argument("--top", type=int, default=12)
    ap.add_argument("--pos-obs", action="store_true", help="v10: konum bilen gözlem")
    ap.add_argument("--init", default="racelab/rl/runs/v4d/final_full.pt")
    args = ap.parse_args()

    runs = os.path.join(HERE, "runs")
    stop = os.path.join(runs, f"{args.prefix}_STOP")
    log = os.path.join(runs, f"{args.prefix}_pipeline.log")
    out = os.path.join(runs, f"{args.prefix}_pipeline.csv")
    hh, mm = map(int, args.until.split(":"))
    start = dt.datetime.now()
    limit = start.replace(hour=hh, minute=mm, second=0, microsecond=0)
    if limit <= start:  # saat geçmişse bir sonraki günün o saati
        limit += dt.timedelta(days=1)
    print(f"son döngü başlangıcı: {limit:%Y-%m-%d %H:%M}", flush=True)
    seed = args.start_seed
    while True:
        now = dt.datetime.now()
        if now >= limit or os.path.exists(stop):
            break
        name = f"{args.prefix}_s{seed}"
        # 1) eğitim
        sh([PY, "racelab/rl/train_sac_v2.py", "--env", "v4", "--name", name, "--c-speed", "0.25", "--tcs-th", "5.0",
            "--safe-pos", "0.9", "--seed", str(seed), "--workers", "8", "--idx-offset", "1",
            "--hours", str(args.minutes / 60), "--init", args.init] + (["--pos-obs"] if args.pos_obs else []),
           os.path.join(runs, f"{name}.log"))
        # 2) seçim
        sel = os.path.join(runs, name, "selection.csv")
        sh([PY, "racelab/rl/select_models.py", "--run", name, "--env", "v4", "--tcs-th", "5.0", "--top", str(args.top),
            "--repeat", "10", "--procs", "4", "--idx-from", "1"], log)
        best = []
        if os.path.exists(sel):
            with open(sel, encoding="utf-8") as f:
                best = list(csv.DictReader(f))[:3]
        # 3) yardımcı varyantları (paralel, farklı portlarda)
        jobs, tags = [], []
        for b in best:
            model = os.path.join(ROOT, b["model"])
            for label, extra in ASSISTS:
                tag = f"{name}|{os.path.basename(model)}|{label}"
                jobs.append([PY, "racelab/rl/quick_exp.py", "--model", model, "--gui", "8", "--tag", tag] + extra)
                tags.append((tag, b))
        for w in range(0, len(jobs), 9):  # scr_server idx 1..9 → en fazla 9 paralel
            procs = [subprocess.Popen(j + ["--idx", str(k + 1)], cwd=ROOT, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL, env=ENV) for k, j in enumerate(jobs[w:w + 9])]
            for p in procs:
                p.wait()
        # 4) özet (quick_exp.csv'den bu döngünün satırları)
        qe = os.path.join(runs, "quick_exp_v4.csv")
        rows = []
        if os.path.exists(qe):
            with open(qe, encoding="utf-8") as f:
                rows = [r for r in csv.DictReader(f) if r["tag"].startswith(name + "|")]
        new = not os.path.exists(out)
        with open(out, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["finished_at", "run", "model", "assist", "sel_clean_rate", "sel_mean_lap",
                            "det_lap", "det_damage", "gui_clean_of_8", "gui_mean_lap"])
            sel_by_model = {os.path.basename(b["model"]): b for _, b in tags}
            for r in rows:
                _, mdl, assist = r["tag"].split("|")
                b = sel_by_model.get(mdl, {})
                w.writerow([dt.datetime.now().strftime("%H:%M"), name, mdl, assist, b.get("clean_finish_rate"),
                            b.get("mean_clean_lap") or b.get("mean_lap"), r["det_lap"], r["det_damage"],
                            r["gui_clean"], r["gui_mean_lap"]])
        seed += 1


if __name__ == "__main__":
    main()
