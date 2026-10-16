#!/usr/bin/env bash
# Blok 3: w512a bitince seçim + yardımcı testleri (sistem boşken). Sonuç: racelab/rl/runs/round_w512a.txt
set -u
cd "$(dirname "$0")/../.."
export PYTHONIOENCODING=utf-8
until grep -q '^bitti' racelab/rl/runs/w512a.log; do sleep 30; done
sleep 20
python racelab/rl/select_models.py --run w512a --env v4 --tcs-th 5.0 --top 16 --repeat 10 --procs 8 --idx-from 1 > racelab/rl/runs/w512a_select.log 2>&1
jobs=()
for m in $(tail -n +2 racelab/rl/runs/w512a/selection.csv | head -n 4 | cut -d, -f1); do
  for cfg in "r02|--assist-dec 4.0 --assist-margin 20 --assist-react 0.2" "a35m20r02|--assist-dec 3.5 --assist-margin 20 --assist-react 0.2" \
             "a3m50|--assist-dec 3.0 --assist-margin 50" "a5m10r02|--assist-dec 5.0 --assist-margin 10 --assist-react 0.2"; do
    jobs+=("$m|$cfg")
  done
done
total=${#jobs[@]}
for ((w=0; w<total; w+=9)); do
  for ((k=0; k<9 && w+k<total; k++)); do
    IFS='|' read m lab args <<< "${jobs[$((w+k))]}"
    python racelab/rl/quick_exp.py --model "$m" --idx $((k+1)) --gui 8 $args --tcs 5 --upshift 18650 --tag "w512final|$m|$lab" > /tmp/w512final_$((w+k)).txt 2>&1 &
  done
  wait
done
{
  echo "w512a değerlendirme bitti: $(date '+%H:%M')"
  echo "== seçim (ilk 8):"; tail -n +2 racelab/rl/runs/w512a/selection.csv | head -n 8 | cut -d, -f1-6
  echo "== yardımcıyla (det, ×8):"
  for ((i=0; i<total; i++)); do
    grep -o "'tag'[^,]*\|'det_lap'[^,]*\|'det_damage'[^,]*\|'gui_clean'[^,]*\|'gui_mean_lap'[^}]*" /tmp/w512final_$i.txt | tr '\n' ' '; echo
  done
} > racelab/rl/runs/round_w512a.txt
