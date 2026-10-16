#!/usr/bin/env bash
# Blok 2: bir tohum turu = iki paralel 2 saatlik v10 eğitimi + seçim + yardımcı değerlendirmesi.
#   bash racelab/rl/seed_round.sh <seedA> <seedB> <saat>
# Sonuç: racelab/rl/runs/round_<seedA>_<seedB>.txt
set -u
cd "$(dirname "$0")/../.."
A=$1; B=$2; H=${3:-2.0}
export PYTHONIOENCODING=utf-8
python racelab/rl/train_sac_v2.py --env v4 --name b2s$A --pos-obs --c-speed 0.25 --tcs-th 5.0 --safe-pos 0.9 \
  --seed $A --workers 4 --idx-offset 1 --hours $H --init racelab/rl/runs/v4d/final_full.pt > racelab/rl/runs/b2s$A.log 2>&1 &
sleep 3
python racelab/rl/train_sac_v2.py --env v4 --name b2s$B --pos-obs --c-speed 0.25 --tcs-th 5.0 --safe-pos 0.9 \
  --seed $B --workers 4 --idx-offset 5 --hours $H --init racelab/rl/runs/v4d/final_full.pt > racelab/rl/runs/b2s$B.log 2>&1 &
wait
OUT=racelab/rl/runs/round_${A}_${B}.txt
for run in b2s$A b2s$B; do
  python racelab/rl/select_models.py --run $run --env v4 --tcs-th 5.0 --top 12 --repeat 10 --procs 4 --idx-from 1 > /dev/null 2>&1
done
# her eğitimin seçimdeki ilk 3 adayı × yardımcı ayarları (deterministik + 10 ms ×8)
jobs=()
for run in b2s$A b2s$B; do
  for m in $(tail -n +2 racelab/rl/runs/$run/selection.csv | head -n 3 | cut -d, -f1); do
    for cfg in "r02|--assist-dec 4.0 --assist-margin 20 --assist-react 0.2" "a35m20r02|--assist-dec 3.5 --assist-margin 20 --assist-react 0.2" \
               "a3m50|--assist-dec 3.0 --assist-margin 50" "a5m10r02|--assist-dec 5.0 --assist-margin 10 --assist-react 0.2"; do
      jobs+=("$m|$cfg")
    done
  done
done
n=${#jobs[@]}
for ((w=0; w<n; w+=9)); do
  for ((k=0; k<9 && w+k<n; k++)); do
    IFS='|' read m lab args <<< "${jobs[$((w+k))]}"
    python racelab/rl/quick_exp.py --model "$m" --idx $((k+1)) --gui 8 $args --upshift 18650 --tag "round|$m|$lab" > /tmp/round2_${A}_$((w+k)).txt 2>&1 &
  done
  wait
done
{
  echo "tur $A/$B bitti: $(date '+%H:%M')"
  for run in b2s$A b2s$B; do echo "== $run seçim (ilk 3):"; tail -n +2 racelab/rl/runs/$run/selection.csv | head -n 3 | cut -d, -f1-6; done
  echo "== yardımcıyla (det, ×8):"
  for ((i=0; i<n; i++)); do
    grep -o "'tag'[^,]*\|'det_lap'[^,]*\|'det_damage'[^,]*\|'gui_clean'[^,]*\|'gui_mean_lap'[^}]*" /tmp/round2_${A}_$i.txt | tr '\n' ' '; echo
  done
} > $OUT
