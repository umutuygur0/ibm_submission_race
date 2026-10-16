# Blok 6 raporu — 2026-10-14 11:27 → 18:20: küçük düzeltmeler + artık RL (hibrit)

Günlük: [BLOK6_PLANI.md](BLOK6_PLANI.md). Eski dosyalara dokunulmadı; yeni dosyalar: `planner/targeted.py`, `planner/lag_stress.py`,
`rl/residual_env.py`, `rl/train_residual.py`, `submission/torcs_jm_par_line_t1.py`, `submission/torcs_jm_par_line_t2.py`.

## Sonuç (18:20, son)
| Teslim dosyası | Det (= GUI) | 10 ms ×30 hasarsız | 10 ms ort | Stres: gecikme | Pist içi | Karar |
|---|---|---|---|---|---|---|
| **`torcs_jm_par_hybrid_res3a.py`** (gr3L + artık RL) | **64,878** | **30/30** | **65,16** | 10/10, ort 65,15 | ✗ (küçük taşmalar, |pos| ≤ 1,035) | **en hızlı geçerli** |
| **`torcs_jm_par_line_gr3L.py`** | 64,964 | 30/30 (×3) | 65,55 | 30/30, ort 65,58 | ✓ (det) | en dengeli |
| `torcs_jm_par_line_t2.py` | 64,840 | 30/30 | — | 12/30 | ✗ | hızlı ama gecikmeye hassas |
| `torcs_jm_par_line_gr1a.py` | 65,498 | 30/30 | 65,78 | 30/30 | ✓✓ | en sağlam yedek |

## Bulgular
1. **Son düzlükte hız kesmiyoruz:** bitişe tam gaz, rakiple aynı hızlar (3596 m: 271/272 km/h), çizgiye 0,66 s önde.
2. **Küçük düzeltmeler (hedefli arama):** 0,09–0,12 s kazanç (64,964 → 64,840), run_benchmark kuralını geçiyor; ama zorla gecikme
   altında gr3L kadar sağlam değil.
3. **Artık RL (gr3L + SAC küçük düzeltme; direksiyon ±0,05, pedal ±0,25):** ilk denemeler (res1, res2) temel sürücüyü geçemedi;
   res2 bilgisayar uykuya geçince durmuştu (`tools/keep_awake.py` ile çözüldü), bilgisayar şarjda değilken eğitim 10 kat yavaşladı.
   res3 (2,2 sa): **64,878** (temel 64,956). RL özellikle gecikme altında güçlü: gecikmeli tur 65,15 vs gr3L 65,58.
   Eksisi: pist kenarını ~20 cm aşan küçük taşmalar (hasar yok). Kenar cezası 10 kat artırılınca (res3b) süre geriledi (65,14).

## Öneri
- **Hasarsız en hızlı:** hibrit `torcs_jm_par_hybrid_res3a.py` (64,878 s; gerçekçi koşulda gr3L'den ~0,4 s hızlı).
- **Pist içinde kalması önemliyse:** `torcs_jm_par_line_gr3L.py` (64,964 s).
- GUI'de ikisini de denemek, videoyu daha iyi görüneni ile çekmek mantıklı. Hibrit dosya numpy ile en hızlı (saf Python 3,7 ms/adım da yeterli).
