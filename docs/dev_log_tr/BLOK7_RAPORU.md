# Blok 7 raporu — 2026-10-14 23:21 → 2026-10-15 04:10: pist sınırı serbest + uzun hibrit RL

Günlük: [BLOK7_PLANI.md](BLOK7_PLANI.md). Eski dosyalara dokunulmadı.

## Kural kontrolü
Toplantı kayıtları ve öğrenci paketi: ölçüt "temiz tur" = **hasarı sıfır, tamamlanmış, zamanlanmış tur**, sonra tur süresi
("tek bir hasar bile varsa başarısız"). **Pist dışına çıkma ile ilgili kural yok.** Bu gece |trackPos| kısıtı kaldırıldı.

## Sonuç
| Teslim dosyası | Det (= GUI) | 10 ms ×30 hasarsız | 10 ms ort | Karar |
|---|---|---|---|---|
| **`torcs_jm_par_hybrid_res3a.py`** (gr3L + artık RL) | **64,878** | **30/30 + 30/30** | **65,13–65,16** | **önerilen (en hızlı geçerli)** |
| `torcs_jm_par_line_gr3L.py` | 64,964 | 30/30 (×3) | 65,55 | sağlam yedek |
| `torcs_jm_par_line_t4off.py` (pist sınırı serbest planlayıcı) | 64,722 | **26/30** ✗ | 65,46 | kural dışı |
| `torcs_jm_par_line_t3off.py` | 64,730 | **23/30** ✗ | 65,57 | kural dışı |
| Rakip (One Qubit Racing) | 65,562 | — | — | — |

## Bulgular
1. **Pist sınırını gevşetmek** deterministik turu 0,24 s hızlandırdı (64,964 → 64,722), ama çözümler gecikme/kaçırmaya hassas:
   gerçek 10 ms testinde 23–26/30. Virajlardaki asıl sınır duvarlar; kenarı kullanmak riski artırıyor.
2. **Uzun hibrit RL (3 tohum + 2 varyant, ~4 saat):** res5 temel sürücüyü geçti (64,936–64,962) ama res3a'yı (64,878) geçemedi.
   Pist sınırı serbest temelde (res4a) RL çöktü; res3'ün devamı (res4b) ilerlemedi. RL tohuma çok bağlı (5 denemeden 2'si başarılı).
3. res3a'nın RL ağı başka temel sürücüye (t4off) taşınamıyor (hasar) — RL düzeltmeleri temel sürücüye özgü.

## Öneri
Teslim için **`torcs_jm_par_hybrid_res3a.py` (64,878 s, 60/60 hasarsız)**; yedek `torcs_jm_par_line_gr3L.py` (64,964 s).
Kalan zaman (son tarih 16 Ekim) teslim hazırlığına: GUI testi + video, temiz ayrı repo, basit livery.
GUI testi:
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_hybrid_res3a.py --mode gui --label "hybrid_res3a@GUI"
```
