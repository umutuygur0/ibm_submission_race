# Blok 8 raporu — 2026-10-15 04:10 → 07:15: stabilite

Günlük: [BLOK8_PLANI.md](BLOK8_PLANI.md). Not: bilgisayar ~04:15–06:44 askıya alındı (keep_awake boşta uykuyu engeller,
kapak/uyku tuşunu engellemez); plan sıkıştırıldı.

## Sonuç
**Önerilen teslim: `racelab/submission/torcs_jm_par_hybrid_res3a.py` — 64,878 s.**

| Model | Det | run_benchmark 10 ms | Stres: kalıcı gecikme | Stres: kaçırma |
|---|---|---|---|---|
| **hibrit res3a** | **64,878** | **60/60** hasarsız (ort 65,13–65,16) | **25/25** (ort 65,15) | **25/25** (ort 64,88) |
| planlayıcı gr3L (yedek) | 64,964 | 90/90 | 30/30 | 30/30 |
| planlayıcı t4off | 64,722 | 26/30 ✗ | 16–24/30 | 17–26/30 |

## Ne denendi
1. **t4off'u sağlamlaştırma (t5rob):** gecikme + kaçırma turlarını zorunlu tutan hedefli arama → iyileşme bulunamadı.
2. **Güvenlik payı taraması:** viraj hızları ×0,995–0,98, fren ×0,95–0,90 → gecikmede hasarsızlık değişmedi (16–17/20),
   fren azaltmak kötüleştirdi. t4off'un gecikme hassasiyeti yapısal (kontrolcü o hızlarda sınırda).
3. Hibrit res3a'nın sağlamlığı uzatılmış testlerle doğrulandı.

## Sonraki adım
Teslim hazırlığı (son tarih 16 Ekim): GUI testi + video (varsayılan livery), temiz ayrı GitHub reposu, basit livery.
Daha fazla süre kazancı için en umut verici yol: res3 tarifiyle (gr3L + artık RL, 8 işçi, ≥ 2 sa) birkaç tohum daha.
