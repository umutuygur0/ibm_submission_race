# Blok 1 — 2026-10-10 00:40 → 06:20

Başlangıç şampiyonu: submission/torcs_jm_par_rl_v10d_dist.py → 73,582 s / 0 hasar (deterministik), 10 ms ×10 10/10.
Başarı ölçütü: deterministik < 73,58 s, 0 hasar, 10 ms ×10 10/10.

| Saat | İş |
|---|---|
| 00:40–05:05 | b1a (seed 76, idx 1-4) + b1b (seed 77, idx 5-8): v10 tarifi (--pos-obs, v4d genişletilerek, c_speed .25, TCS 5, safe .9), 4,4 saat |
| 00:42–01:30 | Yardımcı ayarı (v10d_v04188, idx 9): mesafe a∈{2,2.5,3,3.5,4} g × pay∈{20,35,50,70} m, sabit 160-200 m, kombinasyonlar; en iyi 5 → teslim dosyası ×10 (port 3001) |
| her saat | Kontrol: çöken eğitim → STOP + yeni tohum |
| 05:05–05:50 | Seçim (×10) + ilk 3 × yardımcı varyantları |
| 05:50–06:20 | Final ×20, gerekirse yeni teslim dosyası (yeni adla), BLOK1_RAPORU.md |

## Günlük
- 00:40 b1a + b1b başladı (4,4 saat → ~05:05).
- 00:40 Yardımcı taraması (v10d_v04188, deterministik, quick_exp): kısa pay daha iyi — 3,5g20m 72,968; 4,0g20m 73,148; 4,0g35m 73,23; 2,5g20m 73,238; 3,0g35m 73,30; mevcut 3,0g50m 73,444; sabit mesafe 73,76-74,04; yardımcısız 75,07.
- 00:53 Teslim dosyası olarak (eğitim yükü altında, ×10 10 ms): kontrol (mevcut 3g50m) 73,582 det / 10/10 ort 73,75.
  3,5g20m 73,032 / 10/10 ort 73,31 · **4,0g20m 72,940 / 10/10 ort 73,18** · 4,0g35m 73,164 / 10/10 · 2,5g20m 73,306 / 10/10 · 3,0g35m 73,306 / 10/10.
  → **Yeni teslim dosyası: submission/torcs_jm_par_rl_v10d_dist_a4m20.py (72,94 s, 0 hasar, 10/10).** Eski dosyalar değişmedi.
- 01:23 b1a 45 dk: eval 75,63 (v10d aynı aşamada 76,67), lap20 %90; b1b 76,44, lap20 %80. Sağlıklı.
- 01:53 b1a 76 dk: eval 75,63 (değişmedi), 347 aday, lap20 %80; b1b 76,44, 320 aday, lap20 %75. Sağlıklı.
- 02:24 b1a 106 dk: eval 75,59; b1b 76,39; ikisi de sağlıklı. Ara ölçüm (4g20m yardımcı, det): b1a/v01957 73,89; b1a/v03573 77,86; b1a/v02790 çarptı; b1b/v03713 90,4 → 4g20m v10d'ye özgü, finalde her aday için yardımcı ayarları ayrı taranacak.
- 02:55 b1a 137 dk: eval 75,13 (2 saatten sonra iyileşmeye devam), b1b 75,90; lap20 %80-85, sağlıklı → uzun eğitim faydalı görünüyor.
- 03:25 b1a 167 dk: eval **74,87** (hâlâ iyileşiyor), b1b 75,90; lap20 %85-90, sağlıklı.
- 03:55 b1a 198 dk: eval 74,87 (yeni adaylar 74,96/74,99), lap20 %100; b1b 75,48 (iyileşti), lap20 %70. Sağlıklı.
- 04:25 b1a 228 dk: lap20 %40'a düştü (bozulma başlangıcı), en iyi 74,87 adaylar kayıtlı; ~40 dk kaldığı için müdahale yok. b1b 75,48, lap20 %100.
- 05:02-05:18 Seçim + yardımcı testleri + final ×20 (ayrıntılar BLOK1_RAPORU.md). Şampiyon: güvenli v10d_dist (73,58, 20/20); en hızlı v10d_dist_a4m20 (72,94, 11-14/20, adım kaçmalarına hassas). Blok bitti; açık süreç yok.
