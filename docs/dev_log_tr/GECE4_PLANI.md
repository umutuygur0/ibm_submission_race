# Gece planı 4 — 2026-10-08 23:41 → 2026-10-09 09:00 (otonom)

Eski raporlar korunur. Sonuç: `SABAH_RAPORU_4.md`. Başlangıç:
- GUI'de doğrulanan en iyi: submission/torcs_jm_par_rl_s47_assist.py → 01:18:15
- En iyi teslim adayı: submission/torcs_jm_par_rl_s47_dist.py → 77,436 s (hızlı mod), 10/10
- Referans (TORCS botları, aynı araç): 74,2 s

## Plan
1. n4_s62 değerlendirmesi (yarıda kalmıştı).
2. **v10 — konum bilen agent:** gözleme pistteki konum (distFromStart) sin/cos harmonikleri (k=1,2,4,8,16,32 → en kısa dalga ~113 m) eklenir (32→44 boyut).
   Ağırlıklar v4d/final_full'dan **genişletilerek** alınır (yeni giriş sütunları sıfır → başlangıçta eski modelle aynı davranış).
3. 00:30–02:30 v10 iki tohum → seçim (yardımcısız + mesafe yardımcısı). Umutluysa v10 tohum hattı, değilse n4 hattına devam.
4. 08:15–09:00 final, yeni teslim dosyası (yeni adla), SABAH_RAPORU_4.md.

## Günlük
- 23:44 n4_s62 değerlendirildi: en iyi 9/10, ort ~83 → şampiyonu geçmiyor.
- v10 kodu eklendi (geriye uyumlu, model giriş boyutundan otomatik algılama). Eşdeğerlik testi: v4d 32 giriş 81,904 = 44'e genişletilmiş v4d 81,904; teslim dosyası (44) 81,736 = orijinal. (Skor tablosundaki TEST_sub44_v4d satırı bu testtir.)
- v10a (seed 70, idx 1-4) + v10b (seed 71, idx 5-8): --pos-obs, v6 tarifi, v4d/final_full genişletilerek, 2 saat.
- 00:15 **v10 büyük sıçrama:** v10a 30 dk'da eval 76,89 s YARDIMCISIZ (v6g yardımcısız 79,80), lap20 %100. Deterministik (idx 9): v00621 yardımcısız 76,99 / +mesafe 3g50m **75,238** / +sabit180 76,28; v01244 76,89 / 75,56 / 76,15. Botlara (74,2) fark ~1 s.
  night_pipeline'a --pos-obs ve --init eklendi. Plan: v10a/b ~01:45 bitince seçim+sağlamlık, sonra v10 tohum hattı sabaha kadar.
- 01:45 v10a/b bitti. Seçim ×10 yardımcısız: v10b/best 10/10 ort 76,03; v10a/best 10/10 ort 77,36.
- 01:53 Yardımcı değerlendirmesi (det + ×8 kuyruk boşaltmasız): **v10b/best + mesafe 3g50m det 73,76** (5/8), +2g30m 74,22, +a180 74,48, yardımcısız 75,17 (8/8); v10b/v04324 + 3g50m 74,54 (8/8).
- 01:55 **TESLİM DOSYALARI (yeni):** torcs_jm_par_rl_v10b_best_dist.py → **73,728 s / 0 hasar, 10 ms ×10 10/10 (74,4-75,7)** → BOTLARI (74,2) GEÇTİ.
  torcs_jm_par_rl_v10b_v04324_dist.py 74,416 / 10/10; torcs_jm_par_rl_v10b_best_plain.py 75,156 / 10/10.
- 01:57 v10 tohum hattı (n5): --pos-obs, init v10b/final_full, 60 dk/tohum, seed 80+, son başlangıç 07:10.
- 03:14 n5_s80 (v10b'den devam): en iyi det 74,05 (best+2g30m), ×8 sağlamlık düşük (2-7/8). Şampiyon (73,73) geçilmedi. n5_s81 eğitimde.
- 04:12 n5_s81 en iyi det 74,23 → 2 döngüde rekor yok, v10b'den devam bırakıldı. n5 durduruldu (n5_s82 STOP ile kaydedildi, değerlendirilmedi). Yeni: v10c+v10d = v10a/b tarifi (v4d genişletilerek), seed 72/73, 2 saat paralel.
- 06:16 v10c/v10d bitti. Seçim: v10c en iyi 10/10 ort 77,83; v10d/v04188 9/10 ort 76,26. Yardımcıyla (det + ×8): **v10d/v04188 + 3g50m det 73,444, 8/8** (ort 74,24); +2g30m 73,51 8/8; v10d/best +3g50m 73,65 5/8.
- 06:18 **YENİ ŞAMPİYON:** submission/torcs_jm_par_rl_v10d_dist.py → **73,582 s / 0 hasar / max 270,5**, 10 ms ×10 **10/10** (73,6-74,9). (v10b_best_dist: 73,728, 74,4-75,7)
- 06:20 Son tur: v10e/v10f (seed 74/75, aynı tarif, 1,9 saat → ~08:15), sonra final + SABAH_RAPORU_4.
- 08:20 v10e/v10f final: seçim en iyi 10/10 ort 76,66 (v10f/v02434); yardımcıyla en iyi det 74,51 → şampiyon v10d (73,44) değişmedi. Gece tamamlandı; açık eğitim yok.
