# Blok 2 — 2026-10-10 20:05 → 2026-10-11 08:00

Başlangıç: güvenli şampiyon submission/torcs_jm_par_rl_v10d_dist.py (73,58 s, 20/20); en hızlı v10d_dist_a4m20 (72,94 s, adım kaçmalarına hassas).

| Saat | İş |
|---|---|
| 20:05–00:05 | b2a: standart v10 uzun eğitim (seed 78, idx 1-4, 4 saat) |
| 20:05–20:45 | D1 kodu: konum harmonikleri k=64'e kadar (en kısa dalga ~56 m), seçenek olarak; regresyon testi |
| 20:45–00:45 | b2b: D1 uzun eğitim (seed 79, idx 5-8, 4 saat) |
| 20:45–21:45 | Gecikmeye dayanıklı yardımcı (tepki payı = hız × t), teslim dosyası ×20 (port 3001, quick_exp idx 9) |
| 00:05/00:45 | Seçim + yardımcı varyantları |
| 00:50–04:50 | 2. tur: en iyi tarif, 2 uzun eğitim |
| 04:50–05:40 | Seçim |
| 05:40–07:20 | Yeni modeller için yardımcı ayarı / kısa ek tohumlar |
| 07:20–08:00 | Final ×20, yeni teslim dosyası (yeni adla), BLOK2_RAPORU.md |

## Günlük
- 20:02 b2a başladı (standart v10, seed 78, 4 saat → ~00:02).
- 20:03 D1 kodu eklendi (--pos-obs2, 46 giriş, k=64). Regresyon: v10d 75,068 (aynı). 46'ya genişletilmiş v4d 81,716 (32'lik 81,904): toplu hesapta çıktılar birebir aynı, tek örnekte en fazla 7,6e-6 yuvarlama farkı (çekirdek seçimi) → kaotik yarışta küçük süre farkı; kod hatası değil.
- 20:04 b2b başladı (D1, seed 79, idx 5-8, 4 saat → ~00:04). (Önceki satırlardaki saatler tahmindi, düzeltildi.)
- 20:19 **Gecikmeye dayanıklı yardımcı (tepki payı = hız×t):** ×20 eğitim yükü altında: 4g20m+0,2s **20/20 ort 73,62** (10+ timeout koşuların 6/6'sı temiz); +0,3s 20/20 73,80; +0,5s 20/20 73,84; 4g0m+0,4s 20/20 73,73; kontroller: 4g20m (tepkisiz) 13/20, 3g50m 20/20 ort 74,11.
  → Yeni teslim dosyası: submission/torcs_jm_par_rl_v10d_dist_r02.py
- 20:45 b2a 44 dk: eval 75,94 (b1a aynı aşamada 75,63), lap20 %55; b2b (D1) 42 dk: 76,86, lap20 %85. Sağlıklı.
- 21:16 b2a 74 dk: 75,94 (değişmedi), lap20 %40; b2b (D1) 72 dk: 76,86, lap20 %85.
- 21:46 b2a 104 dk: eval 75,30 (toparlandı, lap20 %85); b2b (D1) 102 dk: 76,78, lap20 %95.
- 22:16 b2a 134 dk: 75,11; b2b (D1) 133 dk: 76,78 (30 dk değişmedi). D1 şimdiye kadar standart v10'un gerisinde.
- 22:46 plato: b2a 75,11, b2b (D1) 76,78. Ara ölçüm (4g20m+0,2s): b2a adayları 73,83-74,58 (v10d aynı yardımcıyla 73,07). Karar (2. tur): D1 bırakılıyor; 2 standart v10 tohumu + b1a/b2a adayları için model başına geniş yardımcı taraması (idx 9).
- 23:13 Model başına yardımcı taraması (6 model × 36 ayar, 169/216 temiz): en iyi b2a/v05219 + 3,5g20m+0,2s **73,73**; b1a/v10554 73,87 → v10d+r02 (73,07) geçilmedi. v10d yardımcıyla nadir uyumlu bir model.
  2. tur kararı: 4 saatlik 2 eğitim yerine 2 saatlik eğitimlerden 2 tur (4 tohum, v10d'nin çıktığı tarif/süre): 00:05-02:05 (seed 80/81), 02:10-04:10 (seed 82/83); her tur sonrası seçim + yardımcı (r02 ve tarama).
- 23:18 b2a/b2b plato (75,11 / 76,27, lap20 %45-55) → STOP (adaylar kayıtlı). Yeni takvim: 2 saatlik tohum turları — T1 23:20-01:20 (80/81), T2 01:25-03:25 (82/83), T3 03:30-05:30 (84/85), T4 05:35-07:15 (86/87), final 07:20-08:00.
- 23:48 T1 30 dk: b2s80 76,81 (lap20 %90), b2s81 **75,98** (lap20 %100; v10d aynı aşamada ~76,7). Sağlıklı.
- 00:18 T1 60 dk: b2s80 75,67, b2s81 75,98; lap20 %65.
- 00:48 T1 90 dk: b2s80 75,67 (lap20 %90), b2s81 **75,28** (lap20 %100).
- 01:30 T1 bitti (özet betiğinde değişken çakışması → sonuçlar elle eklendi; düzeltilmiş kopya `rl/seed_round2.sh`).
  Seçim: b2s80 en iyi 10/10 ort 76,44; b2s81 10/10 ort 76,68. Yardımcıyla en iyi: b2s80/best a3m50 73,39 (×8: 3/8),
  b2s81 v03646 r02 73,52 (×8: 1/8). **Şampiyonu (v10d r02 73,07, 20/20) geçen yok.** Ayrıntı: `rl/runs/round_80_81.txt`.
- 01:31 T2 başladı: seed_round2.sh 82 83 2.0 (~03:45 bitiş). T3 ~03:50, T4 kısaltılmış (~1,5 sa) ~06:00.
- 02:02 T2 31 dk: b2s82 en iyi 76,40 (lap20 %85), b2s83 76,71 (lap20 %95). Normal.
- 02:32 T2 61 dk: b2s82 en iyi 75,46 (lap20 %80), b2s83 76,46 (lap20 %70). Normal.
- 03:02 T2 91 dk: b2s82 en iyi 75,14 (lap20 %90), b2s83 75,64 (lap20 %95). Normal.
- 03:32 T2 eğitim bitti: b2s82 en iyi 75,14, b2s83 75,56 (son dakikalarda lap20 %55-60 — geç dönem bozulması). Seçim+yardımcı değerlendirmesi çalışıyor.
- 03:41 T2 bitti. Yardımcıyla en iyi: b2s82 v03024 r02 73,58 (×8: 6/8), a35m20r02 73,86 (8/8), a5m10r02 73,45 (3/8);
  b2s83 en iyi 74,79 (8/8). **Şampiyon (73,07) geçilmedi.** Ayrıntı: `rl/runs/round_82_83.txt`.
- 03:42 T3 başladı: seed_round2.sh 84 85 2.0 (~05:55 bitiş). T4 ~06:00, 1,3 sa (~07:35 bitiş), sonra rapor.
- 04:13 T3 31 dk: b2s84 zayıf (lap20 %0, en iyi 82,42 — izleniyor), b2s85 76,91 (lap20 %90).
- 04:43 T3 61 dk: b2s84 60 dk boyunca hiç tur bitiremedi (lap20 %0) → STOP ile güvenli durduruldu (2 aday kayıtlı).
  Boşalan idx 1–4'te ek kısa eğitim b2s90 (tohum 90, 0,9 sa, ~05:38 bitiş; T3 değerlendirmesinden önce biter). b2s85 75,85 (lap20 %70).
- 05:14 T3 92 dk: b2s85 75,85 (lap20 %70). b2s90 31 dk: 76,80 (lap20 %85) — iyi başlangıç.
- 05:37 b2s90 bitti (54 dk): en iyi 76,80. Değerlendirmesi T4 sırasında idx 9'da yapılacak.
- 05:44 T3 eğitim bitti (b2s85 en iyi 75,85). Seçim/değerlendirme çalışıyor.
- 05:51 T3 bitti. b2s84 kullanılamaz (yardımcıyla bitiremiyor); b2s85 en iyi best.pt a3m50 74,18 (×8: 2/8). **Şampiyon geçilmedi.**
  Ayrıntı: `rl/runs/round_84_85.txt`.
- 05:53 T4 başladı: seed_round2.sh 86 87 1.5 (~07:35 bitiş). Paralel: b2s90 seçim + yardımcı testi idx 9'da sırayla → `rl/runs/round_90.txt`.
- 06:23 T4 31 dk: b2s86 76,94 (lap20 %65), b2s87 **75,79** (lap20 %90). b2s90 değerlendirmesi: zayıf (seçimde en iyi 5/10 hasarsız; yardımcıyla en iyi 74,52 det, ×8 1/8).
- 06:53 T4 61 dk: b2s86 76,30 (lap20 %65), b2s87 75,79 (lap20 %75). Rapor taslağı hazırlanıyor.
- 07:24 T4 eğitim bitti: b2s86 76,06, b2s87 75,79. Seçim/değerlendirme çalışıyor (~07:35).
- 07:33 T4 bitti. En iyi: b2s87 v02064 a5m10r02 73,68 (×8: 8/8, ort 76,02); b2s86 74,27. **Şampiyon geçilmedi.** Ayrıntı: `rl/runs/round_86_87.txt`.
- 07:35 Blok 2 kapanış: öneri değişmedi → submission/torcs_jm_par_rl_v10d_dist_r02.py. Rapor: BLOK2_RAPORU.md. Eğitim çalışmıyor.
