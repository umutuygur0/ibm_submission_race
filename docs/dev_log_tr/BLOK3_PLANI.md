# Blok 3 — 2026-10-12 00:32 → 08:00: daha büyük ağ (512 × 512 × 512)

Hedef: ağ kapasitesini artırmak (256 × 256 → 512 × 512 × 512, aktör ~78 bin → 550 bin parametre).
**Kullanılabilir model kuralı (kullanıcı):** 30/30 hasarsız; kullanılabilirler arasında süre belirler.
Şampiyon: submission/torcs_jm_par_rl_v10d_dist_r02.py (73,098 s det, 20/20). Yedeği: racelab/YEDEK_SAMPIYON_v10d_2026-10-12/.

| Saat | İş |
|---|---|
| 00:32–00:40 | `rl/widen_net.py` (yeni): v4d → 44 giriş + 512×512×512, davranış korunarak (yeni nöron girişleri rastgele, çıkışları sıfır; 3. katman birim matris) |
| 00:40–06:10 | **w512a** eğitimi: v10 tarifi (v4, pos-obs, c-speed 0,25, tcs 5, safe-pos 0,9), tohum 91, idx 1–4, 5,5 saat |
| 06:10–06:40 | Seçim (select_models) + yardımcı testleri (r02, a35m20r02, a3m50, a5m10r02) |
| 06:40–07:40 | En iyi aday(lar)a ×30 dayanıklılık testi; teslim dosyası (çok katmanlı yeni oluşturucu, yeni adla) |
| 07:40–08:00 | BLOK3_RAPORU.md |

Hız ölçümü (GPU, sadece güncelleme): 256×256 991/sn, 512×512 514, 256×3 681, **512×3 312** → aynı öğrenme ~3,2 kat uzun.
Tek eğitim seçildi (GPU'nun tamamı bu ağa); 60 dk'da hâlâ tur bitiremiyorsa başka tohumla yeniden başlatılır.
Not: teslim dosyası numpy yoksa saf Python'a düşüyor; 512³ ağ saf Python'da adım başına ~10–20 ms olabilir → teslimde numpy şart, kontrol edilecek.

## Günlük
- 00:33 widen_net doğrulaması: aktör çıktı farkı max 4,4e-5, critic 3,9e-3 (float yuvarlaması). Pistte orijinal v4d ile aynı
  davranış (ikisi de tcs 5'te 585 m'de duruyor — v4d'nin bilinen başlangıç hali; v10 tarifi zaten buradan eğitiyor).
- 00:37 w512a başladı (5,5 sa → ~06:07). İlk dakika: 289 güncelleme/sn (beklenen ~300).
- 01:06 w512a 32 dk: **en iyi temiz tur 75,24** (yardımcısız; 10. dakikada, v348 — 256'lık eğitimler 30. dk'da ~76–77). lap20 %35, 279 upd/sn. Umut verici.
- 01:15 `submission/build_submission_v2.py` (yeni; build_submission.py'nin kopyası, her katman sayısını destekler, var olan dosyanın
  üzerine yazmaz). Regresyon: v10d + r02 → **73,098 s** (eskisiyle birebir). 512³ çıkarım süresi: numpy ile <0,1 ms/adım,
  **saf Python 17 ms/adım** (10 ms bütçeyi aşar) → 512³ teslim dosyası numpy'siz ortamda çalışmaz; teslim ortamında numpy olmalı.
- 01:32 w512a 58 dk: en iyi hâlâ 75,24, lap20 **%85** (sağlıklı). Ara test v00348 + yardımcı: det 74,20–74,66 hasarsız ama ×8'de 0/8 (erken aday, kırılgan).
- 02:02 w512a 87 dk: en iyi 75,24 (değişmedi), lap20 %15 (dalgalanma; 58. dk'da %85). İzleniyor.
- 02:32 w512a 118 dk (v3884): en iyi hâlâ 75,24 (v348). Aday turlarının 500 sürümlük ortalamaları (sürüm ∝ güncelleme sayısı):
  w512a 77,3 → 77,5 → 77,8 → 77,5 → 77,1 → 77,4 → 77,2 → 78,2; v10d (256²) aynı sürümlerde 78,9 → … → 77,0 (v2500) → 76,6 → 76,2 → **75,9 (v4000)**.
  → Büyük ağ başta daha iyi, ama v3000'e kadar v10d'nin yaptığı iyileşmeyi henüz göstermedi. v10d'nin asıl kazancı v3000–4500 arasındaydı;
  w512a oraya ~40 dk'da varır. 03:30'a kadar iyileşme yoksa: öğrenme oranını düşürerek (lr 1e-4) en iyi noktadan ikinci deneme düşünülecek.
- 03:02 w512a 147 dk (v~4900): lap20 **%95**; 250 sürümlük aday ortalamaları 78,2 → 78,3 → 78,0 → 77,4 → 77,9 → **76,8** (yeniden iyileşme başladı). En iyi tek tur hâlâ 75,24. Devam.
- 03:32–03:42 w512a 177 dk: aday ortalamaları v4750–5750 arası 76,3–77,1 (plato), lap20 %40–95 dalgalı; en iyi 75,24.
  Ara testler: w512a adayları deterministikte eğitimdeki turu birebir veriyor (v05548 75,666; v00348 75,238; v00348 + a3m50 74,61),
  ama 10 ms ×8 testlerinde hep 0/8. **Kontrol:** şampiyon v10d + r02 aynı anda 1/8 (normalde 20/20) → bu gece 512³ eğitiminin
  yükü altında 10 ms testleri geçersiz (sistem yükü, model değil). Uygulanan ayar 18650 yerine eğitimdeki 18000 vites
  de denendi; fark yok. → Dayanıklılık (×30) testleri eğitim bittikten sonra, sistem boşken yapılacak.
- 04:05 w512a 211 dk (v~7000): aday ortalamaları yavaşça iniyor (76,6 → 76,1), en iyiye yakın adaylar sıklaştı (75,24–75,40). lap20 %50. Devam (~06:07 bitiş).
- 04:35 w512a 241 dk: **yeni en iyi 75,07** (v7804, yardımcısız). Aday ortalaması v7500'de 75,85 (en iyi kova; v10d'nin en iyisi 75,88 idi). lap20 %70.
- 05:05 w512a 270 dk: **yeni en iyi 75,01** (v8916); v8750 kovası ort 75,75 (şimdiye kadarki en iyi, v10d'nin en iyisi 75,88). lap20 %60.
- 05:35 w512a 301 dk: en iyi 75,01 (değişmedi), kova ortalamaları 76,0–76,5 (plato), lap20 %95. 06:07'de biter; final_w512.sh otomatik seçim + yardımcı testlerini başlatacak (arka planda bekliyor).
- 06:04 w512a eğitimi bitti (5,5 sa, ~5,5 M güncelleme ≈ 280/sn, 880+ aday). En iyi yardımcısız 75,01. Seçim başlıyor.
- 06:13 Seçim (8 paralel, 10 ms ×10): w512a adayları sadece %10–20 hasarsız (v10d'nin seçiminde 10/10 vardı). Yardımcıyla ×8: en iyi v08171 a35m20r02/a3m50 5/8.
- 06:16 **Şampiyon ×30 (sistem boş, 10 ms, teslim dosyası): 30/30 hasarsız**, ort 74,32, en iyi 73,10 → 30/30 kuralını geçti.
- 06:20 w512a v08171 + a3m50 teslim dosyası (`submission/torcs_jm_par_rl_w512a_v08171_a3m50.py`, 2,9 MB, numpy gerekir):
  det 74,29 (0 hasar, 1 kez pist dışı), ×30 **30/30 hasarsız**, ort 75,70 → kullanılabilir ama şampiyondan 1,2 s yavaş.
  (Not: quick_exp/select ×8 testleri teslim dosyası yolundan belirgin sert çıkıyor; kesin karar ×30 run_benchmark ile.)
- 06:24 w512a ilk 6 aday × 40 yardımcı ayarı (240 det koşu): en iyi v09888 + 3 g / 20 m / 0,2 s **73,93** → şampiyon (73,10) geçilmedi.
- 06:30 BLOK3_RAPORU.md yazıldı. Çalışan süreç yok.
