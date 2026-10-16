# Blok 4 — 2026-10-12 23:30 → 2026-10-13 08:00: yarış çizgisi + hız planı (kural tabanlı)

**Neden:** Rakip takım (One Qubit Racing) Corkscrew'da **1:05.56**, hasarsız, |trackPos| ≤ 1 bildirdi. Bizim en iyimiz 73,10 s.
Kendi açıklamalarına göre (web sitesi + README; kodları kopyalanmadı, sadece yöntem fikri): pistin geometrisinden önceden
hesaplanmış yarış çizgisi + her noktada izin verilen hız, canlı sensörlerle düzeltme. Son ~8 saniyeyi bu kazandırmış
(1:12.54 → 1:07.40), sonra vites/yükseklik/kazanç ayarları ve kerb kullanımıyla 1:05.56. En yüksek hız 296 km/h, max |trackPos| 0,862.

**Bizim yaklaşım (tamamen kendi kodumuz, `racelab/planner/`, eski hiçbir dosyaya dokunulmadı):**
1. `track_mesh.py`: pistin gerçek geometrisi TORCS'un 3B modelinden (corkscrew.acc, sadece okunur): orta çizgi x,y,z, genişlik 12 m,
   uzunluk 3608,0 m (TORCS 3608,2). XML segmentlerinden hesap spiral virajlarda 41 m hatalıydı → mesh kullanıldı.
2. `plan_line.py`: minimum eğrilik yarış çizgisi (sınırlı doğrusal en küçük kareler, tam çözüm) + hız profili
   (yanal tutunma + aerodinamik baskı, ileri ivme / geri fren geçişleri).
3. `line_driver.py`: çizgi takipçisi (saf takip direksiyonu, hız hedefi, mevcut ABS/TCS/vites).
4. `sim.py`, `try_one.py`, `search.py`: hızlı mod tek tur ve 9 paralel otomatik parametre araması.
Araç kalibrasyonu (şampiyon telemetrisinden): fren 30–38 m/s² (3–4 g), tam gaz ivmesi 11 → 4 m/s² (0 → 280 km/h).

| Saat | İş |
|---|---|
| 23:30–00:05 | Rakip yöntemi incelemesi, pist geometrisi, çizgi, sürücü, ilk test (1978 m'ye kadar çizgide) |
| 00:05–00:30 | p1: ilk parametre araması (rastgele + yerel) |
| 00:30–03:00 | Sürücü iyileştirmeleri (kayma/yaw kontrolü, fren noktası), arama turları |
| 03:00–06:00 | Bölge bazlı hız çarpanları, kerb/margin, vites; uzun arama |
| 06:00–07:30 | En iyi aday: 10 ms ×30 dayanıklılık, teslim dosyası (yeni adla) |
| 07:30–08:00 | BLOK4_RAPORU.md |

## Günlük
- 23:29 Başlangıç. Rakip yaklaşımı incelendi; yöntem notları yukarıda.
- 23:33 İlk tur denemesi (tahmini parametreler): 1978 m'ye kadar çizgi ve hız hedefini ±%3 takip etti; 1940 m sol virajda kayarak
  pistten çıktı (direksiyon %27'de kaldı → kazanç düşük / hız yüksek).
- 23:35 p1 araması başladı (25 dk planlandı). (Bu iki satırın saatleri önce tahminle yazılmıştı, düzeltildi.)
- 23:37 p1 (rastgele + yerel arama, ~700 tur, tur başına ~1,5 s): en iyi **74,80** hasarsız, |pos| 0,99. Durduruldu.
  Analiz: planlayıcı ilk 1400 m'de RL şampiyonundan 0,66 s önde, ama tek global tutunma katsayısı en zor virajlara göre
  ayarlandığı için diğer virajlarda yavaş (ör. 1900 m: 163 vs RL 240 km/h).
- 23:39 `zone_search.py`: viraj bazlı hız çarpanları (9 viraj) + global parametreler, koordinat inişi. z1: 74,80 → 74,47 → 73,98 → 73,64 (3 tur).
- 23:40 Takip hatası analizi: ort 0,5 m, en fazla 2,8 m (Corkscrew 2400–2600 m ve 3200–3400 m). Stanley tipi kontrolcü seçeneği
  eklendi (ctrl="st"; varsayılan "pp" değişmedi); ayarsız ilk denemeler 73,99–74,42.
- 23:44 z1 durduruldu: en iyi **73,37** (12 tur, ~2050 deneme). Teslim oluşturucu `planner/build_line_submission.py` (saf Python, numpy'siz).
  Eşdeğerlik testi: sim 73,41 / teslim dosyası 73,52 → fark: iskelet komutları %.3f ile gönderiyor (sim %.4f). sim.py iskelet biçimine
  getirildi → **73,522 = 73,522 birebir**. Not: iskelet 7. vitesi 0'a (boş) çeviriyor → sürücüde vites ≤ 6.
- 23:47 z2pp (saf takip, idx 1–4) ve z2st (Stanley, idx 5–8) paralel, iskelet biçimiyle, 70 dk.
- 23:50 Stanley açık ara iyi: z2st 72,10 (z2pp saf takip 73,15 → durduruldu). Fren analizi: sürücü pedalı %30–45 kullanıyor,
  21–24 m/s² (RL 30–38). Global sert fren denemeleri ~2470 m'de (Corkscrew tepesi/inişi) pistten çıkıyor → viraj bazlı
  fren çarpanları eklendi (`zone_search2.py --brake`, `bzones`).
- 23:53 z3stb (Stanley + fren çarpanları, idx 1–4) başladı. 00:05: z3stb **71,56**, z2st 71,82 — plato.
- 00:00 Kök neden adayı: 1926 m'deki 54°'lik viraj (R 33,6 m). RL 232 km/h geçiyor, planlayıcı 153. Çizgi apekste iç kenara
  gitmiyor (n = +0,5 m; iç sınır +4,5) → çizgi yarıçapı ~58 m. Minimum eğrilik (kareler toplamı) kısa virajlarda eğriliği yayıyor.
  `line_variants.py`: düzenlemesiz ve yinelemeli ağırlıklı (minimax'a yakın) çizgiler deneniyor.
- 00:05 Hız-çarpanı 1926 virajı testi: 166 km/h'te bile |pos| 0,99; 191'de dışarı. Direksiyon viraj ortasında dalgalanıyor
  (yön terimi geri açıyor), RL ise ~0,38'de sabit tutuyor. Eklenenler (varsayılan kapalı): st_kus (kayma ön beslemesi
  v²·k), st_lp (direksiyon süzgeci). search_ctrl.py c1 (kontrolcü evrimsel arama, 8 işlem): 71,56 → **71,30**.
- 00:10 line_variants.py (L-BFGS-B ile 2 dk): tahmini plan süreleri mc 65,6 / irls 65,0 s (sabit fizik parametreleriyle).
  irls çizgisi c1 parametreleriyle: 71,16 (|pos| 1,04 → yeniden ayar gerekiyor).
- 00:11 Kayıp analizi (c1): plan 66,6 s (iyimser ivmeyle), gerçekçi ivmeyle 68,7 s; gerçekleşen 71,3 → ~2,6 s uygulama kaybı,
  kalan fark viraj hızlarının muhafazakârlığı. Düşük hızda (72–108 km/h) çıkış ivmesi sadece 3,1 m/s² (patinaj sınırı).
- 00:13 z4irls başladı: irls çizgisi + bölge/fren çarpanları + Stanley/kus/lp + global, 8 işlem, 120 dk.
- 00:15 Kaba 10 ms testi (yük altında, c1): 3/5 hasarsız (hasar 4). Final için güvenlik payı + sistem boşken ×30 gerekecek.
- 00:16 Kademeli puanlama (bitirip geçersiz turlar: 100 + süre + 200·sınır aşımı + 0,5·hasar) → arama yön bulabiliyor.
  z5irls (irls çizgisi, 8 işlem): 179 (geçersiz) → 74,04 → 72,77 → … → **70,79** (8 tur).
- 00:26 ILC (`ilc.py`, konuma bağlı direksiyon düzeltme tablosu; sürücüde ilc_file, teslimde "ilc" sütunu): öne alma 8 m ıraksadı,
  20 m yavaş, **35 m yakınsıyor** (hata rms 0,71 → 0,56 m, |pos| 0,997 → 0,977; tur 70,78 → 71,07). Hız artışı için pay sağlıyor.
- 00:28 Apeks kaydırma (viraj bazlı yanal tümsek, `nshift`; zone_search2 --shift) eklendi. Tek başına 1926'da +2 m: 70,49 ama |pos| 1,19.
- 00:32 z5irls hızlanıyor: 70,77 → 70,13 → 69,93 → **69,58** (14 tur). Kazanç büyük ölçüde fren çarpanlarından (2,1 kata kadar →
  çok daha geç frenleme) ve 1926 virajı hız çarpanından (1,44). Not: en iyi turun |pos| 0,9996 → finalde güvenlik payı gerekli.
- 00:39 z5 plato **69,576** (20 tur, 6481 deneme). Teslim dosyası `submission/torcs_jm_par_line_z5.py` (yeni): run_benchmark det
  **69,576 s, 0 hasar, pist dışı 0** (sim ile birebir). RL şampiyonundan 3,5 s hızlı.
  Yük altında 10 ms ×8: 8/8 hasarsız ama tur 70,0–70,4 ve her turda 2–3 kez pist dışı → deterministik tura aşırı uyum.
  → Sonraki aşama dayanıklılık odaklı arama (`--robust 2`: det + 2 tur 10 ms, hepsi |pos| ≤ 1 ve hasarsız).
- 00:41 z6shift (apeks kaydırma + fren + Stanley, 8 işlem) başladı.
- 00:51 z6 durduruldu: **68,95** det (teslim `submission/torcs_jm_par_line_z6.py`, run_benchmark det 68,950, 0 hasar, pist dışı 0).
- 00:52 z7rob (dayanıklılık odaklı: her aday det + 2 tur 10 ms, hepsi geçerli olmalı, puan = ortalama; 8 işlem, 150 dk).
  z6 en iyisi bu ölçütte geçersiz (181,5) → 1. turda 69,01 (ortalama) geçerli noktaya geri çekildi.
- 01:08 z7rob plato 68,992 (det = ortalama → 10 ms turlarında hiç zaman aşımı olmamış; test zorlamamış). Teslim
  `submission/torcs_jm_par_line_z7rob.py`: det 68,992, 0 hasar, pist dışı 0. irls çizgileri 0,8–2,0 m paylarla da hesaplandı.
- 01:12 sim.py: kontrollü adım kaçırma (drop_prob; cevap gönderilmez → TORCS 10 ms sonra önceki komutu kullanır). z7 en iyisi
  drop 0,002 ile 6/6 hasarsız (68,99–69,53), |pos| bazen 1,01–1,03. Dayanıklılık araması bu bozulmayla: z9rob (180 dk).
- 01:16 dt_real (yaw hızı için gerçek süre) seçeneği: belirgin fayda yok, varsayılan kapalı. ILC z7 üzerinde (35 m, γ 0,004):
  hata rms 0,77 → 0,56 m ama max |pos| 0,999 → 1,09 ve tur 68,99 → 69,41 → bu gece kullanılmıyor.
- 01:27 z9rob (adım kaçırma ile dayanıklılık): 68,96 (6 tur). Teslim `submission/torcs_jm_par_line_z9rob.py`.
  Denenip bırakılanlar: kayma oranı tabanlı patinaj kontrolü (69,16–70,16, kötü), TCS eşiği 5–100 (6,75 en iyiye yakın).
- 01:30 11 bölgeye geçiş (2457/2488 ve 2677/2733 ayrıldı; çarpanlar korunarak dönüştürüldü, başlangıç 69,30). z10rob11 başladı.
- 01:39 z10rob11 3 turda **68,93** (teslim `submission/torcs_jm_par_line_z10rob11.py`). `final_check.py` (run_benchmark det + N×10 ms özeti) yazıldı.
  Yük altında z9 ×2: hasarsız 2/2 ama pist dışı (|pos|>1) her turda → gerçek koşularda zaman aşımları yarış başında oluyor,
  bizim adım kaçırma testi ilk 5 adımı hariç tutuyordu. Düzeltme: yarış başında da kaçırma (drop_start) + pist sınırı 0,97 (--pos-lim).
- 01:42 z11rob97 (yeni dayanıklılık ölçütü, z10 en iyisinden) başladı, 180 dk.
- 01:44 z11rob97 (0,97 sınırı) geçerli bölge bulamadı (159) → durduruldu. z10 en iyisi bozulmuş 6 turda 6/6 hasarsız
  (|pos| 0,99; bir turda 1,051). Çarpanları düşürmek kötüleştiriyor (diğer parametreler onlara göre ayarlı).
  Ölçüt: det |pos| ≤ 1,0; bozulmuş turlar hasarsız ve |pos| ≤ 1,05 (kullanıcı kuralı: 30/30 hasarsız). z12rob başladı (200 dk).
- 02:16–02:24 **Final doğrulama (sistem boş, run_benchmark det + 30×10 ms):**
  | Aday | det | 10 ms ×30 hasarsız | pist içi | ort |
  |---|---|---|---|---|
  | **line_z12rob** | **68,792** | **30/30** | 0/30 | 69,79 |
  | line_z10rob11 | 68,934 | 30/30 | 0/30 | 69,96 |
  | line_z9rob | 68,956 | 22/30 | 0/30 | 69,82 |
  | line_z7rob | 68,992 | 20/30 | 0/30 | 69,94 |
  | line_z6 | 68,950 | 27/30 | 0/30 | 69,83 |
  Pist dışı her turda aynı yerde: s≈3048 m, |pos| 1,05–1,06, ~1 s. 10 ms turu 50 m'de bile 0,07 s geride ve fark düzenli büyüyor.
- 02:30 **Kök neden:** run_benchmark 10 ms koşullarında yarış başındaki zaman aşımından sonra kalıcı **1 adım gecikme** oluşuyor
  (zaman aşımı sayılmıyor). Simülasyonda delay1 ile birebir üretildi: 69,874 s, |pos| 1,0475 @ 3048 (run_benchmark: 69,79, 1,047 @ 3048).
  RL'de de aynı (det 73,10 / 10 ms 74,32); GUI testleri det ile eşleşmişti.
- 02:32 z13d1: her aday det + delay1 (ikisi de hasarsız ve |pos| ≤ 1,0), puan = ortalama. 200 dk.
- 02:33–02:40 Hatalar düzeltildi: (1) delay1 modunda bitiş mesajı kapanışla geliyor → tur süresi son curLapTime'dan; (2) `vert` terimi
  hiç çalışmıyormuş (dizi/sayı hatası), düzeltildi (vert=0'da sonuç aynı); (3) puan sıralaması: bitirmeyen tur, bitirip kenarı
  aşan turdan iyi puan alabiliyordu → bitirmeyen artık her zaman ≥ 300. z15d1 (det + delay1, ikisi de |pos| ≤ 1,0) başladı.
- 02:56–03:05 z15/z16: det 68,806, |pos| 1,00002 (s=761 m, kenar çizgisine 0,1 mm) — gecikmeli turla birlikte tek koordinat
  hamlesiyle düzeltilemiyor. Ölçüt: det |pos| ≤ 1,001, delay1 |pos| ≤ 1,03 (hepsi hasarsız). z17d1 başladı.
  Not: araç merkezi bazı virajlarda kenar çizgisine kadar geliyor (aracın yarısı kerb üzerinde); rakip ±0,86 içinde kalmış.
- 03:06–03:12 Gecikme (delay1) sağlamlığı araştırması: durum tahmini (`pred`) taşmayı azaltmıyor (1,048 → 1,085); 2993 virajı
  ızgarası etkisiz; kontrolcü kazanç ızgarası (36 kombinasyon) hepsi hasarsız ama delay1 |pos| 1,02–1,09; kenar payı 1,7/2,0 m
  1,046/1,020 ve yavaş (69,04/69,50). Sonuç: 1 adım gecikmede 2–5 cm'lik kenar taşması bu agresiflikte kaçınılmaz, hasar yok.
  **Ana aday z12** (det 68,79, 30/30 hasarsız). Yürütme analizi: plan 64,2 s (iyimser ivme), virajlarda hedefe ±3 km/h uyuluyor.
- 03:12 e1: evrimsel arama (aynı anda 1–4 boyut; bölge hız/fren/apeks + kontrolcü), det + 1 bozulmuş tur, 150 dk.
- 03:38 e1: 68,81 → **68,39** (det 68,394; ~6000 deneme). Evrimsel çok boyutlu hamleler yerel optimumdan çıkardı.
- 04:03 e1: **68,13** (det 68,128). Ara teslim: submission/torcs_jm_par_line_e1a.py
- 04:28 e1: **67,84** (det 67,828). Ara teslim: submission/torcs_jm_par_line_e1b.py
- 05:15 e1 plato **67,77** (det 67,768; teslim `submission/torcs_jm_par_line_e1c.py`). Ara doğrulama (sistem boş, 30×10 ms):
  **e1c 20/30 hasarsız**, e1b 16/30 → kullanıcı kuralını (30/30) geçmiyor. Simülasyonda e1c delay1: hasar 13, |pos| 1,165
  (z12 delay1: hasarsız, 1,05). Tek bozulmuş turla değerlendirme gecikme sağlamlığını korumadı.
- 05:18 e2d1: evrimsel arama z12'den, her aday det + delay1 (ikisi de deterministik; hasarsız, det |pos| ≤ 1,0, delay1 ≤ 1,06), 125 dk.
- 05:44 e2d1: ortalama 69,08 (det 68,746, delay1 69,408, ikisi de hasarsız; delay1 |pos| 1,056).
- 06:10 e2d1 plato (ortalama 69,077: det 68,746, delay1 69,408). z12–e1c parametre karışımları: ara noktalar det'te bile sınırı aşıyor → bırakıldı.
- 06:20 **Final doğrulama e2d1** (`submission/torcs_jm_par_line_e2d1.py`, sistem boş): det **68,746**, 0 hasar, pist dışı 0;
  10 ms ×30: **30/30 hasarsız**, ort 69,27 (z12: 69,79). → **Yeni ana aday e2d1.**
- 06:27 e3d1: evrimsel arama e2d1'den, aynı ölçüt, 50 dk.
- 07:12 e3d1 bitti: det 68,582 (delay1 69,222). Teslim `submission/torcs_jm_par_line_e3d1.py`: 10 ms ×30 **29/30** → kural dışı.
- 07:16 e2d1 ikinci ×30: **30/30 hasarsız** (ort 69,33) → toplam 60/60. **Önerilen: torcs_jm_par_line_e2d1.py (68,746 s).**
  Rapor: BLOK4_RAPORU.md. Çalışan süreç yok.
