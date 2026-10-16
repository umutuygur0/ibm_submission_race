# Blok 5 — 2026-10-13 21:33 → 2026-10-14 07:00: rakiple fark analizi + çeşitlilik odaklı arama

Hedef: 65,56 s'yi (One Qubit Racing) geçmek. Başlangıç: `submission/torcs_jm_par_line_e2d1.py` 68,746 s (60/60 hasarsız).
Kural: 30/30 hasarsız. Rakibin açık reposu (`ıncelenen_repos/onequbit_IBM-AI-RACE-CHALLENGE`, **lisans yok → sadece
okuma/yerel analiz, hiçbir kodu ya da ayar değeri teslimimize kopyalanmaz**) tur telemetrisiyle karşılaştırma için kullanıldı.

## Rakip turu (65,562 s) ile karşılaştırma — bulgular
1. **1926 m virajı (+0,75 s):** rakip 264–281 km/h, frenlemeden, direksiyonu yumuşakça 0,62'ye çıkarıp sabit tutuyor.
   Biz 191 km/h'e frenliyoruz, direksiyon dalgalı. Çizgi benzer (giriş −0,8, apeks +0,67, çıkış −0,8).
   Kök neden adayı: Stanley yön terimi araç gövdesinin yönünü kullanıyor; araç kayarak döndüğünde (gövde ~0,3 rad içeri)
   direksiyonu geri açıyor. → **gidiş yönü (hız vektörü = gövde + kayma açısı, sınırlı)** seçeneği eklendi (st_course, st_slipmax).
2. **Vites:** rakip vitesi ~19.300–19.500 devirde büyütüyor (biz 18.650), yavaş çıkışlarda 1. vitese düşmüyor (2. vites 27–173 km/h).
   72–144 km/h tam gaz ivmesi 15,3 m/s² (bizde 11–12, yavaş çıkışlarda ~3; patinaj sınırı). → vites genleri eklendi
   (upshift ≤ 19.700, g2_v: bu hızın üstünde en az 2. vites, dn_off). Ayarsız: 68,746 → 68,54.
3. Düzlüklerde en yüksek hızları ~10 km/h fazla (çıkış hızı + vites). Yavaş virajlarda (2400–2500, 3200–3300) biz öndeyiz.

## Çeşitlilik (yerel minimumdan çıkış)
`planner/evo2.py`: ada modeli — 6 bağımsız popülasyon (farklı başlangıçlar: e2d1, z12 soyu, e1c soyu, gidiş yönü kontrolcüsü,
farklı irls çizgisi, farklı vites), %15 büyük sıçrama (5× mutasyon), %12 adalar arası çaprazlama. Değerlendirme: det + 1 adım
gecikme (ikisi de hasarsız).

## Günlük
- 21:33 Başlangıç. Rakip reposu klonlandı (33 MB). Telemetri karşılaştırması (yukarıda).
- 21:37 g1 (5 ada) başladı; 21:40 vites bulgusu → g2 (6 ada, vites genleriyle) yeniden başladı, 490 dk (~05:50).
  Ada 0 (e2d1 + vites) ve 5 (19.100) başlangıçta gecikmeli turda geçersiz; arama düzeltecek.
- 21:43 Fren karşılaştırması: rakip 6 bölgede (7,1 s), biz 10 bölgede (9,8 s) frenliyoruz. Rakip 980, 1874, 2670, 2917 m'de hiç
  frenlemiyor. Yanal ivme zarfı (telemetri + pist geometrisi): rakip 230–290 km/h'te 47–50 m/s² (~5 g), biz 17–30 → hızlı virajlar
  büyük potansiyel. Gidiş yönü kontrolcüsü 1926'yı 219 km/h'te pist içinde tutuyor (gövde yönüyle çıkıyordu).
  Kalibre edilmiş tutunma modeli (mu 3,35, aero 0,00091): plan 58–61 s ama 482 m'de çıkıyor; sadece aero artışı 763'te çıkıyor →
  global değil viraj bazlı ayar gerekiyor.
- 21:44 `greedy_corners.py` (virajları sırayla yerel sınırına zorlama, det + delay1 yerel geçerlilik) idx 9'da başladı.
- 21:48 g2 genel en iyi **67,88** (det+delay1 ortalaması; e1c soyu + vites genleri). Greedy: çoğu virajda hız ×1,0 (sınırda), fren 1033'te ×1,45.
- 22:00 Kalkış analizi: geri sayımda 1. viteste motor boğuluyor, kalkışta direksiyon −1,0, 2. vitese 48 km/h'te geçiş (rakip 29).
  Kalkış seçenekleri (launch, launch_up_v, st_lowv) eklendi — tek başına kazanç yok.
- 22:04 **Debriyaj:** rakip kodunun yorumlarında TORCS simuv2 fiziği: motor torku × min(3(1−debriyaj), 1) → pedal ≤ 2/3'te tam tork
  geçer ama motor güç bandına çıkar; vites değişiminde debriyaj tutulmazsa TORCS gazı 0,1'e sınırlar. Kendi testimizle doğrulandı
  (1 s'de 44,6 → 49,6 km/h). Kendi debriyaj kontrolcümüz (cl_max/cl_top/cl_band/cl_shift; vites kararı tahrik tekerleği devrinden):
  ayarsız **67,57 → 67,11 s**. Teslim eşdeğerliği birebir (67,112). sim.py artık debriyaj değerini de gönderiyor.
- 22:08 g3: 7 ada (g2 adaları + açgözlü sonuç, debriyaj açık; farklı debriyaj başlangıçları), 460 dk (~05:50). Başlangıçta ada 6 (açgözlü + debriyaj) 67,46 ort (det 67,22, delay1 67,71).
- 22:10 **Viraj bazlı kontrolcü kazançları** (kzones / _kz: her virajda yön, yanal, ön besleme çarpanı; sürücü + teslim + arama).
  1926 virajı ızgarası: hız ×1,2, yön ×0,5, yanal ×0,5, ön besleme ×1,3 → viraj **242 km/h** (önce 204–219), |pos| 0,926,
  det tur **66,712** (önce 67,22). Hızlı virajlarda az geri besleme + çok ön besleme gerekiyor.
- 22:14 g3 durduruldu (en iyi det 67,06 / delay1 67,39). `greedy2.py`: 11 virajın her biri için 288 kombinasyon (hız × 3 kazanç)
  + fren çarpanı, det + delay1 yerel geçerlilik, 9 paralel. gr1 (3 geçiş) başladı.
- 22:23 gr1 geçiş 0, 7/11 bölge: det **66,26**, delay1 66,51 (1926: hız ×1,81, fren ×4,0, kazançlar ×0,6/0,6/1,3).
- 22:39 **gr1 geçiş 1: det 65,498 < rakip 65,562** (delay1 65,882, ikisi de hasarsız). Teslim `submission/torcs_jm_par_line_gr1a.py`:
  run_benchmark det **65,498**, 0 hasar, pist dışı 0, max 294,9 km/h. (30/30 doğrulaması sistem boşken yapılacak.)
  Kazançlar: viraj başına yön ×0,36–0,75, yanal ×0,36–0,6, ön besleme ×0,8–1,7; hız çarpanları 1926: ×1,92, 2677: ×1,42, 2733: ×1,44.
- 22:53 gr1 bitti (3 geçiş): det **65,218**, delay1 65,878.
- 23:00 **Final doğrulama (sistem boş, 30×10 ms):**
  | Teslim | det | ×30 hasarsız | pist içi | ort |
  |---|---|---|---|---|
  | `torcs_jm_par_line_gr1b.py` | **65,218** | **30/30** | 10/30 | 65,63 |
  | `torcs_jm_par_line_gr1a.py` | 65,498 | **30/30** | **30/30** | 65,78 |
  **İkisi de rakibin 65,562'sini geçiyor ve 30/30 kuralını sağlıyor.**
- 23:18 gr2 (apeks aşamalı, gr1b'den): det 65,156 (geçiş 1). Bazı virajlarda yanal kazanç ×0,08–0,13'e indi (riskli olabilir).
- 23:20 gr1b vs rakip: bölümlerin çoğunda başa baş ya da öndeyiz (Corkscrew 2350–2650: −0,44 s; 1926: 263/264 km/h).
  Kalkış hâlâ +0,15 s (100 m'de). Kalkış ızgarası (launch, launch_up_v, cl_top, tcs): iyileşme yok.
  **Uyarı:** gr1b parametrelerinde küçük değişiklikler (cl_top 18.155 → 18.000) hasar 2–4 üretiyor → parametre uzayında bıçak sırtı.
  Zamanlama sağlamlığı 30/30, ama güvenli seçenek gr1a (30/30 hasarsız + 30/30 pist içi).
- 23:25 Parametre sağlamlığı (`param_robust.py`, tüm parametreler ±%1 rastgele, 20 tur): gr1a 15/20, gr1b 10/20 hasarsız → ikisi de
  hassas, gr1b daha çok. greedy2'ye `--prob/--prel` eklendi (aday, ±%0,7 oynatılmış 2 turda da yerel hasarsız olmalı).
- 23:26 gr2 durduruldu: det 65,156, delay1 65,746 (teslim `submission/torcs_jm_par_line_gr2.py`, ×30 bekliyor).
  gr3rob başladı: gr1a'dan, apeks + fren + parametre sağlamlığı, daha sıkı sınırlar (det |pos| ≤ 0,98, delay1 ≤ 1,03), 2 geçiş.
- 23:52 gr3rob (sağlamlık kısıtlı) geçiş 1: det **65,148**, delay1 **65,522** (gr2'den iki ölçütte de iyi).
- 00:15 gr3rob bitti: det 65,162, delay1 65,556. ×30: **30/30 hasarsız** (20/30 pist içi, ort 65,27, zaman aşımı ort 1,1).
  gr2 ×30: 30/30 ama bu testte hiç zaman aşımı olmadı (gecikme sınanmadı). Parametre ±%1: gr3rob 12/20, gr2 10/20.
- 00:20 **Kalkış:** gaz %56–87 (patinaj kontrolü kesiyor), devir 7–15 bin; rakip kalkışta güçlü patinaja izin veriyor.
  Kalkışta patinaj eşiği seçeneği (tcs_lv / tcs_l: distRaced < 500 m ve hız < tcs_lv iken eşik tcs_l). 160 km/h / 100:
  100 m'ye 4,206 → **4,052 s** (rakip 4,05), tur **64,964** (delay1 65,578, hasarsız). Debriyaj > 2/3 kötü (tork kesiliyor).
- 00:22 **gr3L** (`submission/torcs_jm_par_line_gr3L.py`): det **64,964**, ×30 **30/30 hasarsız** (29/30 pist içi, ort 64,97).
  gr4rob başladı (gr3L'den, sağlamlık kısıtlı ızgara, 3 geçiş).
- 00:42 gr4rob hatalıydı: yerel ölçüt (bölge + 150 m) iyi ama sonrasında aracı pistten çıkaran aday seçildi (bölge 0, apeks −4,35 m)
  → tüm tur bozuldu. Düzeltme: aday tüm turu da (det + delay1) hasarsız bitirmeli. gr5rob (gr3L'den) yeniden başladı.
- 01:00 gr5rob (tüm tur geçerliliğiyle) 4 bölgede durduruldu: det 64,964, delay1 **65,418** (gr3L'den iyi). Kazanç küçük.
  Dar kenar payı denemesi (gr3L ayarlarıyla, yeniden ayarsız): 1,0 m → det 64,79 (hasar 5), delay1 65,01 hasarsız → potansiyel.
  evo2'ye viraj kazanç genleri (_kz) eklendi. g4: 7 ada (pay 1,5 / 1,3 / 1,2 / 1,0 / 1,0 / 0,8 ve gr1a + kalkış), 270 dk (~05:35).
- 01:27 g4: ada 1 (pay 1,3) det **64,766** / delay1 65,148; ada 2 (1,2) 64,792/65,224; ada 6 (gr1a soyu) 64,842/65,156; ada 0 64,89/65,108.
- 01:57 g4 ortalamalar 64,90–64,97 (det 64,74–64,91, delay1 65,00–65,14); ada 5 (pay 0,8) hâlâ geçersiz.
- 02:27 g4 tüm adalar geçerli. En iyi ort 64,856 (ada 0: det 64,778, delay1 **64,934**); en iyi det 64,702 (ada 2, pay 1,2).
- 02:57 g4: en iyi ort 64,803 (ada 4: det 64,722 / delay1 64,884); en iyi det 64,676 (ada 2). Parametre ±%1 (10 tur): ada 0 7/10,
  ada 4 6/10, ada 3 5/10, ada 6 2/10, **ada 2 0/10** → evrim sağlamlığı gözetmeden bıçak sırtına kayıyor.
  Değerlendiriciye `_prob/_prel` eklendi (aday ±%0,7 oynatılmış 2 tam turda da hasarsız olmalı). g5rob (g4 adalarından) 145 dk.
- 03:30 g5rob (parametre sağlamlıklı): ada 4 det **64,686** / delay1 **64,822**; ada 0 64,772/65,004.
- 04:00 g5rob: ada 4 det 64,674 / delay1 64,822 (en iyi), ada 6 64,718/64,964, ada 1 64,75/65,078.
- 04:29 g5rob plato: ada 4 det 64,670 / delay1 64,808.
- (04:35 civarı bilgisayar kapanmış; sonuçlar diskte.) 10:16 Devam.
- 10:20 g5rob adayları ×30 (sistem boş): g5r4 (det 64,670) **29/30**, g5r6 28/30, g5r0 29/30 → kural dışı.
  gr3L ve gr2 yeniden ×30: 30/30 ama yine 0 zaman aşımı. Zaman aşımları adaya bağlı: g5r* koşularında yarışın ilk cevabında
  zaman aşımı → kalıcı 1 adım gecikme; gr3L'de bu olmuyor → gr3L'in ×30'u gecikmeyi sınamamıştı.
- 10:30 `lag_stress.py` (her adaya aynı zorla gecikme: delay1 + başta kaçırma + %0,2 kaçırma, ve sadece kaçırma; 30'ar tur):
  gr3L gecikme 30/30, kaçırma 30/30 (pist içi 30/30) → **sağlam**; gr1a 30/30 + 30/30 (her ikisinde pist içi) → en sağlam;
  gr3rob 30/30 + 30/30; gr2 29/30 ✗; g5r4 kaçırmada 16/30 ✗; g5r0 29/30 + 16/30 ✗. **Önerilen gr3L (64,964) sürüyor.**
- 10:35 greedy2 `--stress`: aday ayrıca 2 rastgele kaçırma turu + 1 gecikme+kaçırma turunda tüm turu hasarsız bitirmeli.
  gr6s (gr3L'den, 3 geçiş) başladı.
- 11:12 gr6s durduruldu: stres kısıtı altında yerel (bölge sonu) seçim toplam turu kötüleştirdi (64,964 → 65,11), bazı kazançlar ×0,05'e
  indi. Bu yöntemde getiri bitti. **Önerilen: gr3L (64,964 s)**; en sağlam yedek gr1a (65,498 s).
