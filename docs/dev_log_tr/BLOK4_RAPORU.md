# Blok 4 raporu — 2026-10-12 23:29 → 2026-10-13 08:00: yarış çizgisi + hız planı

Saat saat günlük: [BLOK4_PLANI.md](BLOK4_PLANI.md). Hiçbir eski dosya silinmedi ya da üzerine yazılmadı; tüm yeni kod
`racelab/planner/` altında, yeni teslim dosyaları `racelab/submission/torcs_jm_par_line_*.py`. RL şampiyonu ve yedeği yerinde.

## Sonuç
**Önerilen teslim dosyası: `racelab/submission/torcs_jm_par_line_e2d1.py`** — iki ayrı 30'lu testte **60/60 hasarsız**.

| Teslim dosyası | Deterministik (= GUI, adım kaçmazsa) | 10 ms ×30 (sistem boş) | ×30 ort | Not |
|---|---|---|---|---|
| **`torcs_jm_par_line_e2d1.py`** | **68,746 s**, 0 hasar, pist dışı 0 | **30/30 + 30/30 hasarsız** | 69,27 / 69,33 | **önerilen** |
| `torcs_jm_par_line_e3d1.py` | 68,582 s, 0 hasar | 29/30 | 69,10 | daha hızlı ama 1 turda hasar → kural dışı |
| `torcs_jm_par_line_z12rob.py` | 68,792 s, 0 hasar | 30/30 hasarsız | 69,79 | yedek |
| `torcs_jm_par_line_e1c.py` | **67,768 s**, 0 hasar | 20/30 | 68,32 | en hızlı ama kuralı geçmiyor |
| `torcs_jm_par_rl_v10d_dist_r02.py` (RL, önceki) | 73,098 s | 30/30 | 74,32 | eski şampiyon |

**Kazanç: 73,10 → 68,75 s (−4,35 s).** Rakip (One Qubit Racing) 65,56 s; fark ~3,2 s.

GUI testi (Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race):
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_line_e2d1.py --mode gui --label "line_e2d1@GUI"
```
Beklenen ekran süresi ~01:08:75, 0 hasar. Teslim dosyası **saf Python** (numpy gerekmez), torcs_jm_par.py iskeleti ve
REQUIRED blokları korunur.

**Dürüst notlar:**
- **Kenar kullanımı:** araç merkezi bazı virajlarda kenar çizgisine kadar geliyor (deterministik |trackPos| max 0,995).
  Rakip ±0,86 içinde kalmış.
- **10 ms koşuları:** run_benchmark'ın 10 ms koşularında yarış başındaki zaman aşımından sonra **kalıcı 1 adım gecikme**
  oluşuyor (simülasyonda birebir üretildi). Bu durumda tur ~0,5 s yavaş ve s≈3048 m'de araç kenarı ~5 cm aşıyor
  (her turda, ~1 s). Hasar yok. RL modelinde de aynı sistematik fark vardı (73,10 / 74,32), GUI testleri ise deterministik
  sonuçla birebir eşleşmişti. GUI'de de aynısını bekliyoruz ama doğrulanmalı.

## Neden ve ne yaptık
**Durum:** Rakip takım (One Qubit Racing) Corkscrew'da 1:05.56 bildirdi. Açıklamalarına göre (web sitesi ve README okundu,
**kodları kopyalanmadı**) yöntemleri: pistin geometrisinden önceden hesaplanmış yarış çizgisi + her noktada izin verilen hız,
canlı sensörlerle düzeltme.

**Bizim uygulamamız (tamamen kendi kodumuz):**
1. **Pist geometrisi** (`planner/track_mesh.py`): TORCS'un 3B modelinden (corkscrew.acc) gerçek orta çizgi (x, y, z), genişlik
   12 m, uzunluk 3608,0 m (TORCS 3608,2). XML segmentlerinden hesap spiral virajlarda 41 m hatalıydı.
2. **Yarış çizgisi** (`plan_line.py`, `line_variants.py`): minimum eğrilik + yinelemeli ağırlıklandırma (irls).
3. **Hız planı:** yanal tutunma (aerodinamik) → viraj hız sınırı; ileri (motor) ve geri (fren) geçişleri. Araç ölçümü
   (RL telemetrisi): fren 30–38 m/s², tam gaz ivmesi 11 → 4 m/s².
4. **Sürücü** (`line_driver.py`): Stanley tipi direksiyon (yön + yanal hata + eğrilik ön beslemesi + yaw sönümü + süzgeç),
   hız hedefi → gaz/fren, mevcut ABS/patinaj/vites kuralları (vites ≤ 6; iskelet 7'yi boş vitese çeviriyor).
5. **Otomatik ayar** (`search.py`, `zone_search2.py`, `search_ctrl.py`, `evo_all.py`): 8–9 paralel hızlı simülasyon (tur başına
   ~1,5 s, gece boyunca ~150 000 tur). Ayarlanan: 11 virajın her biri için hız çarpanı, fren çarpanı ve apeks kaydırması;
   kontrolcü kazançları. Son aşamada her aday hem gecikmesiz hem 1 adım gecikmeli turda hasarsız olmak zorunda.
6. **Teslim** (`build_line_submission.py`): iskelet + gömülü plan; simülasyonla birebir aynı sonuç (iskeletin %.3f komut
   biçimi simülasyona da uygulandı). Final doğrulama: `final_check.py` (run_benchmark det + 30×10 ms).

## İlerleme
| Aşama | Tur (det, hasarsız) |
|---|---|
| RL şampiyonu (önceki) | 73,10 |
| İlk parametre araması (p1) | 74,80 |
| Viraj bazlı hız çarpanları (z1) | 73,37 |
| Stanley kontrolcüsü (z2st) | 72,10 |
| + fren çarpanları (z3stb) | 71,56 |
| + kontrolcü araması (c1) | 71,30 |
| irls çizgisi (z5) | 69,58 |
| + apeks kaydırma (z6) | 68,95 |
| dayanıklılık + 11 bölge (z12) | 68,79 (30/30) |
| evrimsel, sadece det + rastgele bozulma (e1) | 67,77 (20/30 ✗) |
| **evrimsel, det + 1 adım gecikme (e2d1)** | **68,75 (60/60)** |
| + 50 dk daha (e3d1) | 68,58 (29/30 ✗) |

## Denenip bırakılanlar
- Saf takip (pure pursuit) kontrolcüsü: Stanley'den ~1 s yavaş.
- Global sert fren: Corkscrew tepesinde pistten çıkıyor → viraj bazlı fren çarpanları.
- Kayma oranı tabanlı patinaj kontrolü, TCS eşikleri 5–100: mevcut eşik (6,75) en iyisi.
- ILC (konuma bağlı öğrenen direksiyon düzeltmesi): ortalama hata azaldı ama kenar aşımı arttı, tur yavaşladı.
- Gecikme telafisi (durum tahmini), daha geniş kenar payı (1,7 / 2,0 m): gecikmeli turdaki taşmayı gidermedi.
- Çizgi varyantları (farklı irls ağırlıkları), z12–e1c parametre karışımları.

## Rakiple fark (~3,2 s) ve sıradaki adımlar
- **Gecikmeye dayanıklı daha hızlı ayar:** e1c deterministik 67,77 s ama 1 adım gecikmede hasar alıyor. Gecikme koşulunu
  baştan hedefleyen daha uzun bir arama (birkaç saat) 67,x'e inebilir.
- **Kontrolcü:** viraj içinde direksiyon hâlâ dalgalı; RL modeli aynı virajda direksiyonu sabit tutuyor. Hızla değişen
  kazançlar ya da model tabanlı ön besleme (hız ve eğriliğe göre gereken direksiyon haritası) denenebilir.
- **Çizgi:** minimum eğrilik global bir yaklaşım; gerçek minimum süre çizgisi optimizasyonu (çizgi + hız birlikte) en büyük
  potansiyel ama birkaç saatlik iş.
- **Teslim için:** GUI'de e2d1'i dene (video adayı). Pist kenarına yakınlık konusunda organizatörün kuralı net değilse
  (rakip ±1 içinde kaldığını özellikle belirtmiş) daha temkinli bir sürüm de hazırlanabilir.
