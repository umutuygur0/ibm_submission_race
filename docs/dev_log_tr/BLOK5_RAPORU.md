# Blok 5 raporu — 2026-10-13 21:33 → 2026-10-14 07:00: rakibi geçmek

Saat saat günlük: [BLOK5_PLANI.md](BLOK5_PLANI.md). Hiçbir eski dosya silinmedi ya da üzerine yazılmadı.

## Sonuç (2026-10-14 11:15, son)
**Rakibin 65,562 s'si geçildi.** Önerilen teslim: **`racelab/submission/torcs_jm_par_line_gr3L.py` — 64,964 s, hasarsız.**

| Teslim dosyası | Det (= GUI) | run_benchmark 10 ms ×30 | Stres: kalıcı gecikme ×30 | Stres: adım kaçırma ×30 (pist içi) | Karar |
|---|---|---|---|---|---|
| **`torcs_jm_par_line_gr3L.py`** | **64,964** | 30/30 + 30/30 | **30/30** | **30/30 (30/30)** | **önerilen** |
| `torcs_jm_par_line_gr1a.py` | 65,498 | 30/30 | 30/30 (pist içi 30/30) | 30/30 (30/30) | en sağlam yedek |
| `torcs_jm_par_line_gr3rob.py` | 65,162 | 30/30 | 30/30 | 30/30 (30/30) | sağlam |
| `torcs_jm_par_line_gr2.py` | 65,156 | 30/30 | 29/30 | 30/30 (25/30) | ✗ |
| `torcs_jm_par_line_g5r4.py` | 64,670 | 29/30 | 30/30 | 16/30 | ✗ kırılgan |
| `torcs_jm_par_line_g5r0.py` | 64,778 | 29/30 | 29/30 | 16/30 | ✗ |
| One Qubit Racing (rakip) | 65,562 | — | — | — | — |

Stres testi (`planner/lag_stress.py`): run_benchmark'ta zaman aşımı adaya göre değişiyor (gr3L'de hiç oluşmuyordu), bu yüzden her
adaya aynı zorla bozulma uygulandı: kalıcı 1 adım gecikme + başta kaçırma + %0,2 rastgele kaçırma, ve yalnız kaçırma.
Teslim dosyaları saf Python (numpy gerekmez), torcs_jm_par.py iskeleti ve REQUIRED blokları korunur.

**Gecenin dersi:** daha hızlı deterministik turlar (64,67) bulundu ama bıçak sırtı çıktı; sağlamlık testleri olmadan seçilseydi
kural (30/30) çiğnenirdi. Son 2 saatlik stres kısıtlı arama (gr6s) gr3L'i geçemedi → bu yaklaşımın getirisi bitti.

## Rakiple farkı nasıl kapattık (68,75 → 64,96 s)
Rakibin açık reposundaki tur telemetrisi bizim turumuzla mesafe bazında karşılaştırıldı. Repo **lisanssız**: yalnızca
`ıncelenen_repos/` altında okundu ve analiz edildi; **hiçbir kodu ya da ayar değeri teslimimize kopyalanmadı**, her şey kendi
kodumuzla ve kendi aramalarımızla bulundu.

| Bulgu (telemetri karşılaştırması) | Çözümümüz | Kazanç |
|---|---|---|
| Rakip vitesi ~19.400 devirde büyütüyor, yavaş çıkışlarda 1. vitese düşmüyor | vites genleri (upshift ≤ 19.700, g2_v, dn_off) | ~0,2 s |
| Kalkışta ve her vites değişiminde motor rölantiye düşüyor (bizde); TORCS'ta debriyaj ≤ 2/3 tam tork geçirir (kendi testimizle doğrulandı) | kendi debriyaj kaydırma kontrolcümüz (cl_*) | ~0,46 s |
| 4 hızlı virajda rakip hiç frenlemiyor; yanal ivme 230–290 km/h'te ~5 g; bizim direksiyon dalgalı | **viraj bazlı kontrolcü kazançları** (hızlı virajlarda az geri besleme, çok ön besleme) + viraj viraj ızgara araması (greedy2) | ~1,8 s |
| Yerel minimumda takılma | ada modeli evrimsel arama (6–7 bağımsız popülasyon, büyük sıçramalar, çaprazlama) | ~0,3 s |
| Kalkışta patinaj kontrolü gazı kesiyor (rakip kalkışta güçlü patinaja izin veriyor) | kalkışa özel patinaj eşiği (tcs_lv, tcs_l) | ~0,2 s |
| Sonuçlar parametre uzayında hassas (±%1 oynatmada 10–15/20) | sağlamlık kısıtlı ızgara (aday ±%0,7 oynatılmış turlarda da hasarsız olmalı) | sağlamlık |

1926 m virajı: 191 → ~255 km/h (rakip 264). Her aday hem gecikmesiz hem 1 adım gecikmeli turda hasarsız olmak zorunda
(run_benchmark 10 ms koşulları).

## GUI testi
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_line_gr3L.py --mode gui --label "line_gr3L@GUI"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_line_gr1a.py --mode gui --label "line_gr1a@GUI"
```
Beklenen: gr3L ~01:04:96, gr1a ~01:05:50 (0 hasar).
