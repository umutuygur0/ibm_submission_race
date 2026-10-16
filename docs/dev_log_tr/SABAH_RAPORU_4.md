# Sabah raporu 4 — gece çalışması 4 (2026-10-08 23:41 → 2026-10-09 09:00)

> Son sürüm (08:20). Günlük: [GECE4_PLANI.md](GECE4_PLANI.md).
> Önceki raporlar korunuyor (SABAH_RAPORU.md, _2, _3, ARASTIRMA_HIZ.md).

## Sonuç: TORCS botlarını geçtik
**Önerilen teslim dosyası: `racelab/submission/torcs_jm_par_rl_v10d_dist.py`**

| | Tur (Corkscrew) | Not |
|---|---|---|
| TORCS botları (aynı araç, referans) | 74,20 s | olethros |
| Dünkü GUI'de doğrulanan en iyimiz | 78,15 s | s47_assist |
| **Yeni: v10d_dist** | **73,582 s, 0 hasar**, max 270,5 km/h (deterministik = GUI) | 10 ms ×10: **10/10 hasarsız** (73,6–74,9) |
| v10b_best_dist | 73,728 s, 0 hasar | 10/10 (74,4–75,7) |
| v10b_v04324_dist (yedek) | 74,416 s, 0 hasar | 10/10 |
| v10b_best_plain (yardımcısız) | 75,156 s, 0 hasar | 10/10 |

Hiçbir eski dosyanın üzerine yazılmadı.

GUI testi (Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race):
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v10d_dist.py --mode gui --label "SUBMISSION_v10d_dist@GUI"
```
Beklenen: ~73,58 s (TORCS ekranında 01:13:58 civarı), 0 hasar.

## Ne değişti: konum bilen agent (v10)
- Agent artık pistteki konumunu da görüyor: başlangıç çizgisinden uzaklık, sin/cos harmonikleri olarak (k=1..32, en kısa dalga ~113 m).
  Yarışma tek pistte (Corkscrew) olduğu için agent piste özgü fren/gaz noktalarını öğrenebiliyor.
- Sıfırdan değil: önceki sağlam modelin (v4d) ağırlıkları 32→44 girişe **genişletildi**, yeni girişler sıfırdan başladı
  (başlangıçta davranış birebir aynı — test edildi: 81,904 = 81,904).
- Sonuç: 30 dakikada yardımcısız 76,9 s (konumsuz en iyi model yardımcısız 79,8 s idi); 2 saatte yardımcısız 75,2 s,
  mesafe yardımcısıyla 73,73 s.

## Gecenin seyri
| Eğitim | Ne | En iyi (yardımcıyla, deterministik) |
|---|---|---|
| v10a, v10b | ilk konum bilen modeller (v4d genişletilerek, 2 saat) | v10b/best 73,73 s |
| n5 hattı (s80-82) | v10b'nin son durumundan devam, 60 dk | 74,05 s → devam etmek işe yaramadı, bırakıldı |
| v10c, **v10d** | v10a/b tarifi, yeni tohumlar | **v10d/v04188 73,44 s** (teslimde 73,58) |
| v10e, v10f | aynı tarif, son tohumlar | 74,51 s |

Ders: en iyi modeller sağlam bir başlangıçtan (v4d genişletilerek) yapılan 2 saatlik eğitimlerden çıkıyor;
eğitilmiş bir v10 modelinden devam etmek (n5) iyileştirmedi.

## Sıradaki öneriler
- GUI'de v10d_dist testi (video adayı).
- Aynı tarifle birkaç tohum daha; mesafe yardımcısının parametreleri v10 modelleri için yeniden ayarlanabilir
  (şu an v1 modelleri için seçilmiş 3 g / 50 m kullanılıyor).
