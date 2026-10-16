# Blok 2 raporu — 2026-10-10 20:02 → 2026-10-11 08:00

Plan ve saat saat günlük: [BLOK2_PLANI.md](BLOK2_PLANI.md). Eski dosyaların üzerine yazılmadı.

## Sonuç
**Önerilen teslim dosyası: `racelab/submission/torcs_jm_par_rl_v10d_dist_r02.py`** (Blok 2'nin ilk saatinde üretildi)

| Teslim dosyası | Deterministik (= GUI, adım kaçmazsa) | 10 ms bekleme, eğitim yükü altında ×20 | Not |
|---|---|---|---|
| **`torcs_jm_par_rl_v10d_dist_r02.py`** (4 g / 20 m + 0,2 s tepki payı) | **73,098 s**, 0 hasar | **20/20 hasarsız**, ort 73,62 | **önerilen** |
| `torcs_jm_par_rl_v10d_dist_a4m20.py` (Blok 1) | 72,940 s | 13/20 | en hızlı ama adım kaçınca hasar |
| `torcs_jm_par_rl_v10d_dist.py` (önceki güvenli) | 73,582 s | 20/20, ort 74,11 | yedek |

Bu gece eğitilen 11 yeni modelin hiçbiri bu dosyayı geçemedi (ayrıntı aşağıda).

GUI testi (Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race):
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v10d_dist_r02.py --mode gui --label "r02@GUI"
```
Beklenen: TORCS ekranında ~01:13:10, 0 hasar.

## Ne değişti: gecikmeye dayanıklı yardımcı (r02)
Düzlük yardımcısı (tam gaz kararı) artık fren mesafesine **hız × 0,2 s** ek pay koyuyor. Böylece TORCS bir adımı kaçırıp
araba birkaç ms geç tepki verse bile fren noktası kaçmıyor. Sonuç: Blok 1'in hızlı ama kırılgan 4 g / 20 m ayarının
hızı (72,94 → 73,10) neredeyse korunurken sağlamlık 13/20'den **20/20**'ye çıktı (10+ adım kaçan koşuların hepsi temiz).

## Bu gecenin eğitimleri
| Eğitim | Ne | Yardımcısız en iyi | Yardımcıyla en iyi (det / ×8 temiz) |
|---|---|---|---|
| b2a (4 sa → 3,2 sa'te plato) | standart v10, uzun | 75,11 | 73,73 (model başına geniş yardımcı taraması) |
| b2b (D1, 4 sa → plato) | konum harmonikleri k=64'e kadar | 76,27 | — (standart v10'un gerisinde, bırakıldı) |
| T1 b2s80 / b2s81 (2 sa) | v10d tarifi, yeni tohumlar | 74,83 / 75,23 | 73,39 (3/8) / 73,52 (1/8) |
| T2 b2s82 / b2s83 (2 sa) | aynı | 75,14 / 75,56 | 73,58 (6/8), 73,86 (8/8) / 74,79 (8/8) |
| T3 b2s84 / b2s85 (2 sa) | aynı | 82,42 (60 dk'da durduruldu) / 75,85 | — / 74,18 (2/8) |
| b2s90 (54 dk) | boşalan portlarda ek tohum | 76,80 | 74,52 (1/8) |
| T4 b2s86 / b2s87 (1,5 sa) | aynı | 76,06 / 75,79 | 74,27 (3/8) / 73,68 (8/8, ort 76,02) |

(Karşılaştırma: şampiyon v10d + r02 aynı ölçüm aracında 73,07 det.)

## Bulgular
1. **Yardımcısız politika iyileşiyor ama yardımcıyla birleşince değil.** Birçok yeni model yardımcısız v10d'den
   (76,26 ort) daha iyi (ör. b2s80 74,83), ama yardımcı eklendiğinde 73,4–74,8'de kalıyor ve çoğu zorlamalı
   testte hasar alıyor. v10d'nin yardımcıyla bu kadar uyumlu olması tesadüfi görünüyor (bu gece 11 eğitimde tekrarlanamadı).
2. **D1 (daha ince konum çözünürlüğü) işe yaramadı:** standart v10'dan ~1 s yavaş.
3. **Uzun eğitim plato yapıyor** (~2–3 saatten sonra iyileşme yok, geç dönemde bozulma). 2 saatlik turlar daha verimli.
4. Tohum varyansı büyük: tohumlardan biri (b2s84) hiç öğrenemedi.

## Blok 3 için öneriler
- Tohum taramasından daha fazla kazanç beklenmiyor. Denenmemiş tek mantıklı adım: **v10d'den başlayıp yardımcı açıkken
  düşük öğrenme oranıyla kısa ince ayar** — politika yardımcıya uyum sağlasın. Risk: v8 (yardımcı eğitimde, sıfırdan) ve
  n5 (eğitilmiş modelden devam) daha önce işe yaramadı; kazanç belirsiz, 2–4 saatlik deneme.
- Ya da süre yeterliyse kodu dondurup **teslim hazırlığı** (temiz ayrı repo, basit livery, video). 73,10 s TORCS
  botlarının (74,2 s) 1,1 s önünde.
