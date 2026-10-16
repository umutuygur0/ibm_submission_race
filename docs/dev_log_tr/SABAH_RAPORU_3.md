# Sabah raporu 3 — gece çalışması 3 (2026-10-07 21:35 → 2026-10-08 07:20)

Önceki raporlar korunuyor: SABAH_RAPORU.md, SABAH_RAPORU_2.md. Saat saat günlük: [GECE3_PLANI.md](GECE3_PLANI.md).

## Sonuç
**Önerilen teslim dosyası: `racelab/submission/torcs_jm_par_rl_s47_assist.py`**
= gece bulunan yeni model (n3b_s47) + **düzlük gaz yardımcısı** (önü ≥180 m açık, araç düz, pistin içinde → tam gaz)
+ **vites 18650 rpm**. Hiçbir eski dosyanın üzerine yazılmadı.

| Teslim dosyası (`racelab/submission/`) | Corkscrew deterministik (= GUI) | 10 ms bekleme ×10 | Not |
|---|---|---|---|
| **`torcs_jm_par_rl_s47_assist.py`** | **78,158 s, 0 hasar** | **10/10 hasarsız** (ort ~80,1) | **önerilen** |
| `torcs_jm_par_rl_v6g_assist.py` | 78,934 s, 0 hasar, max 267,9 | 10/10 hasarsız (ort ~79,7) | yedek (adım kaçınca daha az kayıp) |
| `torcs_jm_par_rl_v6g.py` (dün, GUI 01:19:79) | 79,796 s, 0 hasar | 10/10 | önceki |

Dünkü GUI sonucuna (79,79 s) göre beklenen kazanç **~1,6 s**. GUI testi (Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race):
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_s47_assist.py --mode gui --label "SUBMISSION_s47_assist@GUI"
```
Beklenen: ~78,16 s (TORCS ekranında 01:18:16 civarı), 0 hasar. Farklı çıkarsa yedek `v6g_assist` (~78,93 s).

## Gecenin bulguları
1. **Analiz:** v6g düzlükte (toplam 15 s) gaz pedalını ort. %69 kullanıyordu, tam gaz %5. TCS artık kısıt değil (%14);
   asıl kısıt politikanın temkinliliği.
2. **Eğitimsiz kural kazançları (çalıştı):** düzlük yardımcısı ≥180 m −0,39 s; vites 18500/18650 −0,25 s; birlikte **−0,87 s**.
   ≥160 m yardımcı deterministikte 78,27 s ama 10 ms testinde 1/8 → kırılgan, kullanılmadı.
3. **İşe yaramayanlar:**
   - Yardımcıyla birlikte eğitim (v8a-d): politika yardımcı anlarında gaz sinyali alamıyor; modeller daha yavaş/kırılgan.
   - v6g'den devam (v9a-d, aktör dondur + critic ısıt): ısınmadan hemen sonra çöküyor — tek politikanın verisiyle
     eğitilen critic, dağılım dışında yanlış yön gösteriyor (sabit alpha ile de aynı).
   - ABS eşiği 3/4/6/kapalı: hepsi daha yavaş (2 = mevcut en iyi).
   - Tohum hattı 1. tur (tohum 40-44): en iyi 79,58 s (yardımcıyla).
4. **Kazanan:** tohum hattı 2. tur (n3b, tohum 45-48) — **tohum 47**: yardımcısız 79,98 s (v6g'den yavaş) ama yardımcıyla
   **78,16 s**; yardımcıyla uyumu daha iyi. Toplam 9 tohumdan 1'i şampiyonu geçti.

## Notlar
- Tohum hattı sonuçları: runs/n3_pipeline.csv, runs/n3b_pipeline.csv; hızlı deneyler: runs/quick_exp.csv, quick_exp_v2.csv.
- select_models'in 10 ms testi kuyruk boşaltmadan çalıştığı için yardımcılı modellerde gerçeğinden sert (s47 13/20,
  v6g 14/20); teslim dosyası (kuyruk boşaltmalı) ile run_benchmark testi esas alındı (ikisi de 10/10).
- Gece sonunda açık eğitim/TORCS süreci yok.
