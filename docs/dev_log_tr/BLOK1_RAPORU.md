# Blok 1 raporu — 2026-10-10 00:40 → 05:20

Plan ve saat saat günlük: [BLOK1_PLANI.md](BLOK1_PLANI.md). Eski dosyaların üzerine yazılmadı.

## Sonuç
| Teslim dosyası (`racelab/submission/`) | Deterministik (= GUI, adım kaçmazsa) | 10 ms bekleme, CPU boş | Öneri |
|---|---|---|---|
| `torcs_jm_par_rl_v10d_dist.py` (önceki, 3 g / 50 m) | 73,582 s, 0 hasar | **20/20 hasarsız**, ort 74,46 | **güvenli teslim** |
| **`torcs_jm_par_rl_v10d_dist_a4m20.py`** (yeni, 4 g / 20 m) | **72,940 s**, 0 hasar, max 282 km/h | 11/20 ve 14/20 hasarsız | en hızlı, ama adım kaçmalarına hassas |

**Önemli bulgu:** 4 g / 20 m'de başarısız koşuların hepsinde 10+ adım kaçmış (TORCS'un 10 ms bekleme süresi aşılmış);
≤5 adım kaçan koşuların tamamı hasarsız bitti. Gerçek GUI testlerimizde süre deterministik sonuca birebir eşitti
(adım kaçmıyor) → 4 g / 20 m'nin GUI'de 72,94 s yapması muhtemel, ama hata payı küçük.

**Öneri:** GUI'de ikisini de dene. 4 g / 20 m birkaç denemede hasarsız ~01:12:94 verirse videoda onu kullan; aksi halde
3 g / 50 m (01:13:58).
```
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v10d_dist_a4m20.py --mode gui --label "a4m20@GUI"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v10d_dist.py --mode gui --label "v10d_dist@GUI"
```

## Bulgular
1. **Yardımcı ayarı (eğitimsiz):** v10d için kısa pay belirgin daha hızlı (3,5–4 g, 10–20 m bölgesi ~73,0 s; önceki 3 g / 50 m 73,44 s,
   aynı ölçüm aracıyla). Daha agresif ayarlar (5–6 g) daha yavaş/riskli → optimum bulundu.
2. **Uzun eğitim (4,4 saat, b1a/b1b):** politika 2 saatten sonra da iyileşti (b1a yardımcısız eval 75,63 → 74,87 s; seçimde
   10/10 ort 75,76 — v10d modelinin 76,26'sından iyi). 3,8 saatten sonra b1a bozulmaya başladı (adaylar kayıtlıydı).
3. **Ama yardımcıyla birleşince şampiyonu geçemedi:** b1a en iyi 73,92 s (yardımcıyla); b1b modelleri çoğu yardımcı ayarıyla
   bitiremedi. Yardımcının faydası modele çok bağlı; v10d bu yardımcıyla olağanüstü uyumlu.

## Blok 2 için öneriler
- **Yardımcıyı adım kaçmalarına dayanıklı yapmak** (ör. yardımcı sadece gecikme yokken devreye girsin ya da pay hıza göre
  büyüsün) → 4 g / 20 m hızını 3 g / 50 m sağlamlığıyla birleştirmek. Eğitim gerektirmez, 1–2 saat.
- **b1a modelleri için yardımcıyı ayrıca taramak** (yardımcısız daha iyi oldukları için doğru ayarla şampiyonu geçebilirler).
- D1: daha ince konum çözünürlüğü (2 tohum, ~4 saat).
