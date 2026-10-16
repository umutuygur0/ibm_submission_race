# Blok 6 — 2026-10-14 11:27 →: küçük düzeltmeler + artık (residual) RL

Başlangıç: önerilen `submission/torcs_jm_par_line_gr3L.py` 64,964 s (run_benchmark ×30 30/30; stres: gecikme 30/30, kaçırma 30/30).

| Saat | İş |
|---|---|
| 11:28–11:45 | **Aşama A — küçük düzeltmeler** (`planner/targeted.py`: hedefli koordinat araması, toplam tur süresi, stres geçerliliği) |
| 11:47–15:47 | **Aşama B — artık RL** (`rl/residual_env.py`, `rl/train_residual.py`): gr3L temel sürücü + SAC küçük düzeltme (±0,08 direksiyon, ±0,15 pedal), 8 paralel, 4 saat |
| 15:50–16:40 | RL adaylarının seçimi (det + gecikme + kaçırma), RL'li teslim dosyası oluşturucu, ×30 ve stres doğrulaması |
| ~16:45 | Rapor (BLOK6_RAPORU.md) |

## Günlük
- 11:28 Kontrol: son düzlükte hız kesmiyoruz (3503 m: 249/250 km/h, 3596 m: 271/272; bitişe 64,80 vs rakip 65,46). GUI'de görülen
  yavaşlama büyük olasılıkla bitiş çizgisinden sonra (yarış kapanınca).
- 11:34 t1 (kalkış + 763 + 1523 virajları): 64,964 → **64,874**. 11:39 t2 (kalan virajlar): **64,840**.
- 11:42 Doğrulama: t2 run_benchmark ×30 **30/30** (zaman aşımı ort 2,9), ama stres: gecikme **12/30**, kaçırma 27/30.
  t1: run_benchmark 30/30, stres gecikme 20/30, kaçırma 30/30. → Küçük düzeltmeler 0,09–0,12 s kazandırıyor, kuralı geçiyor,
  ama gecikme sağlamlığı gr3L'den düşük. Karar: gr3L ana aday; t2 (64,840) daha hızlı alternatif.
- 11:46 Artık RL ortamı sınaması: sıfır düzeltme 64,956 (gr3L ile aynı), rastgele düzeltme (std 0,2) 65,12 hasarsız.
  res1 başladı (8 işçi, 4 sa, bölümlerin %30'u kalıcı gecikmeli, %2 adım kaçırma).
- 11:56 res1 10 dk: ilk dakikalarda bölümler pistte bitiyordu, toparlandı: lap20 %95. En iyi eval hâlâ 64,956 (temel).
- 12:16 res1 31 dk: lap20 %90–95; adaylar 65,19–65,25 (temel 64,956'nın gerisinde).
- 12:37 res1 51 dk: lap20 %95, en iyi RL adayı 65,18 (temel 64,956 geçilmedi).
- 12:57 res1 72 dk: en iyi RL adayı 65,07 (yavaş iyileşme; temel 64,956).
- 13:17 res1 92 dk: en iyi RL adayı 65,07 (20 dk'dır değişmedi).
- 13:38 res1 durduruldu (112 dk, en iyi RL 65,07 > temel 64,956; 40 dk değişmedi). res2: c_act 0,002, d_steer 0,05, d_pedal 0,25, lr 1e-4, ~15:50'ye kadar.
- 13:40 res2 başladı. 12. dakikada en iyi 65,55; sonrasında öğrenici takıldı (GPU %0, 31 adım/sn). 14:48 durduruldu.
- 14:50 **Sonuç: artık RL bu sürede temel sürücüyü geçemedi** (res1 2 sa: en iyi 65,07; res2: 65,55; temel 64,956).
  Açık kalan tek TORCS argümansız (kullanıcının GUI'si) → dokunulmadı.
- 15:36 res2 bilgisayar uykuya geçince durmuştu (günlükte 12,7 → 20 → 55 dk boşluğu). tools/keep_awake.py (SetThreadExecutionState, güç ayarı değişmez) + res3 (res2 ayarları, 2,2 sa, ~17:50 bitiş).
- 16:38 Bilgisayar şarjda değildi (işlemci yavaşlamış: 8.000 → 750 adım/sn); şarja takılınca normale döndü. Kullanıcı: 18:30'a kadar devam.
  res3 63 dk: en iyi 65,002 (temel 64,956). `rl/res_eval.py` (det / gecikme / kaçırma): v04018 det 65,002, gecikme 65,24
  (gr3L ~65,58) → RL gecikmeyi telafi etmeyi öğreniyor. `planner/build_res_submission.py` (hibrit teslim: çizgi + gömülü aktör):
  eşdeğerlik birebir (65,002), ama det turda pist dışı 6 → ödüle ayarlanabilir kenar cezası (`--c-off`) eklendi.
  Plan: res3 ~17:48 biter → devam eğitimi (final_full'dan, c_off 5) 18:15'e kadar → 18:15–18:30 değerlendirme.
- 17:48 res3 bitti: en iyi adaylar 64,88 / 64,90 / 64,94 (temel 64,956'yı geçti). res3b (devam, c_off 5) başladı.
- 17:54 res_eval (10'ar tur): **v05776: det 64,878, gecikme 10/10 ort 65,15, kaçırma 10/10 ort 64,88**, |pos| ≤ 1,035;
  v06233 det 64,938, 10/10 + 10/10; v05809 det'te hasar (elendi).
- 17:59 Hibrit teslim `submission/torcs_jm_par_hybrid_res3a.py` (gr3L + v05776): eşdeğerlik det 64,878. res3b eğitim yükü altında ×30
  (port 3010): hibrit 27/30 (ort 65,07, zaman aşımı 14,6) vs gr3L aynı yükte 26/30 (ort 65,29, zaman aşımı 18,8) → yük altında hibrit
  daha iyi. Kesin karar sistem boşken (18:15 sonrası).
- 18:15 res3b bitti: en iyi 65,14 (c_off 5 ile geriledi) → kullanılmıyor.
- 18:18 **Final (sistem boş, port 3010, 30×10 ms):** hibrit res3a det **64,878**, **30/30 hasarsız**, ort **65,16**, pist içi 0/30
  (det turda 7 küçük taşma); gr3L det 64,964, 30/30, ort 65,55. Aktör ileri geçişi: numpy ~0 ms, saf Python 3,7 ms (< 10 ms).
