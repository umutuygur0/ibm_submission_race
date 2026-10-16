# Blok 7 — 2026-10-14 23:21 →: pist sınırı serbest (sadece hasar) + uzun hibrit RL

**Kural kontrolü (toplantı kayıtları + öğrenci paketi):** ölçüt "temiz tur = başında ve sonunda hasar sıfır olan tamamlanmış zamanlı tur"
+ tur süresi; eğitmen: "tek bir hasar bile varsa başarısız". Pist dışına çıkma ile ilgili kural **yok** → |trackPos| kısıtı kaldırıldı,
yalnızca hasarsızlık (+ gecikme/kaçırma stres turları) zorunlu.

| Aşama | İş |
|---|---|
| 1 (~1,5 sa) | `targeted.py` gevşek sınırla (det/stres |pos| ≤ 1,5, tur 1,6'da durur), gr3L'den, 11 viraj × (hız, fren, ön besleme, apeks, yön kazancı) |
| 2 (gece) | En iyi yeni sürücü + hibrit RL (pist dışı cezası 0, sadece hasar), uzun eğitim, `keep_awake` açık |

## Günlük
- 23:21 Başlangıç. `search.py` tur durdurma sınırı ayarlanabilir (`_offstop`), `targeted.py --offstop`. keep_awake 9 sa.
- 23:24 t3off başladı.
- 23:33 t3off bitti: det **64,730** (gr3L 64,964). Doğrulama: run_benchmark ×30 30/30 (ama 0 zaman aşımı → gecikme sınanmadı);
  stres: gecikme 26/30, kaçırma 27/30 → kırılgan.
- Plan (kullanıcı: 05:30'da bitsin): 23:37–00:25 t4off (t3off'tan, daha sıkı: gecikme + 4 kaçırma turu) → 00:30–04:00 hibrit RL res4
  (pist dışı cezası 0, bölüm sonu |pos| 1,6) → 04:00–04:45 RL değerlendirme + hibrit teslim → 04:45–05:20 final ×30 + stres → 05:30 rapor.
- 23:50 t4off bitti: det **64,722** (sıkı geçerlilik: gecikme + 4 kaçırma). Stres: gecikme 24/30, kaçırma 26/30 → hâlâ kırılgan.
  Gevşek sınırlı planlayıcı çözümleri hızlı ama gecikmeye hassas → RL'nin telafi etmesi hedefleniyor.
- 23:53 İki paralel RL (04:00'e kadar, 4'er işçi; pist dışı cezası 0, bölüm sonu |pos| 1,6):
  res4a temel t4off (64,722, kırılgan) — RL'nin sağlamlık kazandırması; res4b temel gr3L (sağlam), res3'ün son halinden devam.
- 00:19 28 dk: res4a lap20 %75, en iyi 65,47; res4b lap20 %15 (dalgalanma), en iyi 65,28.
- 00:45 53 dk: res4a lap20 %95, en iyi 65,34; res4b lap20 %90, en iyi 65,28.
- 01:10 78 dk: res4a lap20 %0 (çöküş), en iyi 65,30; res4b lap20 %95, en iyi 65,28 (25 dk sabit). Paralel → her biri res3'ün yarı hızında.
- 01:36 res4a durduruldu (25 dk çöküş, lap20 %0). Yerine res5: res3 tarifi (temel gr3L, c_off 0,5, off_end 1,2) yeni tohum, idx 1–4, 04:00'e kadar.
- 02:01 res5 25 dk: lap20 %95, en iyi 65,40; res4b 129 dk: lap20 %100, en iyi 65,28 (~1 sa sabit).
- 02:27 res5 50 dk: **64,946** (gr3L'i geçti), lap20 %100. res4b durduruldu (65,28'de 1,5 sa sabit) → res6 (res3 tarifi, tohum 6, idx 5–8) 04:00'e kadar.
- 02:52 res5 76 dk: en iyi 64,946 (lap20 %100); res6 25 dk: lap20 %0 (ısınma).
- 03:18 res5 101 dk: 64,946 (~50 dk sabit); res6 51 dk: lap20 %100, 64,96.
- 04:00 res5/res6 bitti: res5 en iyi 64,936–64,962 (gecikme 9–10/10), res6 temeli geçemedi → res3a'yı (64,878) geçen yok.
  res3a aktörü t4off/t3off temelinde: det ve kaçırmada hasar → uyumsuz, bırakıldı.
- 04:09 **Final (port 3010, 30×10 ms):** t4off det 64,722 → **26/30** ✗; t3off 64,730 → **23/30** ✗;
  hibrit res3a det 64,878 → **30/30** (toplam 60/60), ort 65,13 ✓.
