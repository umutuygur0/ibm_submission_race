# Blok 8 — 2026-10-15 04:15 → 08:30: hızlı modelleri stabil hale getirmek

Başlangıç: önerilen hibrit res3a 64,878 (60/60). En hızlı planlayıcı t4off 64,722 ama 10 ms ×30'da 26/30 (gecikmeye hassas).

| Saat | İş |
|---|---|
| 04:15–06:00 | t5rob: t4off'tan sıkı geçerlilikli hedefli arama (det + delay1 + 3 kaçırma + **2 gecikme+kaçırma** turu, hepsi hasarsız) |
| 06:00–06:20 | Doğrulama: run_benchmark ×30 + stres 30+30 |
| 06:20–08:00 | En stabil planlayıcı + hibrit RL (8 işlem) |
| 08:00–08:30 | RL değerlendirme, final doğrulama, rapor |

## Günlük
- 04:15 `search.py`: `_lagrob` (gecikme + başta kaçırma + rastgele kaçırma turları), `targeted.py --lagrob`. t5rob başladı.
- 06:47 Bilgisayar 04:1x–06:44 arası askıya alınmış (t5rob süreci 06:44:47'de oluşturulmuş; keep_awake çalışıyordu ama kapak/uyku tuşu
  kaynaklı askıya almayı engellemez). Yeni plan: t5rob 07:45'e kadar → 07:45–08:10 doğrulama → 08:20 rapor. RL bu turda yok.
- 06:47 t5rob başlangıç: t4off sıkı geçerlilikte de geçerli (det 64,722, delay1 65,524).
- 07:10 t5rob bitti: iyileşme yok (64,722; t4off sıkı geçerliliği şans eseri geçiyor). 07:12 güvenlik payı taraması (hız ×0,995–0,98, fren ×0,95–0,90), 20+20 stres: gecikmede hep 16–17/20, fren düşürmek kötüleştiriyor → t4off'un gecikme hassasiyeti yapısal.
- 07:13 Hibrit res3a uzatılmış stres (res_eval, 15+15): gecikme **15/15** (ort 65,15), kaçırma **15/15** (ort 64,88). Önceki 10+10 ile
  toplam 25/25 + 25/25; run_benchmark 60/60. → **res3a en hızlı ve stabil model.** Kalan sürede yeni RL eğitimi anlamlı değil (≥ 2 sa gerekir).
