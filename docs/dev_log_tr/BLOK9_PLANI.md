# Blok 9 — 2026-10-15 18:12 → 2026-10-16 09:00: en hızlı deterministik tur (tek şart: hasar yok) + drift denemesi

Kullanıcı kararı: gecikme sağlamlığı birincil kriter değil (GUI = deterministik koşu); **hedef hasarsız en hızlı deterministik tur**.
Pist dışı serbest. Ayrıca: kayarak (drift) daha hızlı dönmek mümkün mü?

| Saat | İş |
|---|---|
| 18:20–02:00 | g6det: 8 adalı evrim (yalnız det tur, hasarsız; pist sınırı serbest). Adalar: t4off, t3off, gr3L, drift adaları (gidiş yönü kontrolü kayma sınırı 0,1/0,2/0,15; patinaj eşiği 10–12; viraj hızları ×1,03) |
| 02:00–06:00 | En iyi üzerinde hedefli ince ayar + ikinci evrim turu |
| 06:00–08:00 | Ek ince ayar / gerekirse hibrit RL |
| 08:00–09:00 | Final: det tekrarlanabilirlik + run_benchmark ×30 (bilgi), teslim dosyaları, rapor |

## Günlük
- 18:20 evo2'ye `--no-delay1`, `--offstop`. keep_awake 15,5 sa. g6det başladı (8 işlem, idx 1–8).
