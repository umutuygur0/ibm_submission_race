# Gece planı — 2026-10-05 22:40 → 2026-10-06 07:00 (otonom)

Kurallar: mevcut dosyalara (torcs/, gym_torcs/, benchmark agent'ları) dokunulmaz; paket kurulmaz;
elle açılmış GUI TORCS'a dokunulmaz; her şey racelab/ altında yeni dosyalarla. Sabah raporu: `SABAH_RAPORU.md`.

| Saat (yaklaşık) | Faz | İş |
|---|---|---|
| 22:40–23:20 | A | **v4 ortamı:** sert direksiyon sınırı yok; zikzak ödülle cezalandırılır (hıza bağlı Δsteer, yön değiştirme, düzde büyük direksiyon). Kısa test: 450 m ve 2470 m virajları geçiliyor mu. |
| 23:20 | B | **3 saatlik eğitim:** v4a (normal ceza) idx 1-4 + v4b (güçlü ceza) idx 5-8, ikisi de v2_ft/final_full.pt'den. |
| 23:20–02:20 | B' | Eğitim sürerken: **teslim dosyası** (torcs_jm_par.py iskeleti + numpy'ye gömülü v2 modeli, PyTorch'suz), doğrulama: benchmark'ta 85,736 s çıkmalı. |
| 02:20–02:50 | C | Aday seçimi (select_models), en iyiler run_benchmark ile skor tablosuna, zikzak ölçümü (hız>150 yön değişimi/s). |
| 02:50–06:10 | D | Karar: v4 > v2 ise en iyi v4 varyantından 3+ saat devam (8 worker). Değilse alternatif (v2'den düşük lr ince ayar / farklı ceza). |
| 06:10–06:50 | E | Son seçim, skor tablosu, teslim dosyasını şampiyonla güncelle, `SABAH_RAPORU.md`. |

Başarı ölçütü (sırasıyla): GUI zamanlaması ×10'da hasarsız bitirme oranı → ortalama tur süresi.
Şampiyon (başlangıç): v2_ft_v05581 + sac_agent_v2 → 85,736 s, 0 hasar, 9/10 hasarsız.

## Gece günlüğü
- 22:45 v4a + v4b başladı (3 saat). Teslim dosyası hazırlandı ve doğrulandı (85,704 s, 0 hasar).
- 23:01 v4a ve v4b ~15. dakikada çöktü (her şey 460 m'de pist dışı); adaylar kayıtlı.
  Erken seçim (×6, yük altında): v4b/v00407 → 5/6 hasarsız, ort 82,73 s.
- 23:26 v4b kendiliğinden toparlandı (lap20 %45). v4a 25 dk toparlanmadı → STOP.
  Yerine **v4c** (v4b ayarı + lr 1e-4, daha sakin ince ayar), idx 1-4, 2,25 saat (~01:45 biter).
- 00:14 Kontrol (aynı yük, ×6): v2 şampiyon 1/6 hasarsız (105 s) vs v4b/v00513 3/6 (83,2 s) → yük altındaki
  düşük oranlar ortamdan; v4b adayları v2'den hem hızlı hem sağlam. v4b lap20 %95 (123 aday).
  v4c (lr 1e-4) 30 dk 460 m'de takılı → STOP. Yerine v4d = v4b ayarı, seed 1 (çeşitlilik), ~01:45'e kadar.
  Çöküş nedeni tahmini: ödül değişince eski critic uyumsuz. Faz D: v4b final_full'dan AYNI ödülle devam (çöküş beklenmez).
- 01:45 v4b (3 saat, 436 aday) ve v4d (1,45 saat, 145 aday) bitti. CPU boşken seçim:
  tur 1 (×10): v4d'den 4 model 10/10. Final (×20, kontrol v2): **v4d/v02537 20/20 hasarsız, ort 82,71 s**
  (v2 aynı koşulda 1/20). Resmi benchmark: deterministik **81,904 s / 0 hasar**, GUI zaman. ×10: 10/10 hasarsız
  (82,5–83,0 s, 2–18 kaçan adıma rağmen). Zikzak 8,6→0,7/s, max hız 231→251 km/h.
- 01:57 Teslim dosyası yeni şampiyonla yeniden üretildi (submission/torcs_jm_par_rl.py; eski: _rl_v2.py):
  numpy 81,736 s / 0 hasar, ×10 10/10; saf Python ×5 5/5 hasarsız (150+ kaçan adıma rağmen).
- ~02:00 Faz D (4 saat, ~06:00 biter): v5a = v4d/final_full'dan aynı ödül (seed 2, idx 1-4);
  v5b = v4d'den c_speed 0.4 (seed 3, idx 5-8).
- 03:17 v5a/v5b plato: 80 dk'da eval en iyi 82,15–82,80 s (şampiyon 81,90 s'yi geçen yok); v5b lap20 %5'e düştü → STOP.
  Yerine **v5c** = v4d/final_full'dan gamma 0.998 (daha uzun ufuk; en büyük sıçrama g990→g995 gamma'dan gelmişti),
  c_speed 0.25, idx 5-8, ~06:00'ya kadar. v5a kontrol olarak devam.
- 04:38 v5a 160. dk'da çöktü (460 m, 40 dk yeni aday yok) → STOP; adayları (369) erken seçimde. v5c sağlıklı (lap20 %80).
- 04:47 v5a seçimi: en iyi 9/10 ama ort 100,9 s → şampiyonu geçmiyor (uzun devam hızlandırmadı). Boş idx 1-4'te **v4e** = şampiyonu üreten v4d tarifi (v2_ft'den, c_speed .25), seed 5, ~06:00'ya kadar.
- 05:28 v4e (v4d tarifi, seed 5) 40. dk'da 80,31 s adaylar. 06:00 v4e + v5c bitti.
- 06:04 Seçim: v4e'den 4 aday 10/10 (ort 81,2–82,2). Final ×20: **şampiyon v4d 20/20 (ort 82,52)**;
  v4e adayları 15–18/20 (ort 81,1–81,9) → daha hızlı ama riskli. Şampiyon değişmedi (toplam 40/40 hasarsız).
  v4e_v00248 resmi: 80,668 s / 0 hasar; GUI zaman. ×10 8/10 → submission/torcs_jm_par_rl_fast_risky.py (alternatif).
- 06:10 v4f (v4d tarifi, seed 6, 8 worker, 45 dk) başladı → 06:55 biter, adayları sabah değerlendirilecek.
- Hash kontrolü: torcs/ + gym_torcs/ aynı; sadece quickrace.xml 5 Ekim 15:49'da TORCS menüsünce (kullanıcı GUI testi) güncellenmiş.
- 06:55 v4f seçimi: en iyi 9/10, ort 82,75 s → şampiyon değişmedi. Gece tamamlandı; açık süreç yok.
