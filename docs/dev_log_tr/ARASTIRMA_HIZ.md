# Hız araştırması — 2026-10-08 (gündüz)

Soru: Corkscrew'da daha hızlı nasıl bitiririz, ne kadar mümkün?

## 1. Ne kadar mümkün? (referans: TORCS'un yerleşik botları, aynı araç car1-ow1)
| Sürücü | Tur (Corkscrew, 1 tur, duran kalkış) |
|---|---|
| olethros | **74,20 s** |
| inferno / lliaw / tita | 74,51 s |
| Bizim (GUI'de doğrulandı, s47_assist) | 78,15 s |

Botların setup dosyaları (kanat 13°/10°, fren dengesi 0,47, vites oranları) araç varsayılanıyla **aynı** → fark araç
ayarından değil, sürüşten. Aynı araçla ~4 s daha hızlı tur mümkün. (configs/ref/ref_*.xml, `wtorcs -r`)

## 2. Nerede kaybediyoruz? (analysis/track_profile.py, s47_assist GUI koşusu)
- Pist: 66 segment, 17 viraj; en dar virajlar #13 (R≈12 m, corkscrew), #17 (R≈18 m), #3 (R≈24 m).
- **Tam gaz sadece turun %10'u, fren %16, ne gaz ne fren %11 (~8,6 s sürükleniyor)**; kalan zaman kısmi gaz.
- Bazı virajlarda fren 200 m+ önce başlıyor (#3, #4, #7, #10, #14).
- (Merkez hattıyla hesaplanan yanal ivme şikanlarda abartılı çıkıyor; yarış çizgisi virajı genişletiyor.)

## 3. Kısa testler (quick_exp.py, sonuçlar runs/quick_exp_v3.csv)
| Deney | Sonuç |
|---|---|
| Pedal güçlendirme ×1,2 / ×1,5 | ❌ 590 m'de çarpıyor |
| **Mesafe tabanlı gaz yardımcısı** (önü ≥ v²/(2·a·g) + pay → tam gaz) | **✅ −1,0 s'ye kadar** |
| a=2,0 g pay 30/40/50/60 m | 77,46 / 77,59 / 77,80 / 77,77 |
| a=2,5 g pay 40/60 | 77,39 / 77,67 |
| a=3,0 g pay 50/70 | 77,42 / 77,60 |
| Sağlamlık ×8 (10 ms): **3,0 g + 50 m 8/8 (ort 78,43)**, 2,0g+30 6/8, 2,0g+40 7/8, 2,5g+40 5/8 | |
| Aynı yardımcı v6g ile | ❌ 476 m'de pist dışı → fayda modele bağlı |

## 4. Yeni teslim dosyası
**`submission/torcs_jm_par_rl_s47_dist.py`** = n3b_s47 + mesafe yardımcısı (3,0 g, 50 m) + vites 18650:
deterministik **77,436 s / 0 hasar / max 260,7 km/h**; 10 ms ×10 **10/10 hasarsız** (77,8–79,2).
Önceki (GUI 78,15 s) → beklenen kazanç ~0,7 s. Hiçbir eski dosyanın üzerine yazılmadı.

## 5. Uzun süreç (başladı)
Gece hattı (night_pipeline.py, önek n4): her tohum 45 dk eğitim → seçim → en iyi 3 aday × 4 yardımcı varyantı
{yok, sabit 180 m, mesafe 3,0g+50m, mesafe 2,0g+30m} deterministik + ×8. Amaç: yardımcıyla en iyi uyuşan modeli bulmak.
Sonuçlar: runs/n4_pipeline.csv. Durdurmak için: racelab/rl/runs/n4_STOP dosyası oluştur.

## Uzun süreç günlüğü (n4)
- 14:22 n4 hattı başladı (ilk deneme --until hesap hatasıyla anında kapandı; düzeltildi). Son döngü başlangıcı 09.10 06:30.
- 15:14 n4_s60: en iyi det 79,27 (mesafe 3,0g+50m, 4/8). Şampiyon (77,44) geçilmedi.
- 16:09 n4_s61: çok sağlam (seçim 10/10, ×8 8/8, adım kaçsa da süre aynı) ama yavaş: en iyi 79,35 (sabit 180). Şampiyon geçilmedi.
- 16:20 Kullanıcı isteğiyle güvenli durduruldu: n4_s62 eğitimi STOP ile kapandı (final.pt + 33 aday kayıtlı); hat ve seçim alt süreçleri kapatıldı (n4_s62 seçimi/yardımcı değerlendirmesi YAPILMADI). Açık süreç yok (sadece kullanıcının GUI TORCS'u).
  Devam komutu: python racelab/rl/night_pipeline.py --prefix n4 --start-seed 63 --minutes 45 --until 06:30
  n4_s62'yi değerlendirmek için: python racelab/rl/select_models.py --run n4_s62 --env v4 --tcs-th 5.0 --top 12 --repeat 10 --procs 4 --idx-from 1
