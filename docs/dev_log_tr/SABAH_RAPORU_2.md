# Sabah raporu 2 — gece çalışması 2 (2026-10-07 00:55 → 08:00)

Önceki rapor korunuyor: [SABAH_RAPORU.md](SABAH_RAPORU.md). Saat saat günlük: [GECE2_PLANI.md](GECE2_PLANI.md).
Skor tablosu: [results/LAP_LOG.md](results/LAP_LOG.md)

## Sonuç
Düzlük hızlanması ve viraj çizgisi iyileştirildi. **Yeni önerilen teslim: `racelab/submission/torcs_jm_par_rl_v6g.py`**
→ Corkscrew **79,80 s, 0 hasar, max 264 km/h**; zamanlama testlerinde **20/20 ve 10/10 hasarsız**.
Önceki şampiyon 81,73 s idi (gerçek GUI'de doğrulanmış) → **~1,9 s daha hızlı ve en az onun kadar güvenilir.**
Hiçbir eski dosyanın üzerine yazılmadı.

| Teslim dosyası (`racelab/submission/`) | Corkscrew (deterministik = GUI) | 10 ms bekleme ×10 | Öneri |
|---|---|---|---|
| **`torcs_jm_par_rl_v6g.py`** | **79,796 s, 0 hasar**, max 264,2 | **10/10 hasarsız** (79,8–80,7 s) | **Video + teslim için önerilen** |
| `torcs_jm_par_rl_v6f_fast.py` | 79,806 s, 0 hasar | 4/10 bitiş (hasar yok) | v6g ile aynı hızda ama kırılgan → gerek yok |
| `torcs_jm_par_rl_v6c_safe.py` | 81,352 s, 0 hasar | 10/10 hasarsız | yedek |
| `torcs_jm_par_rl.py` (önceki, değişmedi) | 81,736 s, 0 hasar (GUI'de 81,73 doğrulandı) | 10/10 hasarsız | kanıtlanmış yedek |

Final ×20 (aynı koşul): v6g/best **20/20**, ort 80,72 (en iyi 80,01) · v6g/v01110 20/20, ort 80,42 · önceki şampiyon 19/20, ort 82,97.

**Sabah GUI testi** (TORCS: Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race, mavi ekran):
```
cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v6g.py --mode gui --label "SUBMISSION_v6g@GUI"
```
Beklenen: ~79,8 s (TORCS ekranında 01:19:80 civarı), 0 hasar.

## Ne değişti ve neden işe yaradı
1. **Patinaj kontrolü (TCS) eşiği 2 → 5 m/s.** Analiz: eski eşik düzlük adımlarının %61'inde gazı kesiyordu.
   Hızlanma testinde 0-300 m: 8,26 s/234 km/h → 8,04 s/244 km/h. Her hızlanma bölgesinde ~0,2 s.
2. **Kenar cezası 0,75 → 0,9.** Araç virajlarda pist genişliğini daha çok kullanabiliyor (yarış çizgisi).
   Sadece TCS (v6a) daha kırılgandı; TCS + kenar (v6b tarifi) hem hızlı hem sağlam çıktı.
3. Denenen ama işe yaramayan: **pedal eşlemesi** (v7a: politika 0,5 = tam gaz) — modele uyum sağlayamadı, 460 m virajında takıldı.
4. Eğitimler hâlâ ~60-90. dakikadan sonra bozulmaya meyilli; en iyi modeller ilk 40-80 dakikada çıkıyor.
   Bu yüzden tek uzun eğitim yerine **çok tohumlu kısa eğitim + titiz seçim** en verimli yol.

## Deneyler
| Eğitim | Ayar | Sonuç |
|---|---|---|
| v6a | TCS 5 | eval 80,4–80,7 ama kırılgan (en iyi 9/10) |
| v6b | TCS 5 + kenar 0,9 | 4 aday 10/10; v00249: 81,18 s det. |
| v6c, v6d | v6b tarifi, 2 tohum | v6c/v01404 20/20 (81,42 s det.); v6d çöktü |
| v7a | + pedal eşlemesi | başarısız |
| v6e, v6f | v6b tarifi, 2 tohum | **v6f/best 79,86 s det.**; v6f/v00879 80,70 s (10/10, ×20'de 14/20) |
| **v6g** | v6b tarifi, son tohum (45 dk, 8 worker) | **best: 20/20, 79,80 s det. → yeni şampiyon** |

## Teknik notlar
- Kod değişiklikleri geriye uyumlu (varsayılanlar eski davranış); eski şampiyon regresyon testinde aynı sonucu (81,904 s) verdi.
- v6 modelleri TCS 5,0 ile çalışmalı: agent `racelab/agents/sac_agent_v6.py` (varsayılan 5,0); teslim dosyalarına eşik gömülü.
- torcs/ ve gym_torcs/ dosyalarına dokunulmadı.

## Sıradaki öneriler
- GUI'de v6g denemesi (video adayı).
- Hız için: aynı tarifle birkaç tohum daha (her biri ~45-60 dk) + seçim. Sağlamlık için: v6f_fast'in zamanlama hassasiyetini
  azaltmak (daha yüksek adım düşürme olasılığıyla kısa ince ayar).
