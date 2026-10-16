# Sabah raporu — gece çalışması (2026-10-05 22:40 → 2026-10-06 07:00)

Ayrıntılı saat saat günlük: [GECE_PLANI.md](GECE_PLANI.md) · Skor tablosu: [results/LAP_LOG.md](results/LAP_LOG.md)

## Sonuç (tek cümle)
Corkscrew'da **81,90 s / 0 hasar** yapan, gece boyunca **40/40 koşuda hasarsız** bitiren yeni bir model bulundu ve
**PyTorch gerektirmeyen teslim dosyası** bu modelle hazırlandı ve doğrulandı.

## Teslim dosyası (önerilen)
**`racelab/submission/torcs_jm_par_rl.py`** — model: `rl/snapshots/v4d_v02537_best.pt`

| Test (hızlı mod = GUI ile doğrulanmış) | Sonuç |
|---|---|
| Deterministik | **81,736 s, 0 hasar, 0 çarpma**, max 251 km/h |
| GUI zamanlaması (10 ms bekleme) ×10, numpy | **10/10 hasarsız**, 81,7–83,1 s |
| GUI zamanlaması ×5, numpy YOK (saf Python) | **5/5 hasarsız**, ~82,9 s (150+ kaçan adıma rağmen) |

- `gym_torcs/torcs_jm_par.py` iskeleti korunur: REQUIRED bloklar (port, `--parameters`, `get_servers_input`) aynı.
  Orijinal dosyaya dokunulmadı (hash aynı); dosya `submission/build_submission.py` ile üretilir.
- Çalıştırma (GUI'de Corkscrew, scr_server 1, mavi ekran):
  `python racelab/submission/torcs_jm_par_rl.py`  (RaceRunner gibi `-p <port>` de alır)

Alternatifler:
- `submission/torcs_jm_par_rl_fast_risky.py` (v4e_v00248): **80,67 s** ama ×10'da 8/10 hasarsız (2 kez ~1600 m'de çarpma) → riskli.
- `submission/torcs_jm_par_rl_v2.py`: dünkü model (85,7 s).

## Gelişim
| Model | Corkscrew tur | Hasar | Güvenilirlik | Not |
|---|---|---|---|---|
| snakeoil3_gym (temel) | 134 s (çoğunlukla DNF) | ~2000 | 4/25 bitiş | |
| v2 (dün) | 85,74 s | 0 | 9/10 (agent v2 ile) | düzde zikzak, max 231 km/h |
| **v4d (gece, teslim)** | **81,90 s** | **0** | **40/40** | zikzak 8,6→0,7 yön değ./s, max 251 km/h |
| v4e (gece, alternatif) | 80,67 s | 0 / bazen çarpma | ~17/20 | daha hızlı, daha riskli |

## Gecenin bulguları
1. **Zikzak çözüldü:** sert direksiyon sınırı (v3) virajları imkânsız kılmıştı; v4'te yumuşak cezalar
   (hıza bağlı direksiyon değişimi, yön değiştirme, düzde büyük direksiyon) + %50 ek ilerleme ödülü işe yaradı.
2. **İnce ayar çöküşü:** ödül değişince her eğitim ~15. dk'da çöküyor (her şey 460 m virajında), ~25-40 dk'da
   toparlanıyor. Düşük öğrenme hızı (v4c) çözmedi. Aday kaydı sayesinde iyi anlar kaybolmuyor.
3. **Uzun devam hızlandırmadı:** şampiyondan aynı ödülle 2-3 saat devam (v5a), gamma 0.998 (v5c) → 82-85 s, daha iyi değil.
   En iyi modeller "v2'den v4 ödülüne ince ayar"ın ilk 60-90 dakikasında çıkıyor (v4d, v4e). Tohum çeşitliliği değerli.
4. **Model seçimi şart:** eval'de hızlı tur atan adayların çoğu 10-20 tekrarda %50-85 güvenilir; seçim olmadan
   teslim riskli olurdu.

## Senden istenen (sabah)
1. **GUI testi (Corkscrew):** TORCS → Quick Race → Corkscrew, yalnız scr_server 1, 1 tur → New Race (mavi ekran), sonra:
   ```
   cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
   python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl.py --mode gui --label "SUBMISSION_v4d@GUI"
   ```
   Beklenen: ~81,7–83 s, 0 hasar. (Terminali yarış bitene kadar kapatma.)
2. Teslim biçimini kontrol et: yarışma tek dosya `torcs_jm_par.py` mi istiyor? Öyleyse `torcs_jm_par_rl.py`'ı
   o adla gönderebiliriz (orijinali değiştirmeden, kopyalayarak).

## Not
- `torcs/config/raceman/quickrace.xml` 5 Ekim 15:49'da TORCS menüsü tarafından güncellenmiş (senin GUI'de Corkscrew'u
  seçmen). Bunun dışında torcs/ ve gym_torcs/ dosyalarının hash'i ilk gündekiyle aynı.
- Son eğitim v4f (06:10–06:52, 13 aday) seçimden geçti: en iyisi 9/10 hasarsız, ort 82,75 s → şampiyonu geçmiyor.
- Gece sonunda hiçbir eğitim/TORCS süreci açık değil.
