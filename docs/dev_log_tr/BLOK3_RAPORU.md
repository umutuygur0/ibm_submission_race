# Blok 3 raporu — 2026-10-12 00:32 → 08:00: daha büyük ağ (512 × 512 × 512)

Plan ve saat saat günlük: [BLOK3_PLANI.md](BLOK3_PLANI.md). Eski dosyaların üzerine yazılmadı.
Şampiyonun yedeği: `racelab/YEDEK_SAMPIYON_v10d_2026-10-12/` (model, eğitim klasörü, başlangıç modeli, teslim dosyası).

## Sonuç
**Önerilen teslim dosyası değişmedi: `racelab/submission/torcs_jm_par_rl_v10d_dist_r02.py`** — artık 30/30 kuralı ile doğrulandı.

| Teslim dosyası | Ağ | Deterministik (= GUI, adım kaçmazsa) | 10 ms ×30, sistem boş | ×30 ortalama |
|---|---|---|---|---|
| **`torcs_jm_par_rl_v10d_dist_r02.py`** | 256 × 256 | **73,098 s**, 0 hasar | **30/30 hasarsız** | 74,32 |
| `torcs_jm_par_rl_w512a_v08171_a3m50.py` (yeni) | 512 × 512 × 512 | 74,29 s, 0 hasar | 30/30 hasarsız | 75,70 |

Büyük ağ kullanılabilir (30/30) bir model üretti, ama şampiyondan ~1,2 s yavaş. 6 aday × 40 yardımcı ayarlı taramada
en iyi kombinasyon 73,93 s → şampiyon geçilmedi.

## Ne yapıldı
1. **`rl/widen_net.py` (yeni):** v4d modelini (şampiyonun çıktığı başlangıç) davranışını koruyarak 256×256 → 512×512×512'ye
   genişletir: eski nöronlar aynen kopyalanır, yeni nöronların girişleri rastgele / çıkışları sıfır, 3. katman birim matris.
   Doğrulama: çıktı farkı ~4e-5 (float yuvarlaması), pistte birebir aynı davranış. Aktör 78 bin → 550 bin parametre.
2. **w512a eğitimi:** şampiyonla aynı tarif (v10, tohum 91), 5,5 saat, ~5,5 milyon güncelleme (~280/sn; 256'lık ağın ~3 katı yavaş).
3. **`submission/build_submission_v2.py` (yeni):** her katman sayısını destekleyen teslim oluşturucu; regresyon: şampiyonu
   birebir üretiyor (73,098 s). Var olan dosyanın üzerine yazmayı reddeder.

## Bulgular
1. **Büyük ağ çok hızlı öğreniyor ama daha iyi bir yere varmadı.** 10. dakikada yardımcısız 75,24 s (256'lık eğitimler 30. dakikada
   ~76–77); sonra uzun bir plato, 4,5 saatte 75,01 s. 500 sürümlük aday ortalamalarının en iyisi 75,75 (v10d: 75,88) — yardımcısız
   olarak şampiyonun eğitimiyle başa baş, belirgin üstün değil.
2. **Yardımcıyla uyum yine sorun:** yardımcısız turu iyi olan adaylar yardımcıyla 73,9–74,7'de kalıyor (Blok 2'deki 256'lık
   tohumlarla aynı tablo). v10d'nin yardımcıyla 73,10 yapması hâlâ istisna.
3. **Zamanlama hassasiyeti:** büyük ağın adayları quick_exp/select'in 10 ms testlerinde belirgin daha kırılgan (%10–20), ama
   teslim dosyası yoluyla ×30'da 30/30. Kesin karar her zaman teslim dosyası + run_benchmark ×30 ile verilmeli.
4. **Eğitim sırasında yapılan 10 ms testleri geçersiz:** 512³ eğitiminin yükü altında şampiyon bile 1/8'e düştü.
5. **Teslim notu:** 512³ ağ numpy ile adım başına <0,1 ms, **saf Python ile 17 ms** (10 ms sınırını aşar) → büyük ağ teslim
   edilecekse ortamda numpy şart. 256² şampiyonda bu risk yok.

## Sıradaki öneriler
- **Şampiyonu GUI'de dene** (video adayı; beklenen ekran süresi ~01:13:10):
  ```
  cd "C:\Users\Asus\Desktop\projeler egit\ıbmrace"
  python racelab/run_benchmark.py --agent racelab/submission/torcs_jm_par_rl_v10d_dist_r02.py --mode gui --label "r02@GUI"
  ```
- Kapasite tek başına yeterli olmadı; kazanç büyük olasılıkla politikanın yardımcıyla birlikte eğitilmesinde. Ya bunu
  (şampiyondan başlayarak, düşük öğrenme hızıyla) bir blok daha deneriz, ya da 73,10 s ile (TORCS botlarından 1,1 s hızlı)
  teslim hazırlığına geçeriz: temiz ayrı repo, basit livery, video.
