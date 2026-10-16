# Gece planı 3 — 2026-10-07 21:35 → sabah (otonom)

Hedef: düzlüklerde daha fazla hızlanma. Eski raporlar korunur; sonuç `SABAH_RAPORU_3.md`.
Başlangıç şampiyonu: v6g → submission/torcs_jm_par_rl_v6g.py, **GUI'de 01:19:79 doğrulandı**, 0 hasar.

Faz 1 (→ 00:00): kısa deneme-yanılma ve hızlı kontroller. Faz 2 (00:00 → sabah): gözlemlere göre uzun eğitim.

## Analiz (21:36)
- v6g düzlükte (ön ≥150 m, toplam 15,2 s): pedal ort. 0,69, tam gaz %5, TCS kesme %14 → asıl kısıt politikanın temkinliliği.

## Eğitimsiz hızlı deneyler (quick_exp.py, deterministik; sonuçlar runs/quick_exp.csv)
| Varyant | Tur | Not |
|---|---|---|
| base | 79,804 | |
| düzlük yardımcısı ≥200 / ≥180 m | 79,632 / **79,41** | ≥150 çarptı, ≥120 pist dışı |
| vites 17000 / 18500 / 18650 | 80,154 / **79,55** / 79,552 | |
| TCS 6 / 8 | 79,772 / 79,748 | küçük |
| ≥160+18500 / ≥170+18500 / ≥180+18650 / ≥180+18500 | **78,274** / 78,878 / 78,934 / 79,24 | |
| sağlamlık ×8 (10 ms): ≥160+18500 1/8 ❌, ≥170+18500 8/8 (ort 80,03), ≥180+18650 8/8 (ort 79,73), ≥180+18500 7/8 | | |

- 21:40 **Yeni teslim adayı (eğitimsiz):** submission/torcs_jm_par_rl_v6g_assist.py = v6g + yardımcı 180 m + vites 18650:
  deterministik **78,934 s / 0 hasar / max 267,9**; 10 ms ×10 **10/10 hasarsız** (79,2–80,0).
- Kod (geriye uyumlu): common.straight_assist, auto_gear(upshift), TorcsEnvV2(assist_front, upshift), train/select/agent/builder seçenekleri.
  Regresyon: v6g yardımcısız 79,804 (aynı); ortam içi yardımcı 78,934 (quick_exp ile aynı).

## Günlük
- 21:42 Kısa eğitimler (50 dk, v6g/final_full'dan, yardımcı ortamda): v8a (≥170, 18500, idx 1-4), v8b (≥160, 18500, idx 5-8).
- 22:06 v8a/v8b 25 dk'da 0 tur (2500 m'de takılı). Neden: init olarak kullanılan v6g/final_full ÇÖKMÜŞ son durum (v6g logu: son anda 464 m, lap20 0). → STOP. Yeniden: v4d/final_full'dan (v6 tarifi + yardımcı) v8c (≥170, 18500, idx 1-4), v8d (≥180, 18650, idx 5-8), ~50 dk.
- 23:05 v8c/v8d (yardımcı ORTAMDA eğitim, v4d'den) seçim: v8c en iyi 10/10 ama ort 83,9; v8d %30-40. Eval turları 80,3-81 (v6g 79,87) → yardımcıyla eğitim işe yaramadı (yardımcı anlarında politika gaz sinyali almıyor + yüksek giriş hızları). Strateji: v6 tarifiyle eğit, yardımcıyı SONRADAN ekle.
- 23:05 Sonradan yardımcı (post-hoc) testleri: v6g ≥170-180 → 78,88-78,98 (≥185: 79,26); v6f yardımcıyla bitiremiyor (kırılgan); v6c 80,26.
  → Yardımcının faydası iyi bir taban modele bağlı.

## Faz 2 — gece hattı (night_pipeline.py), 23:10 → ~08:00
Gözlem: en iyi modeller v6 tarifinin kısa (45 dk, 8 worker) eğitimlerinden çıkıyor (tohum şansı); yardımcı + geç vites
sonradan eklenince ~0,9 s kazandırıyor. Hat döngüsü: tohum eğitimi 45 dk → seçim ×10 → en iyi 3 × yardımcı varyantları
{yok, 180/18650, 170/18500} deterministik + ×8 (10 ms) → runs/n3_pipeline.csv. Son döngü başlangıcı 07:05.
Durdurma: runs/n3_STOP. Başlangıç şampiyonu (yardımcılı): v6g + 180/18650 → 78,934 s, 10/10.
- 00:05 Hat çalışıyor. Döngü 1 (n3_s40, 23:56): zayıf tohum — en iyi 5/10; yardımcıyla det 80,58 (şampiyon 78,93'ü geçmiyor). n3_s41 eğitimde.
- 00:56 Döngü 2 (n3_s41): en iyi det 79,58 (v00735 + 180/18650, 7/8); v00379 yardımcısız 80,51 (8/8). Şampiyon geçilmedi. n3_s42 eğitimde.
- 01:48 Döngü 3 (n3_s42): en iyi det 80,34 (v00562 + 170/18500, 7/8). Şampiyon geçilmedi. n3_s43 eğitimde.
- 02:40 Döngü 4 (n3_s43): en iyi det 80,67. 4 tohum, şampiyon geçilmedi → tohum piyangosu verimsiz. Hat n3_s44 döngüsünden sonra duracak (n3_STOP). Yeni yol **v9**: v6g best aktöründen devam — aktör dondurulup critic ısıtılır, sonra düşük lr ince ayar.
- 02:50 v9 smoke (1 worker): ısınmada v6g aktörü %100 tur (79,8 s); 5000 critic güncellemesi yetersiz → aktör güncellenince hızla çöktü. Düzeltme: --critic-warmup 100000 + --actor-lr (critic'ten 10-30x düşük). 03:35'te v9a (actor-lr 3e-5) + v9b (1e-5) başlayacak.
- 03:13 Hat bitti; n3_s44 en iyi 80,73 → 5 tohumda (40-44) şampiyon geçilmedi. 03:15 v9a (actor-lr 3e-5) + v9b (1e-5): v6g aktöründen, critic-warmup 100k, lr 3e-4, alpha 0.03, ~06:58'e kadar.
- 03:36 v9a/v9b: ısınma sonunda (~11 dk) %90-95 tur, hemen ardından çöktü. Neden: otomatik alpha 0,035→0,06 fırladı (kararlı aktörü gürültüye zorluyor). → STOP. Yeniden: v9c (sabit alpha 0,01, actor-lr 1e-5), v9d (sabit alpha 0,003, actor-lr 3e-5). Not: v9a/b ısınma/erken aşamada 79,73-79,75 s adaylar kaydetti.
- 04:00 v9c/v9d (sabit alpha) de ısınmadan sonra çöktü → asıl sorun critic (tek politikanın verisiyle eğitilen critic dağılım dışında yanlış gradyan veriyor). v9 yolu kapatıldı. Kalan süre: eğitimsiz sürüş yardımcısı/donanım deneyleri (ABS/fren), sonra tohum hattı.
- 04:05 ABS eşiği deneyi (v6g+180/18650): 2 (mevcut) 78,934 en iyi; 3: 79,008; 4: 78,998; 6: 79,114; kapalı: 79,216 → fren tarafında kural kazancı yok. quick_exp sonuçları artık runs/quick_exp_v2.csv'de (abs_th sütunu). 04:08 tohum hattı yeniden: n3b, seed 45+, son başlangıç 07:05.
- 04:56 n3b_s45: en iyi det 80,47 (yardımcıyla). Şampiyon geçilmedi. n3b_s46 eğitimde.
- 05:49 n3b_s46: adaylar 10/10 ama en iyi det 80,69. Şampiyon geçilmedi. n3b_s47 eğitimde.
- 06:42 **n3b_s47/best.pt**: yardımcıyla det 78,156 (170/18500, 6/8) ve **78,584 (180/18650, 8/8, ort 79,76)** → şampiyon (78,934) geçildi. snapshots/n3b_s47_best.pt. Hat bitince (~07:32) final ×20 + teslim dosyası.
- 07:17 Hat bitti (n3b_s48 en iyi det 79,75). Final ×20 (select_models, kuyruk boşaltmasız → yardımcıyla sert koşul):
  s47+180/18650 13/20, v6g+180/18650 14/20, s47+170/18500 11/20 → bu koşul teslim dosyası yolunu temsil etmiyor.
- 07:19 Teslim dosyası karşılaştırması (run_benchmark, kuyruk boşaltmalı, CPU boş):
  **submission/torcs_jm_par_rl_s47_assist.py** (n3b_s47 + 180/18650): deterministik **78,158 s / 0 hasar**, 10 ms ×10 **10/10** (79,7-80,4, ort ~80,1).
  submission/torcs_jm_par_rl_v6g_assist.py: deterministik 78,934 / 0 hasar, 10 ms ×10 10/10 (79,4-80,2, ort ~79,7).
  → GUI'de adım kaçmadığı için beklenen: s47_assist ~78,16 s (önerilen), yedek v6g_assist ~78,93 s. Gece tamamlandı; açık süreç yok.
