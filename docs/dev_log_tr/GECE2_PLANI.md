# Gece planı 2 — 2026-10-07 00:55 → 08:00 (otonom)

Amaç: düzlüklerde daha iyi hızlanma + virajları daha optimal alma. Eski raporların üzerine yazılmaz
(GECE_PLANI.md, SABAH_RAPORU.md, submission/torcs_jm_par_rl.py korunur). Sonuç: `SABAH_RAPORU_2.md`.
Başlangıç şampiyonu: v4d_v02537 → 81,904 s (GUI'de 81,73 s), 0 hasar, 40/40.

## Analiz (00:51)
- Düzlüklerde (ön sensör ≥150 m) politika pedalı ort. 0,64, tam gaz %2; **TCS düzlük adımlarının %61'inde gazı kesiyor**
  (ort. kayma 2,14 m/s > eşik 2,0).
- Hızlanma testi (tam gaz, 0-300 m): TCS 2.0 → 8,26 s / 234 km/h; **4.0 → 8,04 s / 244 km/h**; 8.0 → 8,04 / 248; kapalı → 8,54 (kalkışta patinaj).
  → eşik ~5 her hızlanma bölgesinde ~0,2 s + ~10 km/h.
- Virajlar: |trackPos|>0,75 kenar cezası yarış çizgisini (dıştan gir-içe-dışa çık) engelliyor olabilir.

## Plan
| Saat | Faz | İş |
|---|---|---|
| 00:58–02:28 | R1 | **v6a**: TCS 5.0 (idx 1-4) · **v6b**: TCS 5.0 + safe_pos 0.9 (idx 5-8). İkisi de v4d/final_full'dan, c_speed 0.25 |
| 02:30–02:50 | S1 | Seçim ×10 (--tcs-th 5.0), kontrol: şampiyon |
| 02:50–05:20 | R2 | Kazanan ayarla 2 farklı tohum (v2_ft veya v4d'den, hangisi daha iyi çıkarsa) |
| 05:20–05:40 | S2 | Seçim |
| 05:40–07:10 | R3 | En iyi tarifle son tohumlar |
| 07:10–07:50 | F | Final ×20 (kontrol: şampiyon), resmi benchmark, yeni teslim dosyası **yeni adla**, SABAH_RAPORU_2.md |

Kod değişiklikleri (geriye uyumlu, varsayılanlar eski davranış): common.to_command(tcs_threshold=2.0),
TorcsEnvV2(tcs_threshold, safe_pos), train_sac_v2 --tcs-th/--safe-pos, select_models --tcs-th,
agents/sac_agent_v6.py (--tcs-th 5.0), build_submission --tcs-th ve --out zorunlu. Regresyon: şampiyon 81,904 s (aynı).

## Günlük
- 00:58 R1 başladı: v6a, v6b (1,5 saat).
- 02:28 R1 bitti. S1 (×10, TCS 5.0): **v6b** 4 aday 10/10 (en iyi v00249: ort 81,92 s); v6a en iyi 9/10 ort 81,55 (eval'de 80,4-80,7 ama kırılgan).
  Karşılaştırma: şampiyon ilk turda (×10) ort 83,37, final ×20 ort 82,52.
- 02:31 R2: v6b tarifi (TCS 5 + safe_pos 0.9, c_speed .25, v4d/final_full'dan) iki yeni tohum: v6c (seed 12, idx 1-4), v6d (seed 13, idx 5-8), 2,5 saat.
- 03:50 v6c en iyi eval 80,22 s (221 aday) ama bozuluyor; v6d çöktü (460 m) → STOP (121 aday kayıtlı).
  Ara ölçüm: v6b_v00249 deterministik **81,182 s / 0 hasar** (şampiyon 81,904) ama ort. gaz hâlâ %50, tam gaz %2.
- 03:53 **v7** deneyi (idx 5-8, 1,5 saat): v6b tarifi + pedal eşlemesi pedal'=clip(1.333·p+0.333) (politika 0.5 = tam gaz).
  v6b/final_full'dan. Kod: to_command(pedal_gain, pedal_bias) — varsayılanlar eski davranış (regresyon 81,904 aynı).
- 04:56 **v7a başarısız**: pedal eşlemesine uyum sağlayamadı, 460 m'de takılı, 65 dk'da 2 aday (83,8 s) → STOP. Eşleme değişikliği ince ayar için fazla büyük.
- 05:06 S2 (×10, TCS 5): v6c/v01404 10/10 ort 82,01; v6c/v02309 9/10 ort 81,87; v6d en iyi 10/10 ama 85,06.
- 05:08 R3: v6b tarifinin iki yeni tohumu v6e (seed 14, idx 1-4), v6f (seed 15, idx 5-8), 1,45 saat (~06:35).
  Final planı 06:40: S3 + final ×20 (v6b_v00249, v6c_v01404, v6c_v02309, v6a_v02634 + R3 en iyileri; şampiyon TCS 2.0 ayrı kontrol).
- 06:35 R3 bitti. S3 (×10): v6f/v00879 10/10 ort 81,47; v6f/best 9/10 ort 80,72.
- 06:41 Final ×20 (TCS 5, procs 4 — 10'luk turdan daha sert koşul): v6c/v01404 20/20 ort 83,29; şampiyon v4d (kontrol, TCS 2) 19/20 ort 82,97;
  v6b/v00249 17/20; v6f/v00879 14/20 ort 82,07; v6f/best 7/20 ort 81,01 (en iyi 80,07).
- 06:42 Deterministik (= gerçek GUI koşulu, adım kaçmıyor): **v6f/best 79,864 s / 0 hasar** (şampiyondan −2,04 s), v6f/v00879 80,696,
  v6f/v01759 81,056, v6c/v01404 81,424.
- 06:44 Yeni teslim dosyaları (eskilerin üzerine yazılmadı): submission/torcs_jm_par_rl_v6f_fast.py → 79,806 s / 0 hasar, GUI zaman. 4/10;
  submission/torcs_jm_par_rl_v6c_safe.py → 81,352 s / 0 hasar, GUI zaman. 10/10. torcs_jm_par_rl.py (v4d) hash değişmedi.
- 06:46 Son tohum: v6g (v6b tarifi, seed 16, 8 worker, 45 dk) → ~07:31, ardından hızlı seçim + rapor.
- 07:31 v6g bitti (70 aday). Seçim ×10: v6g/best 10/10 ort 80,18; v6g/v01110 10/10 ort 80,31.
- 07:34 Final ×20: **v6g/best 20/20 ort 80,72 (en iyi 80,01)**, v6g/v01110 20/20 ort 80,42.
  Teslim dosyası **submission/torcs_jm_par_rl_v6g.py**: deterministik **79,796 s / 0 hasar / max 264 km/h**, GUI zaman. ×10 **10/10 hasarsız**.
  → GECENİN ŞAMPİYONU. Rapor: SABAH_RAPORU_2.md. Açık eğitim yok.
