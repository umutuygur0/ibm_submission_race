# 2026-10-09 08:20 — ÖNCE SABAH_RAPORU_4.md OKU
- ŞAMPİYON: submission/torcs_jm_par_rl_v10d_dist.py (v10 konum bilen model v10d/v04188 + mesafe yardımcısı 3g/50m + vites 18650):
  73,582 s / 0 hasar, 10 ms ×10 10/10. GUI'de henüz denenmedi. TORCS botları 74,2 s → geçildi.
- v10 = gözleme pist konumu (distFromStart sin/cos, 44 giriş). Tüm araçlar giriş boyutundan otomatik algılıyor.

# 2026-10-08 16:20 — DURDURULDU (güvenli), önce ARASTIRMA_HIZ.md oku
- En iyi teslim adayı: submission/torcs_jm_par_rl_s47_dist.py → 77,436 s / 0 hasar (GUI'de henüz denenmedi).
  GUI'de doğrulanan en iyi: torcs_jm_par_rl_s47_assist.py → 01:18:15.
- Referans: TORCS botları aynı araçla 74,2 s (hedef, ulaşılmadı; fark ~3,2 s).
- Devam: python racelab/rl/night_pipeline.py --prefix n4 --start-seed 63 --minutes 45 --until 06:30
- Sıradaki fikir: gözleme pistteki konum (distFromStart) eklemek → piste özgü fren/gaz noktaları öğrenme.

# SABAH 2026-10-08 — ÖNCE SABAH_RAPORU_3.md OKU
- Önerilen teslim: submission/torcs_jm_par_rl_s47_assist.py (n3b_s47 + düzlük yardımcısı 180 m + vites 18650):
  deterministik 78,158 s / 0 hasar, 10 ms ×10 10/10. GUI'de henüz denenmedi. Yedek: torcs_jm_par_rl_v6g_assist.py (78,934).
- Yardımcı/vites ayarları teslim dosyasına gömülü (build_submission --assist-front --upshift --tcs-th).

# SABAH 2026-10-07 — ÖNCE SABAH_RAPORU_2.md OKU
- Yeni şampiyon: rl/snapshots/v6g_best.pt (TCS 5.0, kenar 0.9). Teslim: submission/torcs_jm_par_rl_v6g.py →
  79,796 s / 0 hasar, 20/20 + 10/10 hasarsız. Henüz gerçek GUI'de denenmedi.
- Eski şampiyon dosyası submission/torcs_jm_par_rl.py (81,73 s GUI'de doğrulandı) değişmedi.
- v6 modelleri için agent: agents/sac_agent_v6.py (--tcs-th 5.0); select_models --tcs-th 5.0.

# SABAH 2026-10-06 — ÖNCE SABAH_RAPORU.md OKU
- Şampiyon: rl/snapshots/v4d_v02537_best.pt (81,90 s, 0 hasar, 40/40). Teslim: submission/torcs_jm_par_rl.py (doğrulandı).
- Alternatif hızlı/riskli: v4e_v00248 (80,67 s, ~8/10). v4f seçildi: en iyi 9/10, 82,75 s (şampiyonu geçmedi).
- Sıradaki: kullanıcı GUI Corkscrew testi; teslim biçimi; v4f seçim; gerekirse yeni tohumlarla v4d tarifi.

# GECE (2026-10-05 22:40 → 07:00) — otonom çalışma, plan: GECE_PLANI.md, sonuç: SABAH_RAPORU.md
- v4 (yumuşak zikzak cezası) çalışıyor: 3 dk testte zikzak 8,6 → 0,7 yön değişimi/s, tur 84,2 s.
- Tespit: hızı asıl sınırlayan politika — düzde bile pedal ort. %43 (temkinli). v4b'de +%50 ilerleme ödülü.
- 22:45 başladı (3 saat): v4a (idx 1-4), v4b --c-speed 0.25 (idx 5-8), ikisi de v2_ft/final_full'dan.
- Teslim dosyası hazır: submission/torcs_jm_par_rl.py (build_submission.py ile üretilir; orijinal dokunulmadı).
  numpy: 85,704 s / 0 hasar; eğitim yükü altında GUI zaman. 4/5 hasarsız. Saf Python: doğru ama 3,65 ms/adım,
  yük altında 10 ms'yi kaçırıyor (resmi kurulum rehberi numpy kurduruyor → numpy modu geçerli).

# Kaldığımız yer — 2026-10-05 ~15:50

Teslim: **11 Ekim** (Corkscrew, 1 tur, en hızlı + sıfır hasar).

## Doğrulananlar
- Hızlı mod (`wtorcs -r`) = gerçek GUI. İki bağımsız GUI testi:
  - snakeoil3_gym, E-Track 4: GUI 113,69 s / 1585 hasar / 302 km/h ↔ hızlı mod 113,698 s / 1585 / 302,3
  - RL v2 modeli, E-Track 4: GUI 130,346 s / 0 hasar / 241 ↔ hızlı mod 130,356 s / 0 / 241,4
- GUI modunda TORCS lastLapTime göndermiyor → racelog.py son curLapTime ile bitişi tespit ediyor (düzeltildi).

## En iyi model şu an
`rl/snapshots/v2_ft_v05581_best.pt` + **`agents/sac_agent_v2.py`** (ısınma + kuyruk boşaltma)
- Corkscrew deterministik: **85,736 s, 0 hasar** (skor tablosunda #1)
- GUI zamanlaması ×10: 10/10 bitiş, 9/10 hasarsız; 0 kaçan adımda hep 85,736 s
- Sorun: düz yolda zikzak (hız>150'de saniyede ~9 yön değişimi, ort gaz %54) → hız kaybı
- sac_agent.py (v1) bu modelle 2/10: başta kaçan adımlar kalıcı gecikme yaratıyordu → v2 agent bunu çözdü.

## 2026-10-05 ~16:50 güncellemesi — çalışan süreç YOK
- v3 (sert direksiyon hız sınırı) BAŞARISIZ: v3_ft_1h ve v3_ft_strong_1h (STOP ile güvenle kapatıldı, final.pt kayıtlı)
  0 tur; araç ~400-500 m (ilk sert fren virajı) ve ~2400-2470 m (corkscrew S virajı) noktalarında pistten çıkıyor.
  Sebep: 200 km/h'te adım başına 0.04 sınırı, hızlı direksiyon gereken virajları imkânsız kılıyor.
- **Şampiyon hâlâ v2:** rl/snapshots/v2_ft_v05581_best.pt + agents/sac_agent_v2.py (85,736 s, 0 hasar).
- SIRADAKİ = v4 (akşam büyük eğitim): sert sınır yok; zikzak sadece ödülle cezalandırılır
  (direksiyon yön değiştirme cezası + önü açıkken/düzde büyük direksiyon cezası). v2_ft/final_full.pt'den devam.
  Önce kısa testle 450 m ve 2470 m virajlarını geçebildiğini doğrula.
- Yeni: eğitim klasörüne `STOP` dosyası → güvenli kapanış. Telemetri artık koşu sırasında diske yazılıyor.
- Kaybolan Corkscrew GUI testi: terminal penceresi kapatıldığı için (window-CLOSE) veri yok; tekrar yapılacak.

## (eski) Çalışan / sıradaki
- 15:50'de başladı, ~17:50 biter: `v3_ft` ve `v3_ft_strong` (torcs_env_v3: direksiyon hız sınırı + zikzak cezası,
  v2_ft/final_full.pt'den devam). Agent: `agents/sac_agent_v3.py`.
- Bitince: `python racelab/rl/select_models.py --run v3_ft --env v3 --top 20 --repeat 10 --procs 6 --idx-from 1`
  (aynısı v3_ft_strong), sonra en iyisini run_benchmark ile sac_agent_v3 üzerinden skor tablosuna yaz ve GUI'de Corkscrew testi.
- Teslim dosyası: torcs_jm_par.py iskeleti (REQUIRED bloklar) + numpy'ye gömülü ağırlıklar, PyTorch'suz. (henüz yapılmadı)

## 2026-10-11 08:00 güncellemesi (Blok 2 sonu) — çalışan süreç YOK
- **Önerilen teslim:** `submission/torcs_jm_par_rl_v10d_dist_r02.py` (v10d_v04188 + yardımcı 4 g / 20 m + 0,2 s tepki payı):
  deterministik 73,098 s, 0 hasar; eğitim yükü altında 10 ms ×20: 20/20 hasarsız, ort 73,62. GUI'de henüz denenmedi.
- Blok 2'de 11 yeni eğitim (b2a, D1 b2b, tohumlar 80–87, 90) şampiyonu geçemedi. Ayrıntı: BLOK2_RAPORU.md.
- Tohum turu betiği: `rl/seed_round2.sh A B SAAT` (seed_round.sh'nin özet hatası düzeltilmiş kopyası).

## 2026-10-12 06:30 güncellemesi (Blok 3 sonu) — çalışan süreç YOK
- Şampiyon `submission/torcs_jm_par_rl_v10d_dist_r02.py` **30/30 hasarsız** (10 ms, sistem boş), det 73,098, ×30 ort 74,32.
- 512×512×512 ağ denendi (`rl/widen_net.py`, eğitim `rl/runs/w512a`, teslim `submission/torcs_jm_par_rl_w512a_v08171_a3m50.py`):
  30/30 ama det 74,29 → şampiyonu geçmedi. Çok katmanlı teslim oluşturucu: `submission/build_submission_v2.py`. Ayrıntı: BLOK3_RAPORU.md.

## 2026-10-13 sabah güncellemesi (Blok 4 sonu) — yarış çizgisi sürücüsü
- Rakip 1:05.56 bildirdi (önceden hesaplanmış çizgi + hız planı). Kendi uygulamamız `racelab/planner/` (RL değil, kural tabanlı
  + otomatik parametre araması). **Önerilen teslim:** `submission/torcs_jm_par_line_e2d1.py` — det 68,746 s, 0 hasar,
  10 ms ×30: 30/30 hasarsız (ort 69,27). Saf Python. RL şampiyonu (73,10) yedek olarak duruyor.
- Önemli: run_benchmark 10 ms koşullarında kalıcı 1 adım gecikme oluşuyor (sim.py delay1 ile birebir taklit). Aramalarda her
  aday det + delay1 ile değerlendirilmeli (`evo_all.py --delay1`), aksi halde 30/30 kaybolur (e1c: 67,77 ama 20/30).
- Ayrıntı: BLOK4_RAPORU.md, günlük BLOK4_PLANI.md.

## 2026-10-16 08:05 güncellemesi (Blok 9 sonu) — çalışan süreç YOK (keep_awake hariç)
- **En hızlı:** `submission/torcs_jm_par_line_g9a.py` — det 63,628 s, hasar 0 (3/3 tekrar aynı). Ama 10 ms ×30 yalnız 3/30 hasarsız
  → sadece gecikmesiz/GUI koşu için. Gecikmeye dayanıklı yedek: `submission/torcs_jm_par_hybrid_res3a.py` (64,878, 30/30).
- Kazanç kaynağı: dar paylı düzgünleştirilmiş çizgi (irlsc8p1, pay 1,2) + det-only evrim. Drift kazandırmadı. Ayrıntı: BLOK9_RAPORU.md.
