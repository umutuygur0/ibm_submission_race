# Blok 9 Raporu — 2026-10-15 18:12 → 2026-10-16 08:05

Hedef (kullanıcı kararı): **hasarsız en hızlı deterministik tur** (GUI = deterministik koşu). Pist dışına çıkmak serbest,
tek şart hasar 0. Gecikme (10 ms) sağlamlığı birincil kriter değil. Ek soru: kayarak (drift) dönmek daha hızlı mı?

## Sonuç

| Teslim dosyası | Det tur | Hasar | 10 ms ×30 hasarsız | Not |
|---|---|---|---|---|
| `submission/torcs_jm_par_line_g9a.py` | **63,628 s** | 0 | 3/30 (ort 63,96) | **en hızlı**, yalnız gecikmesiz koşu için |
| `submission/torcs_jm_par_line_g7a.py` | 63,928 s | 0 | 6/30 (ort 64,47) | ara sürüm |
| `submission/torcs_jm_par_line_g6a.py` | 64,026 s | 0 | ölçülmedi | ara sürüm |
| `submission/torcs_jm_par_hybrid_res3a.py` | 64,878 s | 0 | **30/30** (ort 65,16) | önceki önerilen, gecikmeye dayanıklı |

- Rakip (One Qubit Racing) 1:05,56 → g9a **~1,93 s daha hızlı**; res3a bile ~0,7 s daha hızlı.
- g9a det tekrarlanabilirlik: run_benchmark (port 3010) 3/3 aynı: 63,628 s, hasar 0, çarpma 0, zaman aşımı 0, pist_dışı 14,
  max 299 km/h. Max |trackPos| 1,498 (pist kenarından ~0,5 genişlik dışarı; hasar yok).
- Final özet dosyası: `planner/runs/final_check_20261016-075220.txt`.

### Önemli uyarı
g9a/g7a gecikmesiz koşuda kusursuz, ama 10 ms modunda (run_benchmark'ın kalıcı 1 adım gecikmesi) çoğu koşuda hasar alıyor.
Değerlendirme GUI'de/deterministikse g9a; değerlendirme ortamında gecikme/adım kaçırma olabilecekse res3a güvenli seçim.
**Göndermeden önce g9a'yı kendi GUI TORCS'unda 2–3 kez izleyip hasar 0 olduğunu doğrula.**

## Ne işe yaradı
1. **g6det** (18:20–01:55, `evo2.py --no-delay1 --pos-lim 1.5 --offstop 1.6`): t4off 64,722 → 64,026.
   Kazanç: biraz daha geniş çizgi kullanımı, geç frenleme, tcs ince ayarı.
2. **Hedefli koordinat** (`planner/targeted_det.py`, targeted.py'nin `--no-delay1` eklenmiş kopyası): 79 anahtarın hiçbiri
   iyileştirmedi → 64,026 yerel optimum; durduruldu.
3. **g7det** (01:58–05:00): aynı genler farklı yarış çizgilerinde (pay 1,0/1,2/1,3/1,7, düzgünleştirilmiş irlsc8p1/irlsc4p2).
   Dar pay (1,2–1,3 m) kazandı: 63,890 (pay 1,3).
4. **Yeni çizgiler**: irlsc8p1 (ağırlık 1+8·κ/κmax) pay 1,3 ve 1,2 için üretildi (`cache/line_irlsc8p1_m1.30.npz`,
   `_m1.20.npz`; üretici önbellekteki çizgileri birebir yeniden üretti, fark 0,0).
5. **g9det** (05:37–07:51): irlsc8p1 pay 1,2 → **63,628**.
   g8det (05:00–05:36) iyileştirmedi, erken durduruldu.

## Drift (kayarak dönme) sonucu
5 ada kaymaya izin veren ayarlarla (st_course 0,5–1, st_slipmax 0,1–0,2, düşük tcs eşiği) başladı; evrim hepsini kaymayı
neredeyse kapatan değerlere geri getirdi (st_course 0,056, st_slipmax 0,005). Bu araç/pistte kayma süre kazandırmıyor:
kayma açısı arttıkça yanal tutuş düşüyor, kaybedilen hız kazanılan açıdan fazla. Kazanç çizgi + fren + çekişten geldi.

## Dosyalar (hepsi yeni; eski dosyalara dokunulmadı)
- Parametreler: `planner/runs/snap_g6det.json`, `snap_g6det_0031.json`, `snap_g7det_a.json`, `snap_g7det_b.json`,
  `snap_g7_i{1,2,4}.json`, `snap_g9det.json`; aramalar `g6det*`, `g7det*`, `g8det*`, `g9det*`, `t6det*`.
- Kod: `planner/targeted_det.py`.

## Sonraki olası adımlar
- g9a'nın GUI'de gözle doğrulanması (kullanıcı).
- Hız ile gecikme dayanıklılığı arası orta yol: g9det çizgisi + delay1 geçerlilikli evrim (`evo2.py` varsayılanı) → 30/30'a
  yakın ve 64,5 altı bir sürüm mümkün olabilir.
- Daha dar pay (1,1) / farklı düzgünleştirme (irlsc12p1 vb.) çizgileri.
