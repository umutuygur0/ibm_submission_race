# racelab — benchmark, skor tablosu ve RL eğitimi

Bu klasör projeye **sonradan eklenmiştir**. `torcs/` ve `gym_torcs/` içindeki hiçbir dosyayı
değiştirmez. Yeni agent'lar ve deneyler hep burada **yeni dosyalar** olarak eklenir.

```
racelab/
├── run_benchmark.py        # bir agent'ı koştur → skor tablosuna kaydet
├── configs/
│   ├── corkscrew_1lap.xml  # yarışma koşulu: Corkscrew, 1 tur, scr_server 1
│   ├── etrack4_1lap.xml    # E-Track 4, 1 tur
│   └── train/              # RL worker'ları için otomatik üretilen config'ler
├── tools/racelog.py        # telemetri proxy'si + yarış özeti
├── results/
│   ├── LAP_LOG.md          # skor tablosu (otomatik)
│   ├── leaderboard.csv     # her koşu bir satır
│   └── runs/<run_id>/      # telemetry.csv, summary.json, torcs/agent çıktıları
├── agents/sac_agent.py     # eğitilmiş RL modeliyle yarışan agent
└── rl/
    ├── common.py           # gözlem, aksiyon, vites/ABS/TCS kuralları (eğitim = yarış)
    ├── torcs_env.py        # Windows'ta menüsüz TORCS RL ortamı
    ├── sac.py              # SAC (saf PyTorch, CUDA Graph hızlandırmalı)
    ├── train_sac.py        # paralel eğitim (N TORCS + GPU)
    └── runs/<ad>/          # progress.csv, episodes.csv, best.pt, final.pt
```

## Nasıl çalışır

**Telemetri proxy'si:** agent'ı `-p 3101` ile başlatır; proxy paketleri değiştirmeden
TORCS'un 3001 portuna iletir ve her adımı kaydeder. Agent dosyasına dokunmaya gerek kalmaz.
`-p` alan her snakeoil tabanlı agent ile çalışır.

**Hızlı mod (`--mode fast`, varsayılan):** `wtorcs.exe -r <config>` ile TORCS menüsüz ve
grafiksiz koşar. Bir tur ~3–8 saniye sürer. `-t 1000000` ile TORCS her adımda agent'ı bekler,
bu yüzden sonuç **birebir tekrarlanabilir** (aynı agent = aynı süre).

**GUI zamanlaması (`--udp-timeout-us 10000`):** TORCS'un GUI'deki 10 ms bekleme süresi.
Agent geç cevap verirse TORCS eski komutu kullanır, bu da küçük rastgele farklar yaratır.
Gerçek yarışa daha yakındır; `--repeat` ile birden çok koşturup bitirme oranına bakılır.

**GUI modu (`--mode gui`):** TORCS'u sen açarsın (Quick Race → pist → `scr_server 1` →
New Race, mavi ekran), script proxy ve agent'ı çalıştırıp kaydeder.

## Komutlar (proje kökünden)

```powershell
# mevcut bir agent'ı Corkscrew'da 3 kez koştur
python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py --repeat 3

# GUI zamanlamasıyla 10 koşu (bitirme oranı)
python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py --label "snakeoil3_gym@gui-timing" --udp-timeout-us 10000 --repeat 10

# başka pist
python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py --config etrack4_1lap.xml --label "snakeoil3_gym#etrack4"

# elle açtığın GUI yarışını kaydet
python racelab/run_benchmark.py --agent gym_torcs/snakeoil3_gym.py --mode gui

# RL modelini benchmark'la
python racelab/run_benchmark.py --agent racelab/agents/sac_agent.py --label sac_v1 --model racelab/rl/runs/sac_v1/best.pt

# RL eğitimi (port 3001 boş kalır; idx 1..9 → port 3002..3010)
python racelab/rl/train_sac.py --name sac_v1 --workers 8 --hours 1
```

Skor tablosu: [results/LAP_LOG.md](results/LAP_LOG.md)

## RL tasarımı (özet)

- **Agent'ın öğrendiği:** `steer ∈ [-1,1]`, `pedal ∈ [-1,1]` (pozitif gaz, negatif fren).
- **Kurallar:** 7 vitesli rpm tabanlı vites (18000 rpm'de vites büyütme), ABS, patinaj kontrolü.
- **Gözlem (32):** açı, pist konumu, hızlar, 19 mesafe sensörü (öne yoğun), tekerlek hızları, rpm, vites, önceki aksiyon.
- **Ödül:** ilerleme (m) × 0.5 − direksiyon titremesi − zaman cezası; hasar veya pist dışı → −10 ve episode biter
  (yarışma hedefi sıfır hasar); tur tamamlanırsa +20.
- **Hız:** ortam başına ~1500–4000 adım/s (gerçek zamanın 30–80 katı); CUDA Graph ile ~900 GPU güncellemesi/s.
- **Not:** `results_INVALID_v0_bitis_hatasi/` ilk sürümün hatalı "bitirdi" ölçümlerini içerir; kullanmayın.
