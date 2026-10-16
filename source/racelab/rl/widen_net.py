"""Blok 3: SAC ağlarını (aktör + critic) daha geniş/derin hale getirir, davranışı birebir koruyarak.

  python racelab/rl/widen_net.py --src racelab/rl/runs/v4d/final_full.pt --out racelab/rl/runs/init_w512x3.pt \
      --obs 44 --hidden 512 512 512

- Girişler gerekirse 32→44 genişletilir (expand_obs_inputs, yeni sütunlar sıfır).
- Her gizli katman genişler: eski nöronlar aynen kopyalanır; yeni nöronların GİRİŞ ağırlıkları rastgele
  (öğrenebilsinler diye), ÇIKIŞ ağırlıkları sıfır (başlangıçta çıktıya etkisiz).
- Fazladan gizli katmanlar birim matrisle eklenir (ReLU çıktısı ≥0 olduğundan ReLU(I·h) = h).
Sonuç: yeni ağın çıktısı eskisiyle aynı (float yuvarlaması dışında); kapasite eğitimle kullanılır.
"""
import argparse
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sac import expand_obs_inputs  # noqa: E402

ACT_DIM = 2


def _layers(sd, prefix):
    idx = sorted({int(k[len(prefix):].split(".")[0]) for k in sd if k.startswith(prefix)})
    return [(sd[f"{prefix}{i}.weight"], sd[f"{prefix}{i}.bias"]) for i in idx]


def widen_mlp(layers, hidden, gen):
    """layers: [(W,b), ...] eski MLP (son katman çıktı). hidden: yeni gizli boyutlar."""
    old_hidden = [w.shape[0] for w, _ in layers[:-1]]
    assert len(hidden) >= len(old_hidden) and all(h >= o for h, o in zip(hidden, old_hidden))
    out = []
    prev_old, prev_new = layers[0][0].shape[1], layers[0][0].shape[1]
    for i, h in enumerate(hidden):
        if i < len(old_hidden):
            w_old, b_old = layers[i]
            o = w_old.shape[0]
            bound = 1.0 / prev_new ** 0.5
            w = (torch.rand(h, prev_new, generator=gen) * 2 - 1) * bound  # yeni nöronların girişleri
            w[:, prev_old:] = 0.0 if i > 0 else w[:, prev_old:]  # önceki katmanın yeni nöronları → sıfır
            w[:o, :] = 0.0
            w[:o, :prev_old] = w_old
            b = torch.zeros(h)
            b[:o] = b_old
            if i > 0:  # yeni nöronlar önceki katmanın yeni nöronlarından da beslenebilir (çıkışları
                w[o:, prev_old:] = (torch.rand(h - o, prev_new - prev_old, generator=gen) * 2 - 1) * bound
                # eski nöronlara giden yeni→eski bağlar sıfır kalır: eski çıktı korunur
            prev_old = o
        else:  # eklenen derin katman: birim matris
            assert h == prev_new, "eklenen katman bir öncekiyle aynı genişlikte olmalı"
            w = torch.eye(h)
            b = torch.zeros(h)
        out.append((w, b))
        prev_new = h
    w_old, b_old = layers[-1]
    w = torch.zeros(w_old.shape[0], prev_new)
    w[:, :prev_old] = w_old
    out.append((w, b_old.clone()))
    return out


def to_sd(layers, prefix):
    sd = {}
    for i, (w, b) in enumerate(layers):
        sd[f"{prefix}{2 * i}.weight"] = w.contiguous()
        sd[f"{prefix}{2 * i}.bias"] = b.contiguous()
    return sd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--obs", type=int, default=44)
    ap.add_argument("--hidden", type=int, nargs="+", default=[512, 512, 512])
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    assert not os.path.exists(args.out), f"var olan dosyanın üzerine yazılmaz: {args.out}"

    ck = torch.load(args.src, map_location="cpu", weights_only=False)
    old_obs = ck["actor"]["net.0.weight"].shape[1]
    if old_obs != args.obs:
        ck = expand_obs_inputs(ck, old_obs, args.obs, ACT_DIM)
    new = {"hidden": list(args.hidden), "log_alpha": ck["log_alpha"], "widened_from": args.src}
    gen = torch.Generator().manual_seed(args.seed)
    new["actor"] = to_sd(widen_mlp(_layers(ck["actor"], "net."), args.hidden, gen), "net.")
    for key in ("critic", "critic_targ"):
        sd = {}
        for q in ("q1", "q2"):  # critic ve hedefi aynı rastgele bloklarla genişlesin
            g = torch.Generator().manual_seed(args.seed * 100 + (1 if q == "q1" else 2))
            sd.update(to_sd(widen_mlp(_layers(ck[key], f"{q}."), args.hidden, g), f"{q}."))
        new[key] = sd
    torch.save(new, args.out)

    # doğrulama: eski ve yeni aktör/critic çıktıları aynı mı
    from sac import Actor, Critic
    old_h = [w.shape[0] for w, _ in _layers(ck["actor"], "net.")[:-1]]
    a_old, a_new = Actor(args.obs, ACT_DIM, tuple(old_h)), Actor(args.obs, ACT_DIM, tuple(args.hidden))
    a_old.load_state_dict(ck["actor"]); a_new.load_state_dict(new["actor"])
    c_old, c_new = Critic(args.obs, ACT_DIM, tuple(old_h)), Critic(args.obs, ACT_DIM, tuple(args.hidden))
    c_old.load_state_dict(ck["critic"]); c_new.load_state_dict(new["critic"])
    x = torch.randn(4096, args.obs)
    a = torch.rand(4096, ACT_DIM) * 2 - 1
    with torch.no_grad():
        da = (a_old.net(x) - a_new.net(x)).abs().max().item()
        dq = max((p - q).abs().max().item() for p, q in zip(c_old(x, a), c_new(x, a)))
    n_par = sum(v.numel() for v in new["actor"].values())
    print(f"kaydedildi: {args.out}  gizli={args.hidden}  aktör parametre={n_par}")
    print(f"fark: aktör max {da:.2e}, critic max {dq:.2e}")


if __name__ == "__main__":
    main()
