"""Soft Actor-Critic (SAC) — saf PyTorch, ek bağımlılık yok."""
import copy
import math

import torch
import torch.nn as nn
import torch.nn.functional as F

LOG_STD_MIN, LOG_STD_MAX = -5.0, 2.0


def mlp(inp, out, hidden=(256, 256)):
    layers, d = [], inp
    for h in hidden:
        layers += [nn.Linear(d, h), nn.ReLU()]
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=(256, 256)):
        super().__init__()
        self.net = mlp(obs_dim, 2 * act_dim, hidden)
        self.act_dim = act_dim

    def forward(self, obs, deterministic=False, with_logp=True):
        mu, log_std = self.net(obs).chunk(2, dim=-1)
        log_std = log_std.clamp(LOG_STD_MIN, LOG_STD_MAX)
        std = log_std.exp()
        u = mu if deterministic else mu + std * torch.randn_like(mu)
        a = torch.tanh(u)
        if not with_logp:
            return a, None
        logp = (-0.5 * ((u - mu) / std) ** 2 - log_std - 0.5 * math.log(2 * math.pi)).sum(-1)
        logp -= (2 * (math.log(2) - u - F.softplus(-2 * u))).sum(-1)
        return a, logp


class Critic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=(256, 256)):
        super().__init__()
        self.q1 = mlp(obs_dim + act_dim, 1, hidden)
        self.q2 = mlp(obs_dim + act_dim, 1, hidden)

    def forward(self, obs, act):
        x = torch.cat([obs, act], -1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


class SAC:
    def __init__(self, obs_dim, act_dim, device, hidden=(256, 256), lr=3e-4, gamma=0.99,
                 tau=0.005, init_alpha=0.2):
        self.device = device
        self.gamma, self.tau = gamma, tau
        self.actor = Actor(obs_dim, act_dim, hidden).to(device)
        self.critic = Critic(obs_dim, act_dim, hidden).to(device)
        self.critic_targ = copy.deepcopy(self.critic).requires_grad_(False)
        self.log_alpha = torch.tensor(math.log(init_alpha), device=device, requires_grad=True)
        self.target_entropy = -float(act_dim)
        cap = device.type == "cuda"  # CUDA Graph ile yakalanabilir optimizer
        self.opt_actor = torch.optim.Adam(self.actor.parameters(), lr=lr, capturable=cap)
        self.opt_critic = torch.optim.Adam(self.critic.parameters(), lr=lr, capturable=cap)
        self.opt_alpha = torch.optim.Adam([self.log_alpha], lr=lr, capturable=cap)

    @property
    def alpha(self):
        return self.log_alpha.exp()

    def _update(self, o, a, r, o2, d):
        """Tek SAC güncellemesi; .item() yok (CUDA Graph içinde senkronizasyon olmasın)."""
        with torch.no_grad():
            a2, logp2 = self.actor(o2)
            q1t, q2t = self.critic_targ(o2, a2)
            target = r + self.gamma * (1 - d) * (torch.min(q1t, q2t) - self.alpha * logp2)
        q1, q2 = self.critic(o, a)
        loss_q = F.mse_loss(q1, target) + F.mse_loss(q2, target)
        self.opt_critic.zero_grad(set_to_none=False)
        loss_q.backward()
        self.opt_critic.step()

        for p in self.critic.parameters():
            p.requires_grad_(False)
        a_pi, logp = self.actor(o)
        q1p, q2p = self.critic(o, a_pi)
        loss_pi = (self.alpha.detach() * logp - torch.min(q1p, q2p)).mean()
        self.opt_actor.zero_grad(set_to_none=False)
        loss_pi.backward()
        self.opt_actor.step()
        for p in self.critic.parameters():
            p.requires_grad_(True)

        loss_alpha = -(self.log_alpha * (logp.detach() + self.target_entropy)).mean()
        self.opt_alpha.zero_grad(set_to_none=False)
        loss_alpha.backward()
        self.opt_alpha.step()

        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_targ.parameters()):
                pt.mul_(1 - self.tau).add_(self.tau * p)
        return torch.stack([loss_q.detach(), loss_pi.detach(), self.alpha.detach(), q1.mean().detach()])

    def update(self, o, a, r, o2, d):
        return tuple(self._update(o, a, r, o2, d).tolist())

    def update_critic_only(self, o, a, r, o2, d):
        """v9 critic ısıtma: aktör ve alpha sabit, sadece Q ağları (ve hedef ağ) güncellenir."""
        with torch.no_grad():
            a2, logp2 = self.actor(o2)
            q1t, q2t = self.critic_targ(o2, a2)
            target = r + self.gamma * (1 - d) * (torch.min(q1t, q2t) - self.alpha * logp2)
        q1, q2 = self.critic(o, a)
        loss_q = F.mse_loss(q1, target) + F.mse_loss(q2, target)
        self.opt_critic.zero_grad(set_to_none=False)
        loss_q.backward()
        self.opt_critic.step()
        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_targ.parameters()):
                pt.mul_(1 - self.tau).add_(self.tau * p)
        return loss_q.detach()

    def make_graphed_update(self, rb, batch, k=4):
        """Örnekleme + k güncellemeyi tek CUDA Graph olarak yakalar.

        Windows/WDDM'de her kernel çağrısının sabit maliyeti yüksek; grafik bunu tek
        çağrıya indirir. Dönen fonksiyon her çağrıda k güncelleme yapar ve son
        güncellemenin [loss_q, loss_pi, alpha, q_mean] tensörünü döner (GPU'da).
        """
        def step():
            out = None
            for _ in range(k):
                out = self._update(*rb.sample(batch))
            return out

        # optimizer durumları ve .grad tensörleri yakalamadan önce oluşsun
        s = torch.cuda.Stream()
        s.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(s):
            for _ in range(3):
                step()
        torch.cuda.current_stream().wait_stream(s)
        g = torch.cuda.CUDAGraph()
        with torch.cuda.graph(g):
            out = step()

        def run():
            g.replay()
            return out
        return run

    def state_dict(self):
        return {"actor": self.actor.state_dict(), "critic": self.critic.state_dict(),
                "critic_targ": self.critic_targ.state_dict(), "log_alpha": self.log_alpha.detach().cpu()}


def expand_obs_inputs(ck, old_obs, new_obs, act_dim):
    """v10: 32 girişli modelin ağırlıklarını new_obs girişe genişletir; yeni sütunlar sıfır →
    başlangıçta davranış birebir aynı. Aktör: net.0.weight [h, obs]; critic: q*.0.weight [h, obs+act]."""
    import torch as _t
    extra = new_obs - old_obs
    out = dict(ck)
    a = {k: v.clone() for k, v in ck["actor"].items()}
    w = a["net.0.weight"]
    a["net.0.weight"] = _t.cat([w, _t.zeros(w.shape[0], extra, dtype=w.dtype, device=w.device)], 1)
    out["actor"] = a
    for key in ("critic", "critic_targ"):
        if key not in ck:
            continue
        c = {k: v.clone() for k, v in ck[key].items()}
        for q in ("q1", "q2"):
            w = c[f"{q}.0.weight"]
            z = _t.zeros(w.shape[0], extra, dtype=w.dtype, device=w.device)
            c[f"{q}.0.weight"] = _t.cat([w[:, :old_obs], z, w[:, old_obs:]], 1)  # sıra: obs, [yeni], act
        out[key] = c
    return out


class ReplayBuffer:
    """Tamamı GPU'da duran halka tampon (2M geçiş ≈ 550 MB)."""

    def __init__(self, obs_dim, act_dim, size, device):
        self.o = torch.zeros((size, obs_dim), device=device)
        self.a = torch.zeros((size, act_dim), device=device)
        self.r = torch.zeros(size, device=device)
        self.o2 = torch.zeros((size, obs_dim), device=device)
        self.d = torch.zeros(size, device=device)
        self.size, self.ptr, self.n, self.device = size, 0, 0, device
        self.n_t = torch.zeros((), device=device)  # örnekleme için GPU'daki doluluk

    def add_batch(self, o, a, r, o2, d):
        k = len(r)
        idx = (torch.arange(k, device=self.device) + self.ptr) % self.size
        self.o[idx] = torch.as_tensor(o, device=self.device)
        self.a[idx] = torch.as_tensor(a, device=self.device)
        self.r[idx] = torch.as_tensor(r, device=self.device)
        self.o2[idx] = torch.as_tensor(o2, device=self.device)
        self.d[idx] = torch.as_tensor(d, device=self.device)
        self.ptr = (self.ptr + k) % self.size
        self.n = min(self.n + k, self.size)
        self.n_t.fill_(self.n)

    def sample(self, batch):
        # randint(0, n) yerine: n değişse de CUDA Graph içinde güvenli
        i = (torch.rand(batch, device=self.device) * self.n_t).long()
        return self.o[i], self.a[i], self.r[i], self.o2[i], self.d[i]
