#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
train_cvae.py — CVAE 多模态轨迹生成训练（L2 M3 主体）

结构（维度无关，数据格式 demo-v1 不变）：
    encoder:  [grid | q0 | target | traj] → μ, log σ (潜空间 d_z)
    decoder:  [grid | q0 | target | z]    → traj        （z ~ N(0,I) 采样 → 多模态绕行）
    训练：ELBO = MSE_recon + β·KL(N(μ,σ²)||N(0,1))

导出 policy_cvae.mat（MATLAB cvaePolicyFromFile.m 手写 decoder forward + z 采样）：
    归一化参数 + decoder 权重（键 w_dec_0_weight 等）+ d_z/GRID/EXTENT/N/T。

用法：
    python rl_pipeline/train_cvae.py --data data/demonstrations_n6 --out rl_pipeline/policy_cvae --epochs 60
"""
import argparse
import glob
import os
import random

import numpy as np
import scipy.io as sio
import torch
import torch.nn as nn

# ---- 云/GPU 迁移就绪：设备自动选择（cuda / mps / cpu） ----
DEVICE = torch.device("cuda" if torch.cuda.is_available() else (
    "mps" if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available() else "cpu"))
if DEVICE.type != "cpu":
    print(f"[device] 使用 {DEVICE} 加速")

GRID = 64
EXTENT = (-5.0, 9.0, -7.0, 7.0)   # 工作空间（6 臂 × 1.04393m：可达域半径 6.26，宽 14 含余量；64 格 ≈ 0.22m/格）
T_OUT = 64
D_Z = 16                            # 潜空间维度
BETA = 0.5                          # KL 权重（首版保守，保重建质量）


def rasterize_obs(circles, rects):
    xmin, xmax, ymin, ymax = EXTENT
    img = np.zeros((GRID, GRID), dtype=np.float32)
    yy, xx = np.mgrid[0:GRID, 0:GRID]
    gx = xmin + (xx + 0.5) * (xmax - xmin) / GRID
    gy = ymin + (yy + 0.5) * (ymax - ymin) / GRID
    for c in np.atleast_2d(circles):
        if c.size < 3:
            continue
        cx, cy, r = c[0], c[1], c[2]
        img[(gx - cx) ** 2 + (gy - cy) ** 2 <= r ** 2] = 1.0
    for r_ in np.atleast_2d(rects):
        if r_.size < 5:
            continue
        cx, cy, th, w, h = r_[0], r_[1], r_[2], r_[3], r_[4]
        ct, st = np.cos(th), np.sin(th)
        lx = (gx - cx) * ct + (gy - cy) * st
        ly = -(gx - cx) * st + (gy - cy) * ct
        img[(np.abs(lx) <= w / 2) & (np.abs(ly) <= h / 2)] = 1.0
    return img


def resample_traj(traj, T):
    n = len(traj)
    if n == 1:
        return np.tile(traj[0], (T, 1)).astype(np.float32)
    idx = np.linspace(0, n - 1, T)
    x = np.arange(n)
    out = np.stack([np.interp(idx, x, traj[:, j]) for j in range(traj.shape[1])], axis=1)
    return out.astype(np.float32)


def load_demos(data_dir):
    # 支持单目录或多目录（合并加载 + 全局按文件乱序，保证混合）
    if isinstance(data_dir, str):
        data_dir = [data_dir]
    files = []
    for dd in data_dir:
        fs = sorted(glob.glob(os.path.join(dd, "demo_*.mat")))
        if fs:
            files.extend(fs)
    if not files:
        raise FileNotFoundError(f"no demo files in {data_dir}")
    random.Random(0).shuffle(files)
    Xg, Xq, Xt, Y = [], [], [], []
    for f in files:
        d = sio.loadmat(f)
        for demo in d["demos"].ravel():
            obs = demo["obstacles"][0, 0]
            circ = obs["circles"] if "circles" in obs.dtype.names else np.zeros((0, 3))
            rect = obs["rects"] if "rects" in obs.dtype.names else np.zeros((0, 5))
            Xg.append(rasterize_obs(circ, rect))
            Xq.append(np.asarray(demo["q0"].ravel(), dtype=np.float32))
            Xt.append(np.asarray(demo["target"].ravel(), dtype=np.float32))
            Y.append(resample_traj(demo["traj"], T_OUT))
    return (np.stack(Xg), np.stack(Xq), np.stack(Xt)), np.stack(Y)


class CVAE(nn.Module):
    def __init__(self, d_in, d_traj, d_z=D_Z, hidden=256):
        super().__init__()
        self.d_z = d_z
        self.enc = nn.Sequential(
            nn.Linear(d_in + d_traj, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, d_z * 2),          # μ, logσ
        )
        self.dec = nn.Sequential(
            nn.Linear(d_in + d_z, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, d_traj),
        )

    def encode(self, x, y):
        h = self.enc(torch.cat([x, y], dim=1))
        mu, logvar = h.chunk(2, dim=1)
        return mu, logvar

    def reparam(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, x, z):
        return self.dec(torch.cat([x, z], dim=1))

    def forward(self, x, y):
        mu, logvar = self.encode(x, y)
        z = self.reparam(mu, logvar)
        rec = self.decode(x, z)
        kl = -0.5 * torch.sum(1 + logvar - mu ** 2 - logvar.exp(), dim=1).mean()
        return rec, kl

    def sample(self, x, n=1):
        """推理：z ~ N(0,I)，多次采样取多模态候选"""
        z = torch.randn(n, self.d_z)
        if n == 1:
            return self.decode(x, z)
        return self.decode(x.repeat(n, 1), z)


def train(args):
    # 完整可复现：torch/numpy/random 三源同种子
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    torch.backends.cudnn.deterministic = True
    (Xg, Xq, Xt), Y = load_demos(args.data)
    n = len(Y)
    N = Y.shape[2]
    print(f"[data] {n} demos | grid {Xg.shape} q {Xq.shape} tgt {Xt.shape} traj {Y.shape}")

    q_m, q_s = Xq.mean(0), Xq.std(0) + 1e-6
    t_m, t_s = Xt.mean(0), Xt.std(0) + 1e-6
    # 增量表示：Δ[k] = traj[k] - traj[k-1]（首行 = traj[0] - q0 = 0）
    # 起点由 q0 锚定（traj = q0 + cumsum(Δ)），网络只学运动形状
    Yd = np.diff(np.concatenate([Xq[:, None, :], Y], axis=1), axis=1)
    Y_m, Y_s = Yd.mean(0), Yd.std(0) + 1e-6
    if args.init:   # 微调：沿用 init 模型的归一化统计量，保持权重-输入一致性
        d0 = sio.loadmat(args.init)
        q_m, q_s = d0["q_mean"], d0["q_std"]
        t_m, t_s = d0["t_mean"], d0["t_std"]
        Y_m, Y_s = d0["y_mean"], d0["y_std"]
        print(f"[init] 归一化沿用 {args.init}")
    Xq_n = (Xq - q_m) / q_s
    Xt_n = (Xt - t_m) / t_s
    Y_n = (Yd - Y_m) / Y_s

    perm = np.random.RandomState(args.seed).permutation(n)
    n_tr = int(n * 0.9)
    tr, va = perm[:n_tr], perm[n_tr:]

    def to_t(x):
        return torch.tensor(x, dtype=torch.float32)

    Xtr = torch.cat([to_t(Xg[tr]).flatten(1), to_t(Xq_n[tr]), to_t(Xt_n[tr])], dim=1).to(DEVICE)
    Ytr = to_t(Y_n[tr]).flatten(1).to(DEVICE)
    Xva = torch.cat([to_t(Xg[va]).flatten(1), to_t(Xq_n[va]), to_t(Xt_n[va])], dim=1).to(DEVICE)
    Yva = to_t(Y_n[va]).flatten(1).to(DEVICE)

    d_in = Xtr.shape[1]
    d_traj = Ytr.shape[1]
    model = CVAE(d_in, d_traj, d_z=args.dz, hidden=args.hidden).to(DEVICE)
    if args.init:
        d0 = sio.loadmat(args.init)
        sd0 = {}
        for k in d0:
            if k.startswith("__") or k in ("q_mean", "q_std", "t_mean", "t_std",
                                           "y_mean", "y_std", "T", "N", "grid",
                                           "extent", "dz", "dtype", "delta"):
                continue
            sd0[k.replace("_", ".")] = torch.tensor(np.ascontiguousarray(d0[k])).squeeze().to(DEVICE)
        model.load_state_dict(sd0)
        print(f"[init] 权重加载完成（{len(sd0)} 个张量）")
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    lossf = nn.MSELoss()

    for ep in range(args.epochs):
        model.train()
        opt.zero_grad()
        rec, kl = model(Xtr, Ytr)
        loss = lossf(rec, Ytr) + BETA * kl
        loss.backward()
        opt.step()
        if (ep + 1) % 10 == 0 or ep == args.epochs - 1:
            model.eval()
            with torch.no_grad():
                recv, klv = model(Xva, Yva)
                lv = lossf(recv, Yva).item()
            print(f"[train] ep {ep+1}/{args.epochs} recon {lossf(rec, Ytr).item():.4f} "
                  f"KL {kl.item():.3f} | val {lv:.4f}")

    # 导出 .mat（MATLAB cvaePolicyFromFile 手写 forward）
    # y_mean/y_std 为增量 Δ 的；部署: traj = q0 + cumsum(Δ)
    mat = dict(q_mean=q_m, q_std=q_s, t_mean=t_m, t_std=t_s,
               y_mean=Y_m, y_std=Y_s, T=T_OUT, N=N, grid=GRID,
               extent=np.array(EXTENT, dtype=np.float64), dz=D_Z,
               dtype="float32", delta=True)
    # 权重转回 CPU 再导出
    sd = model.to("cpu").state_dict()
    for k, v in sd.items():
        mat[k.replace(".", "_")] = v.numpy()
    out = args.out
    mat_path = out if out.endswith(".mat") else out + ".mat"
    sio.savemat(mat_path, mat, do_compression=True)
    print(f"[save] {mat_path} ({os.path.getsize(mat_path)/1024:.0f} KB)")
    return mat_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", nargs="+", default=["data/demonstrations_n6"])
    ap.add_argument("--out", default="rl_pipeline/policy_cvae.mat")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--init", default=None, help="加载已有 .mat 权重续训（微调）")
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--dz", type=int, default=D_Z)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    train(args)
