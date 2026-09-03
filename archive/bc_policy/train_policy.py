#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
train_policy.py — L2 模仿学习训练端（BC 基线 + CVAE 扩展点）

数据流（维度无关）：
    demonstrations/*.mat（generateDemonstrations 产出，schema demo-v1）
        → 预处理（变长轨迹重采样到固定 T 点 + 障碍栅格化）
        → 归一化 → BC MLP 训练（PyTorch CPU）
        → 导出 policy_bc.npz（权重 + 归一化参数，供 MATLAB 手写 forward 评估）

特征编码（2D 首版；3D 演进：栅格→体素/点云编码，其余不变）：
    x = [occupancy_grid(64×64 展平) | q0(归一化) | target(x,y,θ 归一化)]

输出：固定 T=64 的关节轨迹 [T×N]，部署时插值到快照密度。

说明：
- 本脚本是 BC 单值回归基线——用于验证整条管线（数据→训练→导出→评估）。
  多模态（绕左/绕右分叉）需 CVAE/Diffusion（见下方 CVAE 扩展点注释）。
- 评估在 MATLAB 侧（evaluatePolicy.m 有运动学/碰撞/基线对比）：
  MATLAB 加载 policy_bc.npz 后手写 MLP forward 作为策略句柄即可接入。

用法：
    python rl_pipeline/train_policy.py --data demonstrations --epochs 30
"""
import argparse
import glob
import os

import numpy as np
import scipy.io as sio
import torch
import torch.nn as nn

# ---------- 1. 数据加载与预处理 ----------

GRID = 64
EXTENT = (-5.0, 9.0, -7.0, 7.0)   # 工作空间（6 臂 × 1.04393m：可达域半径 6.26，宽 14 含余量；与 train_cvae.py 一致）
T_OUT = 64                          # 轨迹重采样点数
# N 从数据读（Y.shape[2]），不硬编码


def rasterize_obs(circles, rects):
    """圆/矩形 → GRID×GRID 占据图（2D；3D 换体素/点云编码）"""
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
    """变长轨迹 → 固定 T 点（线性重采样，含首末点）"""
    n = len(traj)
    if n == 1:
        return np.tile(traj[0], (T, 1)).astype(np.float32)
    idx = np.linspace(0, n - 1, T)
    x = np.arange(n)
    out = np.stack([np.interp(idx, x, traj[:, j]) for j in range(traj.shape[1])], axis=1)
    return out.astype(np.float32)


def load_demos(data_dir):
    """读取 demonstrations/ 分片 .mat → (X, Y)"""
    files = sorted(glob.glob(os.path.join(data_dir, "demo_*.mat")))
    if not files:
        raise FileNotFoundError(f"no demo files in {data_dir}")
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


# ---------- 2. 模型 ----------

class BCMLP(nn.Module):
    """条件 MLP：x → 轨迹 [T_OUT×N]（展平输出）。CVAE 扩展点：把本模型换成
    encoder(z|x) + decoder(x,z)→traj，加 KL 项，即可多模态。"""

    def __init__(self, d_in, d_out, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(d_in, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, d_out),
        )

    def forward(self, x):
        return self.net(x)


# ---------- 3. 训练 ----------

def train(args):
    # 可复现：torch/numpy 同种子（与 train_cvae.py 一致；split 已用 RandomState(args.seed)）
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    (Xg, Xq, Xt), Y = load_demos(args.data)
    n = len(Y)
    N = Y.shape[2]
    print(f"[data] {n} demos | grid {Xg.shape} q {Xq.shape} tgt {Xt.shape} traj {Y.shape}")

    # 归一化（栅格已 0/1，不归一）
    q_m, q_s = Xq.mean(0), Xq.std(0) + 1e-6
    t_m, t_s = Xt.mean(0), Xt.std(0) + 1e-6
    # 增量表示（同 train_cvae）：起点由 q0 锚定，网络只学运动形状
    Yd = np.diff(np.concatenate([Xq[:, None, :], Y], axis=1), axis=1)
    Y_m, Y_s = Yd.mean(0), Yd.std(0) + 1e-6
    Xq_n = (Xq - q_m) / q_s
    Xt_n = (Xt - t_m) / t_s
    Y_n = (Yd - Y_m) / Y_s

    # 划分（按任务随机，防泄漏）
    perm = np.random.RandomState(args.seed).permutation(n)
    n_tr = int(n * 0.9)
    tr, va = perm[:n_tr], perm[n_tr:]

    def to_t(x):
        return torch.tensor(x, dtype=torch.float32)

    Xtr = torch.cat([to_t(Xg[tr]).flatten(1), to_t(Xq_n[tr]), to_t(Xt_n[tr])], dim=1)
    Ytr = to_t(Y_n[tr]).flatten(1)
    Xva = torch.cat([to_t(Xg[va]).flatten(1), to_t(Xq_n[va]), to_t(Xt_n[va])], dim=1)
    Yva = to_t(Y_n[va]).flatten(1)

    d_in = Xtr.shape[1]
    d_out = Ytr.shape[1]
    model = BCMLP(d_in, d_out, hidden=args.hidden)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    lossf = nn.MSELoss()

    for ep in range(args.epochs):
        model.train()
        opt.zero_grad()
        loss = lossf(model(Xtr), Ytr)
        loss.backward()
        opt.step()
        if (ep + 1) % 5 == 0 or ep == args.epochs - 1:
            model.eval()
            with torch.no_grad():
                lv = lossf(model(Xva), Yva).item()
            print(f"[train] ep {ep+1}/{args.epochs} loss {loss.item():.4f} val {lv:.4f}")

    # 导出（供 MATLAB 手写 forward 评估）
    npz = dict(q_mean=q_m, q_std=q_s, t_mean=t_m, t_std=t_s,
               y_mean=Y_m, y_std=Y_s, T=T_OUT, N=N,
               grid=GRID, extent=np.array(EXTENT), dtype="float32", delta=True)
    sd = model.state_dict()
    for k, v in sd.items():
        npz[f"w_{k}"] = v.numpy()
    out = args.out
    np.savez(out, **npz)
    print(f"[save] {out} ({os.path.getsize(out)/1024:.0f} KB)")
    # 双导出 .mat（MATLAB 评估端 load 直接读，无需 Python 运行时）
    mat_path = out.replace(".npz", ".mat")
    mat_dict = {}
    for k, v in npz.items():
        if k in ("extent", "dtype"):
            continue
        mat_dict[k.replace(".", "_")] = v   # PyTorch state_dict 键 → MATLAB 合法字段名
    mat_dict["extent"] = np.array(EXTENT, dtype=np.float64)
    mat_dict["dtype"] = "float32"
    sio.savemat(mat_path, mat_dict, do_compression=True)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="demonstrations")
    ap.add_argument("--out", default="rl_pipeline/policy_bc.npz")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    train(args)
