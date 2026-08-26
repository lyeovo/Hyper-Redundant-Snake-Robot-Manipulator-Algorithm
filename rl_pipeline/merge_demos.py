#!/usr/bin/env python3
"""合并多个示范分片目录 → 单一训练目录（shuffle + 统一重命名 + 重写 manifest）"""
import argparse, glob, os, random, shutil
import numpy as np

def load_mat(path):
    import scipy.io as sio
    d = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    demos = d['demos']
    out = []
    for it in demos:
        dd = it
        traj = np.asarray(dd.traj, dtype=np.float64)
        q0 = np.asarray(dd.q0, dtype=np.float64).ravel()
        tgt = np.asarray(dd.target, dtype=np.float64).ravel()
        obs = dd.obstacles
        circ = np.asarray(getattr(obs, 'circles', np.zeros((0, 3))), dtype=np.float64)
        rect = np.asarray(getattr(obs, 'rects', np.zeros((0, 5))), dtype=np.float64)
        out.append({
            'q0': q0, 'target': tgt, 'traj': traj,
            'cost': float(dd.cost),
            'circles': circ, 'rects': rect,
        })
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_dirs', nargs='+')
    ap.add_argument('--out', default='data/demonstrations_opt')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--n_max', type=int, default=0, help='0=全部')
    args = ap.parse_args()
    random.seed(args.seed); np.random.seed(args.seed)

    all_demos = []
    for sd in args.src_dirs:
        for f in sorted(glob.glob(os.path.join(sd, 'demo_*.mat'))):
            all_demos.extend(load_mat(f))
    print(f'共加载 {len(all_demos)} 条示范')
    if args.n_max and len(all_demos) > args.n_max:
        all_demos = random.sample(all_demos, args.n_max)
        print(f'随机抽取 {args.n_max} 条')
    random.shuffle(all_demos)

    os.makedirs(args.out, exist_ok=True)
    PER = 100
    n_shard = (len(all_demos) + PER - 1) // PER
    import scipy.io as sio
    for s in range(n_shard):
        chunk = all_demos[s*PER:(s+1)*PER]
        # 用 MATLAB 兼容的 cell 结构
        demos = []
        for d in chunk:
            demos.append({
                'q0': d['q0'].reshape(1, -1),
                'target': d['target'].reshape(1, -1),
                'traj': d['traj'],
                'cost': d['cost'],
                'obstacles': {
                    'circles': d['circles'].reshape(-1, 3) if d['circles'].size else np.zeros((0, 3)),
                    'rects': d['rects'].reshape(-1, 5) if d['rects'].size else np.zeros((0, 5)),
                },
            })
        sio.savemat(os.path.join(args.out, f'demo_{s+1:04d}.mat'),
                    {'demos': demos}, do_compression=True)
    # manifest
    sio.savemat(os.path.join(args.out, 'manifest.mat'), {
        'n_total': len(all_demos), 'schema_version': 1,
        'merged_from': ';'.join(args.src_dirs),
    })
    print(f'写出 {n_shard} 个分片到 {args.out}（共 {len(all_demos)} 条）')

if __name__ == '__main__':
    main()
