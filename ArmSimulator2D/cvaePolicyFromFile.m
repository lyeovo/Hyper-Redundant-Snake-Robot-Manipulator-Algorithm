function out = cvaePolicyFromFile(mat_file, model, q0, target, n_cand, z_iters)
%cvaePolicyFromFile 加载 CVAE 策略（train_cvae.py 导出的 .mat）推理一条轨迹
%   out = cvaePolicyFromFile(mat_file, model, q0, target, n_cand, z_iters)
%   n_cand : 潜变量初始化候选数（默认 4；多模态绕行）
%   z_iters: 潜空间优化步数（默认 25；对 z 梯度下降最小化终态误差——解决
%            KL 塌缩下"训练后验 vs 部署先验"分布不匹配，见方案 §5.7.4）
%   out    : struct('traj', [T×N], 'q_final', [1×N])（evaluatePolicy 策略句柄格式）
%
%   推理 = 特征编码 + z 初始化采样 + 潜空间数值梯度优化 + 终态择优。
%   数值差分 dz 维（CPU 下 ~1s/候选）；3D 演进仅换 FK/误差函数。
    if nargin < 5 || isempty(n_cand), n_cand = 4; end
    if nargin < 6 || isempty(z_iters), z_iters = 25; end
    S = load(mat_file);
    N = S.N;  T = S.T;  GRID = S.grid;  EXTENT = S.extent(:)';
    dz = S.dz;
    cfg = model.cfg;
    if cfg.N ~= N
        error('cvaePolicyFromFile:dim', '策略 N=%d 与模型 N=%d 不匹配（需重新训练对应 N 的策略）', N, cfg.N);
    end

    img = rasterize2D(cfg.obstacles.circles, cfg.obstacles.rects, GRID, EXTENT);
    % 展平顺序须与 train_cvae.py 的 .flatten()（行主序）一致：用 reshape(img.',:) 而非 img(:)（列主序）
    x = [reshape(img.', 1, []), (q0(:)' - S.q_mean) ./ S.q_std, (target(:)' - S.t_mean) ./ S.t_std];

    b0 = S.dec_0_bias(:)';  b2 = S.dec_2_bias(:)';  b4 = S.dec_4_bias(:)';

    function [err, traj] = decodeEval(z)
        % decoder forward + 增量重构 + 终态误差（2D 适配）
        h = [x, z] * S.dec_0_weight' + b0;
        h = max(h, 0);
        h = h * S.dec_2_weight' + b2;
        h = max(h, 0);
        y = h * S.dec_4_weight' + b4;
        delta = reshape(y, T, N) .* S.y_std + S.y_mean;   % 增量 Δ
        traj = q0(:)' + cumsum(delta, 1);                 % traj = q0 + cumsum(Δ)
        traj = max(cfg.q_min, min(cfg.q_max, traj));
        [~, pe] = planarFK_L(traj(end,:), model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(traj(end,:), model.DH, cfg.rod_offset_arr);
        err = norm(pe - target(1:2)) + 0.1*abs(wrapAngle(target(3) - th));
    end

    best = struct('traj', [], 'q_final', q0(:)', 'err', inf);
    for c = 1:n_cand
        z = randn(1, dz);
        lr = 0.15;
        for it = 1:z_iters
            [e0, ~] = decodeEval(z);
            % 中心差分梯度（dz 维，代价 ~2·dz 次前向）
            g = zeros(1, dz);
            hh = 0.05;
            for j = 1:dz
                zp = z;  zp(j) = z(j) + hh;  [ep, ~] = decodeEval(zp);
                zm = z;  zm(j) = z(j) - hh;  [em, ~] = decodeEval(zm);
                g(j) = (ep - em) / (2*hh);
            end
            z = z - lr * g;
        end
        [e, tr] = decodeEval(z);
        if e < best.err
            best = struct('traj', tr, 'q_final', tr(end,:), 'err', e);
        end
    end
    % 部署闭环：终点无梯度精修（策略粗生成 → 精修达 tol，见 §5.7.6）
    % 多起点精修：从粗轨迹末端与 1/3 处出发，取终态最优（绕开局部势阱）
    if ~isempty(best.traj)
        starts = {best.traj(end,:)};
        if size(best.traj,1) >= 3
            starts{2} = best.traj(round(size(best.traj,1)/3), :);
        end
        q_best = best.traj(end,:);  err_best = inf;
        for si = 1:numel(starts)
            [qf, pe, ae] = refineRandomGreedy(model, starts{si}, target, ...
                struct('layers', 3, 'steps_per_layer', 120));
            e = norm(pe - target(1:2)) + 0.1*ae;
            if e < err_best
                err_best = e;  q_best = qf;
            end
        end
        best.q_final = q_best;
    end
    out = struct('traj', best.traj, 'q_final', best.q_final);
end
