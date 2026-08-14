function out = bcPolicyFromFile(mat_file, model, q0, target)
%bcPolicyFromFile 加载 BC 策略（train_policy.py 导出的 .mat）并推理一条轨迹
%   out = bcPolicyFromFile(mat_file, model, q0, target)
%   out : struct('traj', [T×N], 'q_final', [1×N])（evaluatePolicy 策略句柄格式）
%
%   与 train_policy.py 的栅格化/归一化/网络结构严格一致（GRID/EXTENT/N/T 读自 .mat；层结构固定为 3 层 MLP，hidden 256）。
%   3D 演进：替换 rasterize 与特征拼接（策略文件格式不变）。
    S = load(mat_file);
    N = S.N;  T = S.T;  GRID = S.grid;  EXTENT = S.extent(:)';

    % ---- 特征：occupancy grid（与 train_policy.py rasterize_obs 一致） ----
    cfg = model.cfg;
    if cfg.N ~= N
        error('bcPolicyFromFile:dim', '策略 N=%d 与模型 N=%d 不匹配（需重新训练对应 N 的策略）', N, cfg.N);
    end
    img = rasterize2D(cfg.obstacles.circles, cfg.obstacles.rects, GRID, EXTENT);
    % ---- 归一化 ----
    q_n  = (q0(:)' - S.q_mean) ./ S.q_std;
    t_n  = (target(:)' - S.t_mean) ./ S.t_std;
    % 展平顺序须与 train_policy.py 的 .flatten()（行主序）一致：用 reshape(img.',:) 而非 img(:)（列主序）
    x = [reshape(img.', 1, []), q_n, t_n];
    % ---- MLP forward（读 .mat 中的 state_dict，键 w_net.0.weight 等） ----
    b0 = S.w_net_0_bias(:)';  b2 = S.w_net_2_bias(:)';  b4 = S.w_net_4_bias(:)';
    h = x(:)' * S.w_net_0_weight' + b0;                 % [1×256]
    h = max(h, 0);                                      % ReLU
    h = h * S.w_net_2_weight' + b2;
    h = max(h, 0);
    y = h * S.w_net_4_weight' + b4;                     % [1×(T*N)]
    % ---- 反归一化 → 增量 → 轨迹（起点由 q0 锚定） ----
    delta = reshape(y, T, N) .* S.y_std + S.y_mean;
    traj = q0(:)' + cumsum(delta, 1);
    traj = max(cfg.q_min, min(cfg.q_max, traj));        % 限位投影
    % 部署闭环：终点无梯度精修（策略粗生成 → 精修达 tol，见 §5.7.6）
    [qf, ~, ~] = refineRandomGreedy(model, traj(end,:), target, ...
        struct('layers', 2, 'steps_per_layer', 100));
    out = struct('traj', traj, 'q_final', qf);
end
