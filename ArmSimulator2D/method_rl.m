function info = method_rl(model, q0, target, opts)
%method_rl 强化学习（OpenAI-ES 参数扰动策略梯度 + residual 集成，方案 §5.5 L0/L2）
%   info = method_rl(model, q0, target, opts)
%
%   策略：π_θ(s) = θ·s'（线性），输出关节增量；动作 = 动量梯度方向 + λ·π_θ(s)
%   （residual 集成：θ 初始为 0 保证不劣于纯梯度）
%   状态特征 s = [e_pos(2), e_ang(1), min_g(1), q 归一化]（D 维）
%
%   opts:
%     .train      训练模式（默认 true）：在任务上 ES 更新 θ
%     .theta0     初始策略参数（默认零矩阵 → residual 从纯梯度起步）
%     .n_pop      扰动种群数（默认 16）
%     .n_gen      进化代数（默认 20）
%     .sigma_es   扰动标准差（默认 0.2）
%     .lambda     residual 权重（默认 0.3，随训练缩放）
%     .max_rollout rollout 步数（默认 60）
%     .snapshot_m .onStep .isCancel
%
%   返回 info 结构同各方法；info.stats.theta 为学得策略（可导出复用）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = optget(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = optget(opts, 'onStep', []);
    isCancel= optget(opts, 'isCancel', []);
    train   = of2(opts, {'train','Train'}, true);
    n_pop   = of2(opts, {'n_pop','NPop'}, 16);
    n_gen   = of2(opts, {'n_gen','NGen'}, 20);
    sig     = of2(opts, {'sigma_es','SigmaES'}, 0.2);
    lam     = of2(opts, {'lambda','Lambda'}, 0.3);
    max_roll= of2(opts, {'max_rollout','MaxRollout'}, 60);
    N = cfg.N;

    % ---- 训练（OpenAI-ES，任务内） ----
    theta = of2(opts, {'theta0','Theta0'}, zeros(N, 2+1+1+N));
    if train
        rng(1);                                     % 可复现
        for gen = 1:n_gen
            if ~isempty(isCancel) && isCancel(), break; end
            eps = sig * randn(n_pop, numel(theta));
            R = zeros(n_pop, 1);
            for p = 1:n_pop
                th_p = theta + reshape(eps(p,:), size(theta));
                [d, a, ~, ~] = rlRollout(model, q0, target, th_p, lam, max_roll);
                R(p) = -(d + 0.1*a);                % 奖励 = 负末端误差
            end
            R = (R - mean(R)) / max(std(R), 1e-6);
            grad_theta = (eps' * R) / (n_pop * sig);
            theta = theta + 0.5 * reshape(grad_theta, size(theta));
        end
    end

    % ---- 推理（用学得策略 rollout） ----
    [dist_end, err_ang, q_final, q_hist] = rlRollout(model, q0, target, theta, lam, max_roll);

    % ---- 组装 ----
    if ~isempty(q_hist) && size(q_hist,1) >= snapshot_m
        snap = q_hist(1:snapshot_m:end, :);
        snap(end+1, :) = q_final;
        t_seq = (1:size(snap,1)) * snapshot_m;
    else
        snap = q_final;
        t_seq = 1;
    end
    info.q_snapshot = snap;
    info.t_seq = t_seq;
    info.V_hist = [];
    info.q_final = q_final;
    info.success = dist_end < cfg.rrt_goal_eps && err_ang < cfg.rrt_goal_ang;
    info.converged = info.success;
    info.cancelled = false;
    info.iter = max_roll;
    info.dist_end = dist_end;
    info.err_ang = err_ang;
    if info.success, info.error_code = 0; else, info.error_code = 2; end
    info.stats.theta = theta;
    info.stats.lambda = lam;
    info.stats.trained = train;
end

function v = of2(s, names, default)
    % 双写别名读取（如 {'n_pop','NPop'}）
    for i = 1:numel(names)
        if isfield(s, names{i}) && ~isempty(s.(names{i}))
            v = s.(names{i});
            return;
        end
    end
    v = default;
end
