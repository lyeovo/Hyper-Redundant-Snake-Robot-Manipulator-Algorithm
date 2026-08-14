function info = method_sa(model, q0, target, opts)
%method_sa 模拟退火（Metropolis 接受准则 + 几何冷却）
%   info = method_sa(model, q0, target, opts)
%   q0    : 初始关节角 [1×N]
%   target: 目标位姿 [x, y, θ]
%   opts  : .snapshot_m .onStep .isCancel .max_iter .alpha .sigma0
%           .T0（缺省按初始 V 自适应） .tol_pos .tol_ang
%
%   安全约束：扰动样本侵入障碍（min g < rho0）直接拒绝，不进入 Metropolis——
%   防止高温把臂推进屏障（方案 §5.2）
%   返回 info 结构同 method_momentum（q_snapshot/V_hist/q_final/success/error_code）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = of(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = of(opts, 'onStep', []);
    isCancel= of(opts, 'isCancel', []);
    max_iter= of(opts, 'max_iter', cfg.sa_max_iter);
    alpha   = of(opts, 'alpha', cfg.sa_alpha);
    sigma0  = of(opts, 'sigma0', cfg.sa_sigma0);
    tol_pos = of(opts, 'tol_pos', cfg.tol_pos);
    tol_ang = of(opts, 'tol_ang', cfg.tol_ang);

    q = q0(:)';
    q_best = q;  V_best = inf;
    V = armValue(model, q);
    T0 = of(opts, 'T0', max(V, 1e-3));
    T = T0;
    snap = zeros(0, cfg.N);  t_seq = [];  V_hist = zeros(1, max_iter);
    converged = false;  cancelled = false;  dist_end = inf;  err_ang = 0;
    N = cfg.N;

    for iter = 1:max_iter
        if ~isempty(isCancel) && isCancel()
            cancelled = true; break;
        end
        % 扰动幅度随温度收缩
        sigma = sigma0 * sqrt(T / T0);
        qp = q + sigma * randn(1, N);
        qp = max(cfg.q_min, min(cfg.q_max, qp));
        % 安全约束：侵入障碍直接拒绝
        [g_all, ~] = obsDistGradAll(model, qp);
        if ~isempty(g_all) && min(g_all) < cfg.rho0
            T = T * alpha;
            continue;
        end
        Vp = armValue(model, qp);
        dV = Vp - V;
        if dV < 0 || rand < exp(-dV / T)
            q = qp;  V = Vp;
            if V < V_best
                V_best = V;  q_best = q;
            end
        end
        T = T * alpha;
        V_hist(iter) = V_best;

        % 收敛检查（基于当前最优解）
        [~, p_end] = planarFK_L(q_best, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(q_best, model.DH, cfg.rod_offset_arr);
        dist_end = norm(target(1:2) - p_end);
        err_ang = abs(wrapAngle(target(3) - th));
        if dist_end < tol_pos && err_ang < tol_ang
            converged = true;
            break;
        end
        % 快照 + 回调
        if mod(iter, snapshot_m) == 0
            snap(end+1, :) = q_best; %#ok<AGROW>
            t_seq(end+1) = iter; %#ok<AGROW>
            if ~isempty(onStep), onStep(q_best, iter, dist_end); end
        end
    end
    if converged || cancelled
        snap(end+1, :) = q_best; %#ok<AGROW>
        t_seq(end+1) = iter; %#ok<AGROW>
    end
    V_hist = V_hist(1:iter);

    % 终点精修：SA 探索出好区域后，用无梯度随机贪心精修兜底（绕过梯度势阱）
    %   采样类方法成功语义 = 达到可达区域（rrt_goal_eps/ang），精确收敛由上层/后续梯度负责
    if ~converged && ~cancelled
        [q_rf, p_rf, a_rf] = refineRandomGreedy(model, q_best, target, ...
            struct('layers', 4, 'steps_per_layer', 250));
        if p_rf < cfg.rrt_goal_eps && a_rf < cfg.rrt_goal_ang
            q_best = q_rf;
            converged = true;
            dist_end = p_rf;
            err_ang = a_rf;
        end
    end

    info.q_snapshot = snap;
    info.t_seq = t_seq;
    info.V_hist = V_hist;
    info.q_final = q_best;
    info.success = converged && ~cancelled;
    info.converged = converged;
    info.cancelled = cancelled;
    info.iter = iter;
    info.dist_end = dist_end;
    info.err_ang = err_ang;
    if cancelled
        info.error_code = 6;
    elseif converged
        info.error_code = 0;
    else
        info.error_code = 2;
    end
    info.stats.T_final = T;
    info.stats.sigma_final = sigma;
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
