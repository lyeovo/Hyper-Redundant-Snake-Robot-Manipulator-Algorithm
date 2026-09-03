function info = method_momentum(model, q0, target, opts)
%method_momentum 动量法（统一价值函数梯度下降 + 动量累积 + 自适应步长）
%   info = method_momentum(model, q0, target, opts)
%   q0    : 初始关节角 [1×N]
%   target: 目标位姿 [x, y, θ]（覆盖 model.cfg 默认目标）
%   opts  : struct（可缺省）
%           .snapshot_m  快照间隔（每 m 步记录一行 q）
%           .onStep      回调 @(q, iter, dist_end)（每 snapshot_m 步触发）
%           .isCancel    取消检查 @() bool（每步触发，true 则中断）
%           .max_iter / .tol_pos / .tol_ang  覆盖模型默认
%           .verbose     打印开关（默认 false）
%
%   返回 info：.q_snapshot [K×N] .t_seq [1×K] .V_hist [1×T] .q_final
%             .success .converged .cancelled .iter .dist_end .err_ang
%             .error_code .stats(含 dt_mean)
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = optget(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = optget(opts, 'onStep', []);
    isCancel= optget(opts, 'isCancel', []);
    max_iter= optget(opts, 'max_iter', cfg.max_iter);
    tol_pos = optget(opts, 'tol_pos', cfg.tol_pos);
    tol_ang = optget(opts, 'tol_ang', cfg.tol_ang);
    verbose = optget(opts, 'verbose', false);

    q = q0(:)';
    X_t = target(1:2);  th_t = target(3);
    v = zeros(1, cfg.N);
    snap = zeros(0, cfg.N);
    t_seq = [];
    V_hist = zeros(1, max_iter);
    t_accum = 0;
    converged = false;  cancelled = false;  stalled = false;
    % 无进展早停：窗口趋势判据（见循环内），最近 3 窗口 vs 前 3 窗口最佳
    STALL_N = 30;
    STALL_TOL = 1e-4;   % 180 步内窗口最佳位置误差改善阈值（m）
    win_errs = [];
    dist_end = inf;  err_ang = 0;

    % 动量法用动量本身做平滑，显式加速度正则项关闭（避免 V 判据与速度耦合）
    m_eff = model;
    m_eff.cfg.w_acc = 0;

    % 两阶段收敛状态：硬达标（tol）立即收敛；初步达标（宽松）后二次精修，
    % 目标函数逐项贡献（位置/角度/障碍）变化 < 0.001 连续 5 步 → 收敛（动态精度）
    % strict=true（默认）：只认硬达标（1e-4/1e-3），精修兜底关闭——简单场景保持高精度，
    %   到不了硬达标则正常失败（走 auto 链换方法）
    % strict=false：动态收敛（逐项 < 0.001 即收敛，精度 ~0.03）
    strict = optget(opts, 'strict', true);
    phase2 = false;  phase2_stall = 0;
    v_p_prev = inf;  v_a_prev = inf;  v_o_prev = inf;
    conv_type = 'hard';

    for iter = 1:max_iter
        [~, p_end] = planarFK_L(q, m_eff.DH, m_eff.cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(q, m_eff.DH, m_eff.cfg.rod_offset_arr);
        e_pos = X_t - p_end;
        e_ang = wrapAngle(th_t - th);
        dist_end = norm(e_pos);
        % 硬达标：固定阈值立即收敛（无障碍/易达场景保持高精度）
        if dist_end < tol_pos && abs(e_ang) < tol_ang
            converged = true;
            conv_type = 'hard';
            break;
        end
        % 二次精修阶段（strict=false 时启用；strict=true 只认硬达标）
        if ~strict && dist_end < 0.2
            phase2 = true;
        end
        if phase2
            % 目标函数逐项贡献（用当前两阶段调度权重）
            v_p = m_eff.cfg.w_pos * dist_end^2;
            v_a = m_eff.cfg.w_ang * e_ang^2;
            v_o = 0;
            [~, g_all] = obsDistGradAll(m_eff, q);
            if ~isempty(g_all)
                gmin = min(g_all);
                if gmin < cfg.barrier_range
                    v_o = cfg.w_obs * (log(cfg.barrier_range) - log(max(gmin, cfg.g_min)));
                end
            end
            d_items = abs([v_p - v_p_prev, v_a - v_a_prev, v_o - v_o_prev]);
            if all(d_items < 0.001)
                phase2_stall = phase2_stall + 1;
            else
                phase2_stall = 0;
            end
            v_p_prev = v_p;  v_a_prev = v_a;  v_o_prev = v_o;
            % 动态收敛门槛：逐项停滞 5 步 **且 末端位置/角度已实际接近目标** 才判收敛
            % （防止势阱处各项变化小但误差仍大时误报"已收敛"）
            if phase2_stall >= 5 && dist_end < 0.05 && abs(e_ang) < 0.2
                converged = true;
                conv_type = 'item';
                break;
            end
        end
        if ~isempty(isCancel) && isCancel()
            cancelled = true;
            break;
        end

        V_cur = armValue(m_eff, q);
        [g_all, ~] = obsDistGradAll(m_eff, q);
        % —— 两阶段权重调度（打破位置-角度互消） ——
        % 阶段1（远）：位置主导，角度弱化让臂先到位
        % 阶段2（近）：位置粗达后角度主导，用冗余自由度调姿
        % 两阶段在 dist_end = 0.08 处权重连续
        if dist_end > 0.08
            m_eff.cfg.w_ang = cfg.w_ang * 0.2;
            m_eff.cfg.w_pos = cfg.w_pos * 1.0;
        else
            s2 = min(1, dist_end / 0.08);   % 1→0 当 dist: 0.08→0
            m_eff.cfg.w_ang = cfg.w_ang * (0.2 + 3.8*(1 - s2));
            m_eff.cfg.w_pos = cfg.w_pos * (0.3 + 0.7*s2);
        end
        grad = armGradient(m_eff, q, [], []);
        % 动量系数近目标衰减（erf 平滑）：dist→0 时 β→0，消除目标附近震荡
        beta_eff = m_eff.cfg.momentum_beta * erf(dist_end / (sqrt(2) * 0.05));
        v = beta_eff * v + (1 - beta_eff) * (-grad);

        % 靠近障碍时降速（λ(g) 缩放），否则全速（受 dq_max 约束）
        if ~isempty(g_all)
            lambda_g = max(1, m_eff.cfg.rho_critical / max(min(g_all), m_eff.cfg.g_min));
            v = v / lambda_g;
        end

        % 步长上限 + 回溯线搜索（V 单调下降，防震荡/伪平衡）
        over = max(abs(v)) / m_eff.cfg.dq_max;
        if over > 1, v = v / over; end
        q_try = q + v;
        q_try = max(m_eff.cfg.q_min, min(m_eff.cfg.q_max, q_try));
        for k = 1:6
            if armValue(m_eff, q_try) <= V_cur + 1e-12
                break;
            end
            v = v * 0.5;
            q_try = q + v;
            q_try = max(m_eff.cfg.q_min, min(m_eff.cfg.q_max, q_try));
        end
        q = q_try;

        % —— 无进展早停：窗口趋势判据 ——
        % 每 STALL_N 步记录窗口误差；最近 3 窗口最佳 vs 前 3 窗口最佳，
        % 无实质改善则判卡住。容忍 momentum 的非单调路径（过冲/调姿回升），
        % 真卡住（窗口间持续持平）才触发
        if mod(iter, STALL_N) == 0
            [~, p_new] = planarFK_L(q, m_eff.DH, m_eff.cfg.rod_offset_arr);
            d_new = norm(X_t - p_new);
            win_errs(end+1) = d_new; %#ok<AGROW>
            nw = numel(win_errs);
            if nw >= 6
                cur = min(win_errs(nw-2:end));      % 最近 3 窗口最佳
                prv = min(win_errs(nw-5:nw-3));     % 前 3 窗口最佳
                if cur > prv - STALL_TOL
                    stalled = true;
                    break;
                end
            end
        end

        V_hist(iter) = armValue(m_eff, q);
        % 每步时间估计：步长占 dq_max 的比例（0~1 单位），供 t_seq 节拍
        t_accum = t_accum + min(1, norm(v) / m_eff.cfg.dq_max);

        % 快照 + 回调
        if mod(iter, snapshot_m) == 0
            snap(end+1, :) = q; %#ok<AGROW>
            t_seq(end+1) = t_accum; %#ok<AGROW>
            if ~isempty(onStep), onStep(q, iter, dist_end); end
        end
    end
    V_hist = V_hist(1:iter);
    % 回放一致性：无论收敛/卡住/取消/迭代耗尽，末帧必须 = q_final（当前 q）
    if size(snap,1) >= 1 && norm(snap(end,:) - q) > 1e-9
        snap(end+1, :) = q; %#ok<AGROW>
        t_seq(end+1) = t_accum; %#ok<AGROW>
    end

    info.q_snapshot = snap;
    info.t_seq = t_seq;
    info.V_hist = V_hist;
    info.q_final = q;
    info.success = converged && ~cancelled;
    info.converged = converged;
    info.cancelled = cancelled;
    info.stalled = stalled;
    info.conv_type = conv_type;   % 'hard'=固定阈值达标 | 'item'=二次精修逐项<0.001收敛
    info.iter = iter;
    info.dist_end = dist_end;
    info.err_ang = abs(e_ang);
    if cancelled
        info.error_code = 6;                       % 任务被取消/急停
    elseif converged
        info.error_code = 0;
    elseif stalled
        info.error_code = 7;                       % 卡住（无进展早停）
    else
        info.error_code = 2;                       % 未收敛（迭代耗尽）
    end
    info.stats.dt_mean = t_accum / max(iter, 1);
    if verbose
        fprintf('[momentum] iter=%d conv=%d |err|=%.4f |err_ang|=%.3f\n', ...
            iter, converged, dist_end, abs(e_ang));
    end
end
