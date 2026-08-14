function info = method_auto(model, q0, target, opts)
%method_auto 自动调度链（方案 §5.6）
%   0. 动量法（快路径：无障碍/简单场景直接收敛，精度达 tol_pos/tol_ang）
%   1. RRT*（渐近最优；预算自适应 + 失败升档一次）
%   2. 图引导 graph（走廊图 + A* + 逐段 RRT*：窄通道/多障碍逃逸）
%   3. PRM（路线图 + A* + 终点精修）
%   4. SA（最后兜底）
%   每层失败才触发下一层；全部失败返回误差最小者（success=false）
%
%   分层依据：采样层解决障碍势阱连通性，梯度层解决精度，SA 兜底漏网局部坑。
%   返回 info 结构同各方法，info.method_used 记录实际使用的方法。
    if nargin < 4 || isempty(opts), opts = struct(); end
    m = model;

    % 0. 快路径：动量法
    i_m = method_momentum(m, q0, target, opts);
    if i_m.success
        info = i_m;  info.method_used = 'momentum';
        return;
    end
    % 局部最优检测：确认需要采样层（可打印诊断）
    [is_min, gn, de] = detectLocalMin(m, i_m.q_final, target);

    % 1. RRT*（渐近最优；预算按场景复杂度自适应 + 失败升档一次）
    opts_r = opts;
    if ~isfield(opts, 'max_samples')
        opts_r.max_samples = min(adaptiveBudget(m, q0, target), 6000);
    end
    i_r = method_rrtstar(m, q0, target, opts_r);
    if i_r.success
        info = i_r;  info.method_used = 'rrtstar';
        return;
    end
    % 升档重试（×2 一次：覆盖样本不足型窄通道；受 cfg 上限约束）
    opts_r.max_samples = min(opts_r.max_samples * 2, m.cfg.rrt_max_samples);
    i_r2 = method_rrtstar(m, q0, target, opts_r);
    if i_r2.success
        info = i_r2;  info.method_used = 'rrtstar';
        return;
    end

    % 2. 图引导分层（走廊图 + A* + 逐段 RRT*：窄通道/多障碍逃逸，成功率 > RRT*）
    opts_g = opts;
    opts_g.max_samples = 500;      % auto 实时约束：段内预算小
    opts_g.total_budget = 2500;    % 图引导总预算（中等难度够用，极端场景快速失败）
    i_g = method_graph(m, q0, target, opts_g);
    if i_g.success
        info = i_g;  info.method_used = 'graph';
        return;
    end

    % 3. PRM（与 RRT* 采样互补；auto 实时约束：小图 + 小预算）
    opts_p = opts;
    opts_p.n_nodes = 100;
    opts_p.max_samples = 100;
    i_p = method_prm(m, q0, target, opts_p);
    if i_p.success
        info = i_p;  info.method_used = 'prm';
        return;
    end

    % 4. SA 兜底（auto 实时约束：降迭代上限）
    opts_s = opts;
    opts_s.max_iter = 600;
    i_s = method_sa(m, q0, target, opts_s);
    if i_s.success
        info = i_s;  info.method_used = 'sa';
        return;
    end

    % 全部失败：取末端误差最小者（RRT* 用升档结果——预算更大误差通常更小）
    if i_r2.dist_end < i_r.dist_end, i_r = i_r2; end
    cand = {i_m, i_r, i_g, i_p, i_s};
    best = cand{1};
    for k = 2:5
        if cand{k}.dist_end < best.dist_end
            best = cand{k};
        end
    end
    info = best;
    info.success = false;
    info.method_used = 'auto(全部失败)';
    info.stats.localmin = is_min;
    info.stats.grad_norm = gn;
end
