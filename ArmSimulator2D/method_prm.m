function info = method_prm(model, q0, target, opts)
%method_prm PRM* 关节空间路线图 + A* 寻路（采样层，无梯度局部最优）
%   info = method_prm(model, q0, target, opts)
%   q0    : 初始关节角 [1×N]
%   target: 目标位姿 [x, y, θ]
%   opts  : .snapshot_m .onStep .isCancel .n_nodes .rad .goal_eps .goal_ang
%
%   流程：多起点短动量种子 → 采样无碰节点 → 半径连接（边插值检查）→
%         A*（启发 h=到种子关节距离）→ 回溯路径 → 终点无梯度随机贪心精修
%   失败（A* 无路径）：返回最近可达节点（warm start，error_code=3）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = of(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = of(opts, 'onStep', []);
    isCancel= of(opts, 'isCancel', []);
    n_nodes = of(opts, 'n_nodes', cfg.prm_n_nodes);
    rad     = of(opts, 'rad', cfg.prm_rad);
    goal_eps = of(opts, 'goal_eps', cfg.rrt_goal_eps);
    goal_ang = of(opts, 'goal_ang', cfg.rrt_goal_ang);
    N = cfg.N;
    q_min = cfg.q_min(:)';  q_max = cfg.q_max(:)';
    cancelled = false;
    if ~isempty(of(opts, 'seed', [])), rng(of(opts, 'seed', [])); end   % 可复现
    use_prescan = of(opts, 'use_prescan', true);                       % 种子预跑开关（无障时可关省时）

    function ok = isFree(qq)
        g = obsDistAll(model, qq);
        ok = isempty(g) || min(g) >= cfg.rho0;
    end
    function ok = edgeFree(qa, qb)
        dq = qb - qa;
        n_chk = max(3, ceil(norm(dq) / 0.4));
        ok = true;
        for s = 0:n_chk
            if ~isFree(qa + (s/n_chk)*dq), ok = false; return; end
        end
    end
    function [pe, th] = fkEnd(qq)
        [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qq, model.DH, cfg.rod_offset_arr);
    end

    % ---- 1. 目标种子（多起点短动量；可关闭） ----
    best = inf;  q_seed = [];
    if use_prescan
        for k = 1:6
            qr = q_min + rand(1,N) .* (q_max - q_min);
            ri = method_momentum(model, qr, target, struct('max_iter', 30, 'snapshot_m', inf));
            if ri.dist_end < best, best = ri.dist_end; q_seed = ri.q_final; end
        end
    end
    % 种子已达标 → 直接成功
    [p_seed, a_seed] = fkEnd(q_seed);
    if norm(p_seed - target(1:2)) < goal_eps && abs(wrapAngle(a_seed - target(3))) < goal_ang ...
            && edgeFree(q0, q_seed)
        path = [q0; q_seed];
        q_goal = q_seed;  success = true;  err_code_final = 0;
        % ---- 组装 ----
        info = struct('success', true, 'q_final', q_goal, 'q_snapshot', path, ...
            't_seq', (1:size(path,1)), 'error_code', 0, ...
            'dist_end', norm(p_seed - target(1:2)), ...
            'err_ang', abs(wrapAngle(a_seed - target(3))), ...
            'stats', struct('nodes', 2), 'method_used', 'prm');
        return;
    else
        % ---- 2. 采样节点：起点 + 种子 + 无碰随机点（30% 种子引导，保证目标区域稠密） ----
        nodes = [q0(:), q_seed(:)];
        cnt = 0;  guard = 0;
        while cnt < n_nodes && guard < 12*n_nodes
            guard = guard + 1;
            if ~isempty(isCancel) && isCancel(), cancelled = true; break; end
            if rand < 0.3
                qr = q_seed + 0.3 * randn(1, N);
                qr = max(q_min, min(q_max, qr));
            else
                qr = q_min + rand(1, N) .* (q_max - q_min);
            end
            if isFree(qr)
                nodes(:, end+1) = qr(:); %#ok<AGROW>
                cnt = cnt + 1;
            end
        end
        M = size(nodes, 2);

        % ---- 3. 半径连接（无向，边插值碰撞检查） ----
        adj = cell(1, M);  costs = cell(1, M);
        for i = 1:M
            d = vecnorm(nodes - nodes(:,i), 2, 1);
            nb = find(d <= rad & d > 0);
            for jj = 1:length(nb)
                j = nb(jj);
                if j < i, continue; end
                if edgeFree(nodes(:,i)', nodes(:,j)')
                    adj{i}(end+1) = j;  costs{i}(end+1) = d(j); %#ok<AGROW>
                    adj{j}(end+1) = i;  costs{j}(end+1) = d(j); %#ok<AGROW>
                end
            end
            if ~isempty(isCancel) && isCancel(), cancelled = true; break; end
        end

        % ---- 4. A*：起点=1，种子=2 ----
        g_val = inf(1, M);  g_val(1) = 0;
        f_val = inf(1, M);  f_val(1) = norm(nodes(:,1) - nodes(:,2));
        came = zeros(1, M);
        open = 1;  visited = false(1, M);
        while ~isempty(open)
            [~, mi] = min(f_val(open));
            cur = open(mi);  open(mi) = [];
            if cur == 2, break; end
            visited(cur) = true;
            for k = 1:length(adj{cur})
                nb = adj{cur}(k);
                if visited(nb), continue; end
                t = g_val(cur) + costs{cur}(k);
                if t < g_val(nb)
                    g_val(nb) = t;
                    f_val(nb) = t + norm(nodes(:,nb) - nodes(:,2));
                    came(nb) = cur;
                    if ~ismember(nb, open), open(end+1) = nb; end %#ok<AGROW>
                end
            end
        end

        if isinf(g_val(2))
            % ---- 5a. A* 失败：返回种子（momentum 解，末端误差最小者）作为 warm start ----
            success = false;
            q_goal = q_seed;
            path = [q0; q_seed];
            err_code_final = 3;
        else
            % ---- 5b. 回溯路径 + 终点精修 ----
            path = nodes(:, 2)';
            c = 2;
            while c ~= 1
                c = came(c);
                path = [nodes(:, c)'; path]; %#ok<AGROW>
            end
            [q_rf, p_rf, a_rf] = refineRandomGreedy(model, path(end,:), target, ...
                struct('layers', 4, 'steps_per_layer', 200));
            if p_rf < goal_eps && a_rf < goal_ang
                q_goal = q_rf;
                path = [path; q_rf];
                success = true;
                err_code_final = 0;
            else
                q_goal = path(end, :);     % 路径终点（未达精确，作 warm start）
                success = false;
                err_code_final = 2;
            end
        end
    end
    if cancelled, success = false; err_code_final = 6; end   % 取消/急停

    % ---- 组装 ----
    [pe, th] = fkEnd(q_goal);
    dist_end = norm(pe - target(1:2));
    err_ang = abs(wrapAngle(th - target(3)));
    info.q_snapshot = path;
    info.t_seq = (1:size(path,1));
    info.V_hist = [];
    info.q_final = q_goal;
    info.success = success;
    info.converged = success;
    info.cancelled = cancelled;
    info.iter = M;
    info.dist_end = dist_end;
    info.err_ang = err_ang;
    info.error_code = err_code_final;
    info.stats.nodes = M;
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
