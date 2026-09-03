function info = method_rrt(model, q0, target, opts)
%method_rrt RRT 关节空间快速探索随机树（采样层，无梯度局部最优）
%   info = method_rrt(model, q0, target, opts)
%   q0    : 初始关节角 [1×N]
%   target: 目标位姿 [x, y, θ]
%   opts  : .snapshot_m .onStep .isCancel .max_samples .max_step .goal_eps .goal_ang
%
%   设计要点（方案 §5.3）：
%   - 障碍只是采样拒绝条件（不构造势场）——RRT 天然无局部最优
%   - 边碰撞检查沿边插值多采样（防穿段漏检，修复旧版只查端点）
%   - 目标区域判定参数化（position 容差 + 角度容差）
%   - 成功 → 树回溯路径；失败 → 返回最近可达节点（warm start，error_code=3）
%   返回 info 结构同 method_momentum（q_snapshot=路径快照 / q_final / success / error_code）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = optget(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = optget(opts, 'onStep', []);
    isCancel= optget(opts, 'isCancel', []);
    max_samples = optget(opts, 'max_samples', cfg.rrt_max_samples);
    max_step    = optget(opts, 'max_step', cfg.rrt_max_step);
    goal_eps    = optget(opts, 'goal_eps', cfg.rrt_goal_eps);
    goal_ang    = optget(opts, 'goal_ang', cfg.rrt_goal_ang);

    N = cfg.N;
    q_min = cfg.q_min(:)';  q_max = cfg.q_max(:)';
    if ~isempty(optget(opts, 'seed', [])), rng(optget(opts, 'seed', [])); end   % 可复现
    use_prescan = optget(opts, 'use_prescan', true);                       % 种子预跑开关（无障时可关省时）

    % ---- 内部工具（闭包于本函数） ----
    function ok = isFree(qq)
        g = obsDistAll(model, qq);
        ok = isempty(g) || min(g) >= cfg.rho0;
    end
    function [pe, th] = fkEnd(qq)
        [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qq, model.DH, cfg.rod_offset_arr);
    end
    function d = goalDist(qq)
        [pe, th] = fkEnd(qq);
        d = norm(pe - target(1:2)) + 0.1 * abs(wrapAngle(th - target(3)));
    end
    function ok = edgeFree(qa, qb)
        % 沿边插值多采样碰撞检查（含端点）
        dq = qb - qa;
        n_chk = max(3, ceil(norm(dq) / 0.4));
        ok = true;
        for s = 0:n_chk
            qi = qa + (s/n_chk) * dq;
            if ~isFree(qi), ok = false; return; end
        end
    end

    % ---- RRT 主循环 ----
    tree = q0';  parent = 0;
    d_best = goalDist(q0);  q_best = q0;
    success = false;
    cancelled = false;
    path = [];
    q_goal = [];

    % 任务空间引导：多起点短动量找目标种子（随机初值绕开势阱，取末端误差最小者）
    %   失败回退均匀采样；种子误差越小，后续扰动越精细
    q_seed = [];
    best_seed_err = inf;
    n_seed = 6;
    if use_prescan
        for k = 1:n_seed
            qr = q_min + rand(1, N) .* (q_max - q_min);
            ri = method_momentum(model, qr, target, ...
                struct('max_iter', 30, 'snapshot_m', inf));
            if ri.dist_end < best_seed_err
                best_seed_err = ri.dist_end;
                q_seed = ri.q_final;
            end
        end
    end
    coarse_eps = 0.4;   % 粗达阈值：末端距目标 < 此值即启动动量精修

    for iter = 1:max_samples
        if ~isempty(isCancel) && isCancel(), cancelled = true; break; end
        % 采样：30% 目标种子附近扰动（扰动幅度随种子误差自适应），否则均匀
        if ~isempty(q_seed) && rand < 0.3
            sigma_seed = 0.05 + 0.2 * (1 - min(1, best_seed_err / 1.0));
            q_rand = q_seed + sigma_seed * randn(1, N);
            q_rand = max(q_min, min(q_max, q_rand));
        else
            q_rand = q_min + rand(1, N) .* (q_max - q_min);
        end
        [~, idx] = min(vecnorm(tree - q_rand', 2, 1));
        q_near = tree(:, idx)';
        q_new = steer(q_near, q_rand, max_step, q_min, q_max);
        if ~edgeFree(q_near, q_new), continue; end
        tree(:, end+1) = q_new'; %#ok<AGROW>
        parent(end+1) = idx; %#ok<AGROW>
        d_new = goalDist(q_new);
        if d_new < d_best
            d_best = d_new;  q_best = q_new;
        end
        % 目标检查：粗达 → 无梯度随机贪心精修（绕过梯度势阱）
        [pe, th] = fkEnd(q_new);
        if norm(pe - target(1:2)) < coarse_eps
            [q_rf, p_rf, a_rf] = refineRandomGreedy(model, q_new, target, ...
                struct('layers', 4, 'steps_per_layer', 150));
            if p_rf < goal_eps && a_rf < goal_ang
                q_goal = q_rf;
                success = true;
                % 最后一段（树末 → 精修点）必须无碰撞，否则回退树节点
                if edgeFree(tree(:, size(tree,2))', q_rf)
                    path = [extractPath(tree, parent, size(tree,2)); q_rf];
                else
                    path = extractPath(tree, parent, size(tree,2));
                    q_goal = q_new;
                end
                break;
            elseif p_rf < coarse_eps
                % 精修后仍接近目标（但角度未达）：以精修结果为起点继续扩展
                q_new = q_rf;
            end
        end
        % 精确命中（不经精修的直接达标）
        if norm(pe - target(1:2)) < goal_eps && abs(wrapAngle(th - target(3))) < goal_ang
            q_goal = q_new;
            success = true;
            path = extractPath(tree, parent, size(tree,2));
            break;
        end
        % 快照 + 回调（按路径节点数）
        if mod(size(tree,2), snapshot_m) == 0 && ~isempty(onStep)
            onStep(q_new, size(tree,2), norm(pe - target(1:2)));
        end
    end

    % ---- 组装结果 ----
    if success
        q_final = path(end, :);
        snap = path;
        t_seq = (1:size(path,1));
        error_code = 0;
        [pe, th] = fkEnd(q_final);
        dist_end = norm(pe - target(1:2));
        err_ang = abs(wrapAngle(th - target(3)));
    else
        q_final = q_best;                     % 最近可达节点（warm start）
        % 失败也返回已扩展路径（供回放/诊断；可能含未达目标的中间节点）
        if size(tree,2) > 0
            try
                snap = extractPath(tree, parent, size(tree,2));
            catch
                snap = [q0(:)'; q_best];
            end
        else
            snap = [q0(:)'; q_best];
        end
        % 回放一致性：末帧必须 = q_final
        if size(snap,1) >= 1 && norm(snap(end,:) - q_final) > 1e-9
            snap(end+1, :) = q_final; %#ok<AGROW>
        end
        t_seq = (1:size(snap,1));
        error_code = 3;
        [pe, th] = fkEnd(q_final);
        dist_end = norm(pe - target(1:2));
        err_ang = abs(wrapAngle(th - target(3)));
    end
    if cancelled, error_code = 6; end   % 取消/急停

    info.q_snapshot = snap;
    info.t_seq = t_seq;
    info.V_hist = [];
    info.q_final = q_final;
    info.success = success;
    info.converged = success;
    info.cancelled = cancelled;
    info.iter = iter;
    info.dist_end = dist_end;
    info.err_ang = err_ang;
    info.error_code = error_code;
    info.stats.tree_nodes = size(tree, 2);
    info.stats.d_best = d_best;
end

%% ---------- 工具 ----------
function q_new = steer(q_from, q_to, step, q_min, q_max)
    delta = q_to - q_from;
    d = norm(delta);
    if d < 1e-12
        q_new = q_from;
    elseif d <= step
        q_new = q_to;
    else
        q_new = q_from + (step/d) * delta;
    end
    q_new = max(q_min, min(q_max, q_new));
end

function path = extractPath(tree, parent, idx)
    path = tree(:, idx)';
    c = idx;
    while c ~= 1
        c = parent(c);
        path = [tree(:, c)'; path]; %#ok<AGROW>
    end
end
