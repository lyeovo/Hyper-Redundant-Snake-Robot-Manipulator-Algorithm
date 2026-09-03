function info = method_rrtstar(model, q0, target, opts)
%method_rrtstar RRT* 渐近最优快速探索随机树（采样层，无梯度局部最优）
%   info = method_rrtstar(model, q0, target, opts)
%   q0    : 初始关节角 [1×N]
%   target: 目标位姿 [x, y, θ]
%   opts  : .snapshot_m .onStep .isCancel .max_samples .max_step .goal_eps .goal_ang
%           .rrt_gamma   rewiring 半径常数（默认 4.0）
%           .cost_weights  C-space 代价权重 [1×N]（默认全 1，可加关节偏好）
%
%   RRT* 相对 RRT 的核心（维度无关，见方案 §5.7.2）：
%   - 代价 = 到根路径的加权 C-space 长度（渐近最优的对象）
%   - choose parent：新节点在半径 r 内选"根代价最小"的父（非最近邻）
%   - rewire：半径内邻居若"经新节点更短"则重连——路径代价单调不增
%   - 半径 r = gamma·(log n / n)^(1/N)，随节点数收缩（Karaman & Frazzoli 2011）
%
%   2D 适配层（可替换为 3D 几何而不动核心）：isFree / fkEnd / goalDist / edgeFree
%   （3D 演进：换碰撞函数与任务校验函数即可）
%
%   返回 info 结构同 method_rrt（q_snapshot=路径快照 / q_final / success / error_code，
%   额外 stats.cost_path = 最优路径代价）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    snapshot_m = optget(opts, 'snapshot_m', cfg.snapshot_m);
    onStep  = optget(opts, 'onStep', []);
    isCancel= optget(opts, 'isCancel', []);
    max_samples = optget(opts, 'max_samples', cfg.rrt_max_samples);
    max_step    = optget(opts, 'max_step', cfg.rrt_max_step);
    goal_eps    = optget(opts, 'goal_eps', cfg.rrt_goal_eps);
    goal_ang    = optget(opts, 'goal_ang', cfg.rrt_goal_ang);
    rf_steps    = optget(opts, 'rf_steps', 150);   % 终点精修步数（走廊段可调小）
    gamma_r     = optget(opts, 'rrt_gamma', 2.5);
    cw          = optget(opts, 'cost_weights', ones(1, cfg.N));   % C-space 代价权重

    N = cfg.N;
    q_min = cfg.q_min(:)';  q_max = cfg.q_max(:)';
    cw = cw(:)';
    if ~isempty(optget(opts, 'seed', [])), rng(optget(opts, 'seed', [])); end   % 可复现
    use_prescan = optget(opts, 'use_prescan', true);                       % 种子预跑开关（无障时可关省时）

    % ============ 2D 适配层（3D 演进时替换此处四个函数） ============
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
        dq = qb - qa;
        n_chk = max(4, ceil(norm(dq) / 0.25));   % 6 维下 0.25rad 步长防漏检
        ok = true;
        for s = 0:n_chk
            if ~isFree(qa + (s/n_chk) * dq), ok = false; return; end
        end
    end
    % ============ 2D 适配层结束 ============

    % ---- C-space 加权距离（维度无关代价度量） ----
    function d = cdist(qa, qb)
        d = norm(cw .* (qb - qa));
    end

    % ---- RRT* 主循环 ----
    tree = q0';  parent = 0;  cost = 0;          % cost(k): 根到节点 k 的路径代价
    d_best = goalDist(q0);  q_best = q0;
    success = false;  cancelled = false;  path = [];  q_goal = [];
    best_cost = inf;

    % q0 邻域自由种子：贴障碍起点的可扩展方向锥极窄，先铺一片安全邻域
    % （保持单根=q0，小扰动避开障碍，使树能向任意方向生长）
    for k = 1:64
        qs = q0 + 0.05 * randn(1, N);
        qs = max(q_min, min(q_max, qs));
        if isFree(qs) && edgeFree(q0, qs)
            tree(:, end+1) = qs'; parent(end+1) = 1;
            cost(end+1) = cdist(q0, qs);
        end
    end

    % 多起点短动量种子（2D 引导；3D 可换成 IK 采样引导；可关闭）
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
    coarse_eps = 0.4;
    r = max_step * 3;   % rewiring 半径默认值（循环未赋值时兜底）
    % 粗达候选池（预算内最优：保留树代价最小的前 CAND_MAX 个，循环后逐个精修）
    CAND_MAX = 8;
    coarse_cands = struct('cost', {}, 'q_goal', {}, 'idx', {});

    for iter = 1:max_samples
        if ~isempty(isCancel) && isCancel(), cancelled = true; break; end
        % 采样：轻微目标引导（8% 种子扰动；高比例偏置会破坏 RRT* 渐近最优收敛）
        if ~isempty(q_seed) && rand < 0.08
            sigma_seed = 0.1 + 0.3 * (1 - min(1, best_seed_err / 1.0));
            q_rand = q_seed + sigma_seed * randn(1, N);
            q_rand = max(q_min, min(q_max, q_rand));
        else
            q_rand = q_min + rand(1, N) .* (q_max - q_min);
        end
        m = size(tree, 2);
        % ---- 最近邻扩展（与 RRT 相同） ----
        dq_all = tree - q_rand';
        d_all = sqrt(sum((cw' .* dq_all).^2, 1));           % [1×m]
        [~, idx_near] = min(d_all);
        q_near = tree(:, idx_near)';
        q_new = steer(q_near, q_rand, max_step, q_min, q_max);
        if ~edgeFree(q_near, q_new), continue; end
        % ---- choose parent（RRT* 核心 1）：对 q_new 在半径 r 内选根代价最小父 ----
        r = min(gamma_r * (log(m) / m)^(1/N), max_step * 3); % rewiring 半径
        near_idx = find(d_all <= r);
        [idx_best, c_new] = pickParent(tree, q_new, near_idx, cost, @cdist, @edgeFree, idx_near);
        % ---- 加入树 ----
        tree(:, end+1) = q_new'; %#ok<AGROW>
        parent(end+1) = idx_best; %#ok<AGROW>
        cost(end+1) = c_new; %#ok<AGROW>
        new_idx = m + 1;
        % ---- rewire（RRT* 核心 2）：半径内邻居经新节点更短则重连 ----
        for j = near_idx
            if j == new_idx, continue; end
            d_j = cdist(q_new, tree(:, j)');
            if cost(new_idx) + d_j < cost(j) && edgeFree(q_new, tree(:, j)')
                parent(j) = new_idx;
                cost(j) = cost(new_idx) + d_j;
            end
        end
        % 目标检查：粗达 → 记录最优粗达解（预算内持续优化，不再早停）
        [pe, th] = fkEnd(q_new);
        d_new = norm(pe - target(1:2));
        if d_new < d_best
            d_best = d_new;  q_best = q_new;
        end
        if norm(pe - target(1:2)) < coarse_eps
            % 粗达解入候选池（只存索引；路径在循环结束后按 rewire 后的 parent 链重取）
            coarse_cands = insertCand(coarse_cands, cost(new_idx), q_new, new_idx, CAND_MAX);
        end
        % 精确命中（树节点直接达标，代价已最优附近，提前返回）
        if norm(pe - target(1:2)) < goal_eps && abs(wrapAngle(th - target(3))) < goal_ang
            q_goal = q_new;
            success = true;
            path = extractPath(tree, parent, new_idx);
            best_cost = cost(new_idx);
            break;
        end
        if mod(m, snapshot_m) == 0 && ~isempty(onStep)
            onStep(q_new, m, norm(pe - target(1:2)));
        end
    end

    % ---- 预算内最优粗达解：循环结束后精修全部候选，取总代价最小达标者 ----
    % 路径统一按 rewire 收敛后的 parent 链重新提取（修复候选路径与实时树代价脱节）
    if ~success && ~cancelled && ~isempty(coarse_cands)
        best_cand = [];
        for ci = 1:numel(coarse_cands)
            [q_rf, p_rf, a_rf] = refineRandomGreedy(model, coarse_cands(ci).q_goal, target, ...
                struct('layers', 4, 'steps_per_layer', rf_steps));
            % 候选必须同时：达标（精修后误差 < 阈值）且 树末节点→精修点 无碰撞——
            % 穿障则淘汰（修复：旧代码回退到未精修树节点仍置 success，误差可达 0.3m+）
            if p_rf < goal_eps && a_rf < goal_ang && edgeFree(coarse_cands(ci).q_goal, q_rf)
                path_cur = extractPath(tree, parent, coarse_cands(ci).idx);
                % 沿真实 parent 链重算树路径代价（rewire 后 cost 数组可能过期）
                c_tree = 0;
                for pt = 1:(size(path_cur,1)-1)
                    c_tree = c_tree + cdist(path_cur(pt,:), path_cur(pt+1,:));
                end
                ctot = c_tree + cdist(coarse_cands(ci).q_goal, q_rf);
                if isempty(best_cand) || ctot < best_cand.ctot
                    best_cand = struct('ctot', ctot, 'q_goal', q_rf, ...
                        'path', path_cur, 'q_tree', coarse_cands(ci).q_goal);
                end
            end
        end
        if ~isempty(best_cand)
            q_goal = best_cand.q_goal;
            success = true;
            path = [best_cand.path; q_goal];
            best_cost = best_cand.ctot;
        end
    end

    if getenv('RRTSTAR_DIAG')
        fprintf('[diag] samples=%d nodes=%d d_best=%.3f coarse_cands=%d\n', ...
            max_samples, size(tree,2), d_best, numel(coarse_cands));
    end
    % ---- 组装结果（同 method_rrt） ----
    if success
        q_final = path(end, :);
        snap = path;
        t_seq = (1:size(path,1));
        error_code = 0;
        [pe, th] = fkEnd(q_final);
        dist_end = norm(pe - target(1:2));
        err_ang = abs(wrapAngle(th - target(3)));
    else
        q_final = q_best;
        if size(tree,2) > 0
            try
                snap = extractPath(tree, parent, size(tree,2));
            catch
                snap = [q0(:)'; q_best];
            end
        else
            snap = [q0(:)'; q_best];
        end
        % 回放一致性：末帧必须 = q_final（q_best 可能不是树末节点）
        if size(snap,1) >= 1 && norm(snap(end,:) - q_final) > 1e-9
            snap(end+1, :) = q_final; %#ok<AGROW>
        end
        t_seq = (1:size(snap,1));
        error_code = 3;
        [pe, th] = fkEnd(q_final);
        dist_end = norm(pe - target(1:2));
        err_ang = abs(wrapAngle(th - target(3)));
        % 失败也给出已探索的最短代价（诊断用）
        try
            bp = extractPath(tree, parent, size(tree,2));
            best_cost = cost(end);
        catch
            best_cost = inf;
        end
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
    info.stats.cost_path = best_cost;   % 最优路径代价（渐近最优对象）
    info.stats.rewire_radius = r;
end

%% ---------- 工具 ----------
function [idx, c] = pickParent(tree, q_new, near_idx, cost, cdist, edgeFree, fallback)
    % choose parent：半径内选 根代价+边代价 最小且边无碰撞；无候选回退最近邻
    best = inf;  idx = [];
    for j = near_idx
        cj = cost(j) + cdist(tree(:, j)', q_new);
        if cj < best && edgeFree(tree(:, j)', q_new)
            best = cj;  idx = j;
        end
    end
    if isempty(idx)
        idx = fallback;
        c = cost(idx) + cdist(tree(:, idx)', q_new);
    else
        c = best;
    end
end

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

function cands = insertCand(cands, cq, q_goal, idx, cap)
% 按树代价升序插入粗达候选；超容量丢弃代价最大者
    n = numel(cands);
    if n > 0 && cq >= cands(end).cost && n >= cap
        return;                    % 比池中最差还差且已满
    end
    cands(end+1) = struct('cost', cq, 'q_goal', q_goal, 'idx', idx);
    [~, order] = sort([cands.cost]);
    cands = cands(order);
    if numel(cands) > cap
        cands = cands(1:cap);
    end
end
