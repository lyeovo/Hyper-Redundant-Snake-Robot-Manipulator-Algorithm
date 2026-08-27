function info = method_graph(model, q0, target, opts)
%method_graph 图引导分层规划（走廊图 + A* 序列 + 逐段 RRT* + 段失败回溯）
%
%   info = method_graph(model, q0, target, opts)
%
%   思想（方案：工作空间走廊图，非 C-space 区域图）：
%   1) 图层：膨胀障碍（+rho0+margin）两两缝隙 → 节点（缝隙中点/绕行点/起终点），
%      节点间直线"管道安全"验证 → 边。走廊节点只含末端位置 (x,y)（走廊语义）。
%   2) A*：起点→终点走廊序列（排除已知坏边）。
%   3) 规划层：逐段 method_rrtstar（每段从上一段终态出发，末端到走廊节点，
%      角度保持接续；最后一段到 target 含角度，精修达标）。
%   4) 回溯：段规划失败 → 标记坏边 → 重搜换路，直至成功或图耗尽。
%
%   维度无关原则：走廊节点是"末端位置"，2D 用 (x,y)；3D 演进改 fkEnd 输出维度
%   与障碍几何（圆形→球、矩形→盒），图搜索/回溯逻辑不变。
%
%   opts: .max_samples（段内 RRT* 预算，默认 600）
%         .seg_retry（同段失败重试次数，默认 3）
%         .gap_max（缝隙宽度阈值 m，默认 1.0；超过则末端可直连无需节点）
%         .margin（膨胀额外边距 m，默认 0.15）

    if nargin < 4, opts = struct(); end
    cfg0 = model.cfg;
    max_samples = of(opts, 'max_samples', cfg0.graph_max_samples);
    seg_retry   = of(opts, 'seg_retry', cfg0.graph_seg_retry);
    gap_max     = of(opts, 'gap_max', cfg0.graph_gap_max);
    margin      = of(opts, 'margin', cfg0.graph_margin);
    max_iters   = of(opts, 'max_iters', cfg0.graph_max_iters);   % 回溯迭代上限（控制总预算）
    total_budget = of(opts, 'total_budget', cfg0.graph_total_budget);   % 全任务累计样本预算（防空转）

    N = model.cfg.N;
    q_min = model.cfg.q_min(:)';  q_max = model.cfg.q_max(:)';
    cfg = model.cfg;

    % ============ 2D 适配层（3D 演进替换） ============
    function [pe, th] = fkEnd(qq)
        [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qq, model.DH, cfg.rod_offset_arr);
    end
    % ============ 适配层结束 ============

    [p0, th0] = fkEnd(q0);

    % ---- 1. 膨胀障碍 ----
    obs = struct('circles', [], 'rects', []);
    if isfield(cfg.obstacles, 'circles')
        obs.circles = cfg.obstacles.circles;
    end
    if isfield(cfg.obstacles, 'rects')
        obs.rects = cfg.obstacles.rects;
    end
    infl = inflateObs(obs, cfg.rho0 + margin);

    % ---- 2. 节点生成（缝隙中点 + 绕行点 + 起终点） ----
    nodes = genNodes(infl, p0, target);
    if size(nodes, 1) < 2
        info = failInfo(model, q0, target, 3, '图节点不足（无可用走廊）');
        return;
    end

    % ---- 3. 边验证（管道安全） ----
    [adj, costs] = buildEdges(nodes, infl, cfg.rho0);

    % ---- 4/5. A* 序列 + 逐段规划 + 回溯 ----
    pen = ones(numel(adj), numel(adj));     % 边代价惩罚表（坏边 ×5，软降级而非删除）
    q_cur = q0;
    path = q0;
    success = false;  final_err = inf;
    n_iter = 0;
    budget_left = total_budget;
    while n_iter < max_iters && budget_left > 0
        n_iter = n_iter + 1;
        % 4. A*（坏边软惩罚：代价 ×5 降级，仍可作最后手段）
        seq = astar(nodes, adj, costs, pen);
        if isempty(seq)
            break;                              % 图耗尽：无可行走廊序列
        end
        % 5. 逐段规划
        q_run = q_cur;  path_run = path;
        seg_fail = [];
        for k = 2:numel(seq)
            p_seg = nodes(seq(k), :);
            % 最后一段 → 真实目标（含角度，精修达标）
            if k == numel(seq)
                tgt_seg = target;
            else
                % 走廊段：末端到节点，角度保持上一段终态（平滑接续）
                [~, th_now] = fkEnd(q_run);
                tgt_seg = [p_seg, th_now];
            end
    % 段内规划（同段重试；走廊段只约束位置，角度放宽；
    % 失败后预算翻倍再试一次——近处段失败多为预算不足）
            ok_seg = false;
            ms = min(max_samples, budget_left);
            for r = 1:seg_retry
                if k == numel(seq)
                    si = method_rrtstar(model, q_run, tgt_seg, ...
                        struct('max_samples', ms, 'rf_steps', 80));
                else
                    si = method_rrtstar(model, q_run, tgt_seg, ...
                        struct('max_samples', ms, 'goal_ang', 1.0, 'rf_steps', 60));
                end
                if si.success
                    ok_seg = true;
                    break;
                end
                ms = min(ms * 2, budget_left);  % 失败 → 预算翻倍（受剩余预算约束）
            end
            budget_left = budget_left - si.stats.tree_nodes;
            if ok_seg
                path_run = [path_run; si.q_snapshot(2:end, :)]; %#ok<AGROW>
                q_run = si.q_final;
            else
                seg_fail = [seq(k-1), seq(k)]; %#ok<AGROW>
                break;
            end
        end
        if isempty(seg_fail)
            path = path_run;
            q_cur = q_run;
            success = true;
            break;
        end
        % 回溯：坏边代价 ×5（软降级），重搜换路；同一段反复失败则封死
        pen(seg_fail(1), seg_fail(2)) = pen(seg_fail(1), seg_fail(2)) * 5;
        pen(seg_fail(2), seg_fail(1)) = pen(seg_fail(1), seg_fail(2));
        if pen(seg_fail(1), seg_fail(2)) > 1e4
            pen(seg_fail(1), seg_fail(2)) = inf;
            pen(seg_fail(2), seg_fail(1)) = inf;
        end
    end

    % ---- 6. 组装 ----
    [pe_f, th_f] = fkEnd(q_cur);
    info = struct('success', success, 'q_final', q_cur, ...
        'q_snapshot', path, 't_seq', (1:size(path,1))', ...
        'error_code', 0, 'dist_end', norm(pe_f - target(1:2)), ...
        'err_ang', abs(wrapAngle(th_f - target(3))), ...
        'converged', success, 'cancelled', false, 'stalled', false, ...
        'iter', n_iter, 'method_used', 'graph', ...
        'stats', struct('nodes', size(nodes,1), 'edges', sum(cellfun(@numel, adj))/2, ...
            'bad_edges', sum(pen(:) > 1)/2, 'n_iter', n_iter, 'nodes_xy', nodes, ...
            'edges_xy', adj2xy(nodes, adj), 'seq_used', seq));
    if ~success
        info.error_code = 3;
        info.stats.diagnosis = sprintf('图引导失败：%d 节点/%d 边，%d 条边段内不可行（走廊被堵死）', ...
            size(nodes,1), sum(cellfun(@numel, adj))/2, sum(pen(:) > 1)/2);
        info.stats.seq_used = [];
    end
end

%% ---------- 膨胀 ----------
function obs = inflateObs(obs, d)
    if ~isempty(obs.circles)
        obs.circles(:, 3) = obs.circles(:, 3) + d;
    end
    if ~isempty(obs.rects)
        obs.rects(:, 4) = obs.rects(:, 4) + 2*d;
        obs.rects(:, 5) = obs.rects(:, 5) + 2*d;
    end
end

%% ---------- 节点生成 ----------
function nodes = genNodes(infl, p0, target)
    nodes = [p0; target(1:2)];
    nc = size(infl.circles, 1);
    nr = size(infl.rects, 1);
    % A. 圆-圆缝隙
    for i = 1:nc
        for j = i+1:nc
            c1 = infl.circles(i,1:2); r1 = infl.circles(i,3);
            c2 = infl.circles(j,1:2); r2 = infl.circles(j,3);
            d = norm(c1-c2);
            gap = d - r1 - r2;
            if gap >= 0 && gap < 1.5
                u = (c2-c1)/max(d, 1e-9);
                nodes(end+1, :) = c1 + u*(r1 + gap/2); %#ok<AGROW>  缝隙中点
            end
        end
    end
    % B. 含矩形障碍的缝隙（采样最近点对）
    % B1. 圆-矩形
    for i = 1:nc
        c = infl.circles(i,1:2); r = infl.circles(i,3);
        for j = 1:nr
            [p_r, gap] = rectNearestPoint(infl.rects(j,:), c, r);
            if gap >= 0 && gap < 1.5
                u = (p_r - c)/max(norm(p_r-c), 1e-9);
                nodes(end+1, :) = c + u*(r + gap/2); %#ok<AGROW>
            end
        end
    end
    % B2. 矩形-矩形
    for i = 1:nr
        for j = i+1:nr
            [pa, pb, gap] = rectRectNearest(infl.rects(i,:), infl.rects(j,:));
            if gap >= 0 && gap < 1.5
                nodes(end+1, :) = (pa + pb)/2; %#ok<AGROW>
            end
        end
    end
    % C. 绕行点（每障碍 4 方向外移，须落在自由空间）
    off = 0.4;
    for i = 1:nc
        c = infl.circles(i,1:2); r = infl.circles(i,3);
        for dir = 1:4
            u = [cos(dir*pi/2), sin(dir*pi/2)];
            q = c + u*(r + off);
            if ~insideAny(q, infl), nodes(end+1, :) = q; %#ok<AGROW>
            end
        end
    end
    for i = 1:nr
        r = infl.rects(i,:);
        ct = cos(r(3)); st = sin(r(3));
        hw = r(4)/2; hh = r(5)/2;
        cand = [r(1)+ct*(hw+off), r(2)+st*(hw+off);
                r(1)-ct*(hw+off), r(2)-st*(hw+off);
                r(1)-st*(hh+off), r(2)+ct*(hh+off);
                r(1)+st*(hh+off), r(2)-ct*(hh+off)];
        for k = 1:4
            if ~insideAny(cand(k,:), infl), nodes(end+1, :) = cand(k,:); %#ok<AGROW>
            end
        end
    end
    % D. 随机自由节点（兜底连通：多障碍的复合通道/角落由采样覆盖，PRM 思路）
    if ~isempty(infl.circles) || ~isempty(infl.rects)
        lo = min([p0; target(1:2)]);  hi = max([p0; target(1:2)]);
        lo = lo - 0.8;  hi = hi + 0.8;
        for k = 1:12
            q = lo + rand(1,2) .* (hi - lo);
            if ~insideAny(q, infl), nodes(end+1, :) = q; %#ok<AGROW>
            end
        end
    end
    % 去重（间距 < 0.3 合并）
    keep = true(size(nodes,1), 1);
    for i = 1:size(nodes,1)
        if ~keep(i), continue; end
        for j = i+1:size(nodes,1)
            if keep(j) && norm(nodes(i,:)-nodes(j,:)) < 0.3
                keep(j) = false;
            end
        end
    end
    nodes = nodes(keep, :);
end

function [p_r, gap] = rectNearestPoint(rect, c, r)
    % 矩形到圆心 c 的最近边界点 p_r；gap = 最近点处 圆表面到矩形距离
    ct = cos(rect(3)); st = sin(rect(3));
    lx = (c(1)-rect(1))*ct + (c(2)-rect(2))*st;
    ly = -(c(1)-rect(1))*st + (c(2)-rect(2))*ct;
    qx = max(-rect(4)/2, min(rect(4)/2, lx));
    qy = max(-rect(5)/2, min(rect(5)/2, ly));
    p_r = rect(1:2) + [ct*qx - st*qy, st*qx + ct*qy];
    dist_rc = norm(c - p_r);
    gap = dist_rc - r;   % 圆表面到矩形（膨胀后）
end

function [pa, pb, gap] = rectRectNearest(r1, r2)
    % 两矩形最近点对（顶点+边采样近似）
    pts1 = rectCorners(r1);
    pts2 = rectCorners(r2);
    best = inf;
    for a = 1:4
        for b = 1:4
            d = norm(pts1(a,:) - pts2(b,:));
            if d < best, best = d; pa = pts1(a,:); pb = pts2(b,:); end
        end
    end
    gap = best;
end

function pts = rectCorners(r)
    ct = cos(r(3)); st = sin(r(3));
    hw = r(4)/2; hh = r(5)/2;
    pts = [r(1)-hw*ct+hh*st, r(2)-hw*st-hh*ct;
           r(1)+hw*ct+hh*st, r(2)+hw*st-hh*ct;
           r(1)+hw*ct-hh*st, r(2)+hw*st+hh*ct;
           r(1)-hw*ct-hh*st, r(2)-hw*st+hh*ct];
end

function ok = insideAny(p, infl)
    ok = false;
    for i = 1:size(infl.circles, 1)
        if norm(p - infl.circles(i,1:2)) < infl.circles(i,3), ok = true; return; end
    end
    for i = 1:size(infl.rects, 1)
        [~, g] = rectNearestPoint(infl.rects(i,:), p, 0);
        if g < 0, ok = true; return; end
    end
end

%% ---------- 边构建（管道安全验证） ----------
function [adj, costs] = buildEdges(nodes, infl, rho0)
    M = size(nodes, 1);
    adj = cell(1, M);  costs = cell(1, M);
    for i = 1:M
        for j = i+1:M
            if pipelineFree(nodes(i,:), nodes(j,:), infl, rho0)
                d = norm(nodes(i,:) - nodes(j,:));
                adj{i}(end+1) = j; costs{i}(end+1) = d; %#ok<AGROW>
                adj{j}(end+1) = i; costs{j}(end+1) = d; %#ok<AGROW>
            end
        end
    end
end

function ok = pipelineFree(p_a, p_b, infl, rho0)
    % 末端直线走廊：沿线上每点，半径 rho0 的圆不与膨胀障碍相交（管道检查）
    n = max(2, ceil(norm(p_b - p_a) / 0.2));
    ok = true;
    for s = 0:n
        p = p_a + (s/n)*(p_b - p_a);
        for i = 1:size(infl.circles, 1)
            c = infl.circles(i,1:2); R = infl.circles(i,3);
            if norm(p - c) < R + rho0, ok = false; return; end
        end
        for i = 1:size(infl.rects, 1)
            if pointRectSignedDist(p, infl.rects(i,:)) < rho0
                ok = false; return;
            end
        end
    end
end

%% ---------- A*（排除坏边） ----------
function seq = astar(nodes, adj, costs, pen)
    M = size(nodes, 1);
    g = inf(1, M);  g(1) = 0;
    f = inf(1, M);  f(1) = norm(nodes(1,:) - nodes(2,:));
    came = zeros(1, M);
    open = 1;  closed = false(1, M);
    while ~isempty(open)
        [~, mi] = min(f(open));
        cur = open(mi);  open(mi) = [];
        if cur == 2, break; end
        closed(cur) = true;
        for k = 1:numel(adj{cur})
            nb = adj{cur}(k);
            if isinf(pen(cur, nb)), continue; end
            t = g(cur) + costs{cur}(k) * pen(cur, nb);
            if t < g(nb)
                g(nb) = t;
                f(nb) = t + norm(nodes(nb,:) - nodes(2,:));
                came(nb) = cur;
                if ~ismember(nb, open), open(end+1) = nb; end %#ok<AGROW>
            end
        end
    end
    if isinf(g(2)), seq = []; return; end
    seq = 2;
    c = 2;
    while c ~= 1
        c = came(c);
        seq = [c, seq]; %#ok<AGROW>
    end
end

%% ---------- 工具 ----------
function E = adj2xy(nodes, adj)
    % 邻接表 → 边端点矩阵 [x1 y1 x2 y2; ...]（去重）
    E = [];
    M = size(nodes, 1);
    for i = 1:M
        for j = adj{i}
            if j > i
                E(end+1, :) = [nodes(i,:), nodes(j,:)]; %#ok<AGROW>
            end
        end
    end
end

function info = failInfo(model, q0, target, code, msg)
    [pe, th] = planarFK_L(q0, model.DH, model.cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(q0, model.DH, model.cfg.rod_offset_arr);
    info = struct('success', false, 'q_final', q0, 'q_snapshot', q0, ...
        't_seq', 1, 'error_code', code, ...
        'dist_end', norm(pe - target(1:2)), ...
        'err_ang', abs(wrapAngle(th - target(3))), ...
        'converged', false, 'cancelled', false, 'stalled', false, ...
        'iter', 0, 'method_used', 'graph', ...
        'stats', struct('diagnosis', msg));
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field)), v = s.(field); else, v = default; end
end
