%% ===================== PRM* 渐进最优求解器（优化版 + 空间索引 + 全路径返回） =====================
function [q_best, success, prm_cache, path_q] = runPRMStar(params, prm_cache)
    % 返回:
    %   q_best   - 最优终点关节构型 [1×N]
    %   success  - 是否找到可行路径
    %   prm_cache - 路线图缓存（可保存为.mat复用）
    %   path_q   - A* 搜索到的完整路径 [M×N]，每行一个 waypoint
    path_q = [];  % 默认空
    N = params.N;
    L = params.L_seg;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    obs = params.obs;
    rho0 = params.rho0;
    X_tgt = params.X_target;
    th_tgt = params.theta_end_target;
    rod = params.rod_offset_arr;
    DH = zeros(N,4); DH(:,3) = L;
    obs_lines_loc = {};
    if isfield(params,'obs_lines'), obs_lines_loc = params.obs_lines; end
    use_full = isfield(params,'prm_full') && params.prm_full;
    n_nodes_base = iff(use_full, 50000, 10000);
    k_nearest = min(15, n_nodes_base);
    q_init_use = params.q_init(:)';

    % ---- 线段障碍物空间索引 ----
    line_grid = struct('cells',{{}},'res',0,'min_xy',[0 0],'nx',0,'ny',0);
    if ~isempty(obs_lines_loc)
        line_grid = buildLineGrid(obs_lines_loc, rho0);
    end

    function ok = isFree(qq)
        [p_all,~] = planarFK_L(qq,DH,rod);
        [g_check,~] = obsSegGradient(qq,DH,obs,rho0,p_all,rod,{});
        if ~isempty(g_check) && min(g_check) < rho0
            ok = false; return;
        end
        if ~isempty(obs_lines_loc)
            if lineCellCheck(p_all, line_grid, rho0, obs_lines_loc)
                ok = false; return;
            end
        end
        ok = true;
    end
    function [pe,th] = fkEnd(qq)
        [~,pe] = planarFK_L(qq,DH,rod);
        th = getEndEffectorAngle_L(qq,DH,rod);
    end

    % --- Use cache if available ---
    if nargin >= 2 && ~isempty(prm_cache) && isfield(prm_cache,'nodes')
        fprintf('[PRM*] 使用缓存路线图 (%d nodes)...\n', size(prm_cache.nodes,2));
        nodes = prm_cache.nodes; adj = prm_cache.adj; costs_mtx = prm_cache.costs;
    else
        % --- Build PRM roadmap ---
        n_nodes = n_nodes_base;
        fprintf('[PRM*] Building roadmap (%d nodes, %s, 线段数=%d)...\n', n_nodes, iff(use_full,'full','lite'), length(obs_lines_loc));
        t_build = tic;
        nodes = zeros(N, n_nodes + 2);
        n_valid = 0;
        sample_batch = 1000;
        report_interval = max(1, floor(n_nodes / 10));
        while n_valid < n_nodes
            batch_qs = q_min + rand(sample_batch, N).*(q_max-q_min);
            for bi = 1:size(batch_qs,1)
                if n_valid >= n_nodes, break; end
                qs = batch_qs(bi,:);
                if isFree(qs)
                    n_valid = n_valid + 1;
                    nodes(:, n_valid) = qs';
                end
            end
            if mod(n_valid, report_interval) < sample_batch
                pct = 100 * n_valid / n_nodes;
                fprintf('[PRM*] 采样进度: %d/%d (%.1f%%) | %.1fs\n', n_valid, n_nodes, pct, toc(t_build));
            end
            if toc(t_build) > 60 && n_valid < n_nodes * 0.1
                fprintf('[PRM*] 采样困难 (自由空间占比低)，强制结束采样\n');
                break;
            end
        end
        fprintf('[PRM*] 采样完成: %d 有效节点 (%.1fs)\n', n_valid, toc(t_build));
        nodes = nodes(:, 1:n_valid);
        n_valid = size(nodes,2);
        
        fprintf('[PRM*] 边连接中 (每节点 %d 近邻)...\n', k_nearest);
        adj = cell(n_valid, 1);
        costs_mtx = cell(n_valid, 1);
        rep_edge = max(1, floor(n_valid / 10));
        t_edge = tic;
        n_edges = 0;
        for i = 1:n_valid
            dists = vecnorm(nodes - nodes(:,i), 2, 1);
            [~, idxs] = sort(dists);
            cnt = 0;
            for jj = 1:min(k_nearest+5, n_valid)
                j = idxs(jj);
                if j == i, continue; end
                if checkEdgeFreeFast(nodes(:,i)', nodes(:,j)', N, @isFree, q_min, q_max)
                    cnt = cnt + 1;
                    n_edges = n_edges + 1;
                    d = norm(nodes(:,i)-nodes(:,j));
                    adj{i}(end+1) = j;
                    costs_mtx{i}(end+1) = d;
                    adj{j}(end+1) = i;
                    costs_mtx{j}(end+1) = d;
                    if cnt >= k_nearest, break; end
                end
            end
            if mod(i, rep_edge) == 0
                fprintf('[PRM*] 边连接进度: %d/%d (%.1f%%) | %d 条边 | %.1fs\n', ...
                    i, n_valid, 100*i/n_valid, n_edges, toc(t_edge));
            end
        end
        fprintf('[PRM*] 边连接完成: %d 条边 (%.1fs)\n', n_edges, toc(t_edge));
        fprintf('[PRM*] Roadmap built total (%.1fs), nodes=%d\n', toc(t_build), n_valid);
        prm_cache = struct('nodes',nodes,'adj',{adj},'costs',{costs_mtx});
    end

    % --- Connect start + goal ---
    fprintf('[PRM*] 连接起点和终点...\n');
    nodes(:, end+1) = q_init_use';
    s_idx = size(nodes,2);
    adj{end+1} = []; costs_mtx{end+1} = [];
    q_goal = q_init_use;
    for attempt = 1:500
        q_goal = q_min + rand(1,N).*(q_max-q_min);
        [pe, th] = fkEnd(q_goal);
        if norm(pe - X_tgt) < params.rrt_goal_eps && abs(th - th_tgt) < 0.1 && isFree(q_goal)
            break;
        end
    end
    nodes(:, end+1) = q_goal';
    g_idx = s_idx + 1;
    adj{end+1} = []; costs_mtx{end+1} = [];
    for ii = [s_idx, g_idx]
        dists = vecnorm(nodes(:, 1:g_idx) - nodes(:,ii), 2, 1);
        [~, idxs] = sort(dists);
        cnt = 0;
        for jj = 1:min(k_nearest+5, g_idx)
            j = idxs(jj);
            if j == ii, continue; end
            if checkEdgeFreeFast(nodes(:,ii)', nodes(:,j)', N, @isFree, q_min, q_max)
                cnt = cnt + 1;
                d = norm(nodes(:,ii)-nodes(:,j));
                adj{ii}(end+1) = j; costs_mtx{ii}(end+1) = d;
                adj{j}(end+1) = ii; costs_mtx{j}(end+1) = d;
                if cnt >= k_nearest, break; end
            end
        end
    end
    fprintf('[PRM*] 起点目标已连接, start=%d goal=%d\n', s_idx, g_idx);

    % --- A* search ---
    fprintf('[PRM*] A* 搜索中 (图规模 %d nodes, %d edges)...\n', g_idx, n_edges);
    open = [s_idx];
    g_val = inf(1, g_idx); g_val(s_idx) = 0;
    f_val = inf(1, g_idx); f_val(s_idx) = norm(nodes(:,s_idx)-nodes(:,g_idx));
    came_from = zeros(1, g_idx);
    visited = false(1, g_idx);
    t_astar = tic;
    expanded = 0;
    report_a = max(1, floor(g_idx / 5));
    while ~isempty(open)
        [~, mi] = min(f_val(open));
        cur = open(mi);
        open(mi) = [];
        expanded = expanded + 1;
        if mod(expanded, report_a) == 0
            fprintf('[PRM*] A* 进度: %d/%d 节点已展开 | %.1fs\n', expanded, g_idx, toc(t_astar));
        end
        if cur == g_idx, break; end
        visited(cur) = true;
        for ki = 1:length(adj{cur})
            nb = adj{cur}(ki);
            if visited(nb), continue; end
            tentative = g_val(cur) + costs_mtx{cur}(ki);
            if tentative < g_val(nb)
                g_val(nb) = tentative;
                f_val(nb) = tentative + norm(nodes(:,nb)-nodes(:,g_idx));
                came_from(nb) = cur;
                if ~ismember(nb, open), open(end+1) = nb; end
            end
        end
    end
    if g_val(g_idx) == inf
        fprintf('[PRM*] A* failed (%.2fs, expanded %d nodes): no path. fallback RRT.\n', toc(t_astar), expanded);
        q_best = q_init_use; success = false; return;
    end
    % Backtrack full path
    path_q = q_goal;
    c = g_idx;
    while c ~= s_idx
        c = came_from(c);
        path_q = [nodes(:,c)'; path_q];
    end
    q_best = path_q(end, :);
    fprintf('[PRM*] Path found (%.2fs): %d waypoints | A* expanded %d nodes | total graph %d nodes\n', ...
        toc(t_astar), size(path_q,1), expanded, g_idx);
    success = true;
end

function ok = checkEdgeFreeFast(qa, qb, Nvar, isFree, qmin, qmax)
    d = norm(qb - qa);
    if d < 1e-8, ok = true; return; end
    n_seg = ceil(d / 0.6);
    for s = 1:n_seg
        tvar = s/n_seg;
        qi = (1-tvar)*qa + tvar*qb;
        qi = max(qmin, min(qmax, qi));
        if ~isFree(qi), ok = false; return; end
    end
    ok = true;
end

function v = iff(c,a,b)
    if c, v=a; else, v=b; end
end

%% ---- 线段障碍物网格索引 ----
function grid = buildLineGrid(obs_lines, rho0)
    res = 2 * rho0;
    bb_min = [inf inf]; bb_max = [-inf -inf];
    for li = 1:length(obs_lines)
        ln = obs_lines{li};
        bb_min = min(bb_min, min(ln,[],1));
        bb_max = max(bb_max, max(ln,[],1));
    end
    bb_min = bb_min - rho0;
    bb_max = bb_max + rho0;
    nx = max(1, ceil((bb_max(1)-bb_min(1))/res));
    ny = max(1, ceil((bb_max(2)-bb_min(2))/res));
    cells = cell(nx, ny);
    for li = 1:length(obs_lines)
        ln = obs_lines{li};
        xr = floor((ln(:,1) - bb_min(1))/res) + 1;
        yr = floor((ln(:,2) - bb_min(2))/res) + 1;
        for ix = min(xr):max(xr)
            for iy = min(yr):max(yr)
                if ix>=1 && ix<=nx && iy>=1 && iy<=ny
                    cells{ix,iy}(end+1) = li;
                end
            end
        end
    end
    grid.cells = cells;
    grid.res = res;
    grid.min_xy = bb_min;
    grid.nx = nx;
    grid.ny = ny;
end

function collides = lineCellCheck(p_all, grid, rho0, obs_lines)
    res = grid.res;
    x0 = grid.min_xy(1);
    y0 = grid.min_xy(2);
    nx = grid.nx;
    ny = grid.ny;
    n_pts = size(p_all, 1);
    for i = 1:n_pts-1
        p0 = p_all(i,:);
        p1 = p_all(i+1,:);
        xr = floor(([p0(1) p1(1)] - x0)/res) + 1;
        yr = floor(([p0(2) p1(2)] - y0)/res) + 1;
        if any(isnan(xr)) || any(isnan(yr)), continue; end
        xr = [min(xr) max(xr)];
        yr = [min(yr) max(yr)];
        for ix = max(1,xr(1)):min(nx,xr(2))
            for iy = max(1,yr(1)):min(ny,yr(2))
                if isempty(grid.cells{ix,iy}), continue; end
                for li = grid.cells{ix,iy}
                    ln = obs_lines{li};
                    l0 = ln(1,:); l1 = ln(2,:);
                    d = pointSegDist(p0, p1, l0, l1);
                    if d < rho0
                        collides = true; return;
                    end
                end
            end
        end
    end
    collides = false;
end

function d = pointSegDist(a0, a1, b0, b1)
    mida = (a0 + a1)/2;
    midb = (b0 + b1)/2;
    d1 = pointToSegDist2(mida, b0, b1);
    d2 = pointToSegDist2(midb, a0, a1);
    d = min(sqrt(d1), sqrt(d2));
end

function dsq = pointToSegDist2(p, seg0, seg1)
    dx = seg1(1)-seg0(1); dy = seg1(2)-seg0(2);
    len2 = dx*dx + dy*dy;
    if len2 < 1e-12
        dsq = (p(1)-seg0(1))^2 + (p(2)-seg0(2))^2;
        return;
    end
    t = ((p(1)-seg0(1))*dx + (p(2)-seg0(2))*dy) / len2;
    t = max(0, min(1, t));
    px = seg0(1) + t*dx;
    py = seg0(2) + t*dy;
    dsq = (p(1)-px)^2 + (p(2)-py)^2;
end