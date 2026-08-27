function [graph, info] = buildConnectivityGraph(obstacles, start, goal, opts)
%buildConnectivityGraph 从障碍几何构建末端(单点)连通图（LGM 的几何骨架）
%   [graph, info] = buildConnectivityGraph(obstacles, start, goal, opts)
%
%   obstacles : struct('circles',[n×3],'rects',[n×5])
%   start,goal: [1×2] 起/终点末端工作空间位置
%   opts      : .GRID(64) .min_clear(0.10R) .suppress_r(0.18R)
%               .conn_r(0.35R) .knn(4) .dirTest(false)
%
%   流程（无信号/图像工具箱依赖）：栅格占用 → 二遍 Chamfer 距离场 →
%   走廊中心取点(距离场局部极大+非极大抑制) → 可见性加边(直线段自由) → 图
%
%   返回 graph: .nodes[K×2] .edges[M×2](无向) .dirEdges[D×2](有向) .start_i .goal_i
%   info: .occ .dist .gx .gy .dx .n_nodes .n_edges .start_goal_reachable
    if nargin < 4 || isempty(opts), opts = struct(); end
    if isfield(opts,'seed') && ~isempty(opts.seed), rng(opts.seed); end
    if isfield(opts,'model') && ~isempty(opts.model)
        R = opts.model.cfg.N * opts.model.cfg.L_seg(1);
    else
        R = 6;
    end
    GRID = of(opts,'GRID',64);

    % ---- 1. 场景范围（覆盖起点/终点/障碍 + 边距），正方形网格 ----
    allPts = [start; goal];
    for ci = 1:size(obstacles.circles,1)
        allPts = [allPts; obstacles.circles(ci,1:2)]; %#ok<AGROW>
    end
    for ri = 1:size(obstacles.rects,1)
        allPts = [allPts; obstacles.rects(ri,1:2)]; %#ok<AGROW>
    end
    margin = 0.35*R;
    xmin = min(allPts(:,1))-margin;  xmax = max(allPts(:,1))+margin;
    ymin = min(allPts(:,2))-margin;  ymax = max(allPts(:,2))+margin;
    cx = (xmin+xmax)/2;  cy = (ymin+ymax)/2;
    span = max(xmax-xmin, ymax-ymin);
    xmin = cx-span/2; xmax = cx+span/2;
    ymin = cy-span/2; ymax = cy+span/2;
    dx = (xmax-xmin)/GRID;
    gx = xmin + (0.5:1:GRID)*dx;      % 1×GRID  x 格心（列下标）
    gy = (ymin + (0.5:1:GRID)*dx)';   % GRID×1 y 格心（行下标）
    occ = true(GRID,GRID);            % true = 自由

    % ---- 2. 画障碍（圆 / 可旋转矩形）----
    for ci = 1:size(obstacles.circles,1)
        c = obstacles.circles(ci,:);
        occ((gx - c(1)).^2 + (gy - c(2)).^2 <= c(3)^2) = false;
    end
    for ri = 1:size(obstacles.rects,1)
        r = obstacles.rects(ri,:); ct = cos(r(3)); st = sin(r(3));
        for ii = 1:GRID
            for jj = 1:GRID
                lx = (gx(jj)-r(1))*ct + (gy(ii)-r(2))*st;
                ly = -(gx(jj)-r(1))*st + (gy(ii)-r(2))*ct;
                if abs(lx) <= r(4)/2 && abs(ly) <= r(5)/2, occ(ii,jj) = false; end
            end
        end
    end
    si = xy2cell(start,xmin,ymin,dx,GRID);  gi = xy2cell(goal,xmin,ymin,dx,GRID);
    occ = relaxFree(occ,si,GRID);  occ = relaxFree(occ,gi,GRID);

    % ---- 3. 二遍 Chamfer 距离场（到最近障碍，工作空间单位）----
    dist = chamferDist(occ) * dx;

    % ---- 4. 走廊中心取点 + 非极大抑制 ----
    minClear = of(opts,'min_clear',0.10*R);
    suppR    = of(opts,'suppress_r',0.18*R);
    nodes = extractCorridorNodes(occ, dist, gx, gy, dx, minClear, suppR);

    % 起/终节点
    sxy = [gx(si(2)), gy(si(1))];  gxy = [gx(gi(2)), gy(gi(1))];
    nodes = [sxy; gxy; nodes];
    start_i = 1;  goal_i = 2;

    % ---- 5. 可见性加边（无向：直线段全程自由）----
    connR = of(opts,'conn_r',0.35*R);
    knn   = of(opts,'knn',4);
    edges = visibilityEdges(nodes, occ, gx, gy, dx, connR, knn);

    % ---- 6. 起/终连通性检查（图 BFS/传播）----
    reach = false(1,size(nodes,1));  reach(start_i) = true;  changed = true;
    while changed
        changed = false;
        for e = 1:size(edges,1)
            a = edges(e,1); b = edges(e,2);
            if reach(a) && ~reach(b), reach(b)=true; changed=true; end
            if reach(b) && ~reach(a), reach(a)=true; changed=true; end
        end
    end

    graph = struct('nodes',nodes,'edges',edges,'dirEdges',zeros(0,2), ...
        'start_i',start_i,'goal_i',goal_i);
    info = struct('occ',occ,'dist',dist,'gx',gx,'gy',gy,'dx',dx, ...
        'n_nodes',size(nodes,1),'n_edges',size(edges,1), ...
        'start_goal_reachable', reach(goal_i));
end

%% ---------- 工具 ----------
function cellidx = xy2cell(p, xmin, ymin, dx, GRID)
    cj = max(1, min(GRID, 1 + floor((p(1)-xmin)/dx)));
    ci = max(1, min(GRID, 1 + floor((p(2)-ymin)/dx)));
    cellidx = [ci, cj];
end

function occ = relaxFree(occ, cell, GRID)
    i = cell(1); j = cell(2);
    for di = -2:2, for dj = -2:2
        ii = i+di; jj = j+dj;
        if ii>=1 && ii<=GRID && jj>=1 && jj<=GRID, occ(ii,jj) = true; end
    end, end
end

function d = chamferDist(occ)
    % 二遍 Chamfer 距离变换（权重 1,√2）；返回【格数】距离（带边界保护）
    GRID = size(occ,1);  INF = 1e6;  d = INF*ones(GRID,GRID);  d(~occ) = 0;
    a = 1;  b = sqrt(2);
    for i = 2:GRID
        for j = 2:GRID
            if occ(i,j)
                m = [d(i,j), d(i-1,j)+a, d(i,j-1)+a, d(i-1,j-1)+b];
                if j < GRID, m(end+1) = d(i-1,j+1)+b; end      % (i-1,j+1) 需要 j+1<=GRID
                d(i,j) = min(m);
            end
        end
    end
    for i = GRID-1:-1:1
        for j = GRID-1:-1:1
            if occ(i,j)
                m = [d(i,j), d(i+1,j)+a, d(i,j+1)+a, d(i+1,j+1)+b];
                if j > 1, m(end+1) = d(i+1,j-1)+b; end          % (i+1,j-1) 需要 j-1>=1
                d(i,j) = min(m);
            end
        end
    end
end

function nodes = extractCorridorNodes(occ, dist, gx, gy, dx, minClear, suppR)
    GRID = size(occ,1);
    cand = zeros(0,2);  cl = [];
    for i = 1:GRID
        for j = 1:GRID
            if occ(i,j) && dist(i,j) >= minClear
                cand(end+1,:) = [gx(j), gy(i)]; %#ok<AGROW>
                cl(end+1) = dist(i,j); %#ok<AGROW>
            end
        end
    end
    [~,order] = sort(cl,'descend');
    cand = cand(order,:);
    nodes = zeros(0,2);  kept = 0;  maxNodes = 120;
    for t = 1:size(cand,1)
        if kept >= maxNodes, break; end
        keep = true;
        for k = 1:size(nodes,1)
            if norm(cand(t,:) - nodes(k,:)) < suppR, keep = false; break; end
        end
        if keep, nodes(end+1,:) = cand(t,:); kept = kept + 1; end %#ok<AGROW>
    end
end

function edges = visibilityEdges(nodes, occ, gx, gy, dx, connR, knn)
    K = size(nodes,1);  edges = zeros(0,2);
    for a = 1:K
        d = sqrt(sum((nodes - nodes(a,:)).^2, 2));  d(a) = inf;
        [~,ord] = sort(d);  cnt = 0;
        for oi = 1:numel(ord)
            b = ord(oi);
            if d(b) > connR || cnt >= knn, break; end
            if a < b && lineFree(nodes(a,:), nodes(b,:), occ, gx, gy, dx)
                edges(end+1,:) = [a,b]; %#ok<AGROW>
                cnt = cnt + 1;
            end
        end
    end
end

function ok = lineFree(p1, p2, occ, gx, gy, dx)
    ok = true;  GRID = size(occ,1);
    xmin = gx(1) - dx/2;  ymin = gy(1) - dx/2;
    n = max(2, ceil(norm(p2-p1)/(0.6*dx)));
    for t = linspace(0,1,n)
        p = p1 + t*(p2-p1);
        j = max(1, min(GRID, 1 + floor((p(1)-xmin)/dx)));
        i = max(1, min(GRID, 1 + floor((p(2)-ymin)/dx)));
        if ~occ(i,j), ok = false; return; end
    end
end

function v = of(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
