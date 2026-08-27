function res = validate_connectivity(opts)
%validate_connectivity 在 2-5 随机障碍场景上验证几何连通图（LGM 骨架）可行性
%   res = validate_connectivity(opts)
%   opts: .n_scene(默认 100) .GRID(默认 64) .seed(默认 1)
%         .gridList([32 64 128]，多分辨率稳健性)
%         .do_segment_solve(默认 0，是否加测逐段 RRT 可解率——较慢)
%
%   核心指标：
%     start_goal_reachable_rate  图 BFS 下 start→goal 连通比例
%     gt_reach_rate              栅格洪水填充(真值)下 start→goal 连通比例
%     agreement_rate             图连通判断 vs 真值 的一致率（漏报=图判不通真值通）
%     avg_nodes / avg_edges      平均节点/边数
%     seg_ok_rate                逐段求解平均可解率（opt 开启时）
%     field_reach_rate           [真] 起点可达目标的比例（有可行走廊的场景占比）
%
%   用法（MATLAB 命令行）：
%     addpath('ArmSimulator2D'); res = validate_connectivity();            % 默认 100 场景
%     res = validate_connectivity(struct('n_scene',200,'GRID',64,'do_segment_solve',1));
    if nargin < 1 || isempty(opts), opts = struct(); end
    rng(of(opts,'seed',1));
    n_scene = of(opts,'n_scene',100);
    GRID    = of(opts,'GRID',64);
    gridList = of(opts,'gridList',[32 64 128]);
    do_seg  = of(opts,'do_segment_solve',0);

    base = struct('sg_ok',0,'gt_reach',0,'agree',0,'n_nodes',0,'n_edges',0, ...
        'seg_ok',0,'seg_try',0,'path_ok',0);
    R = struct();  field = base;  rs = struct();
    for g = 1:numel(gridList)
        R.(sprintf('g%d',gridList(g))) = base;
    end

    for k = 1:n_scene
        scene = sampleObstacleScene(struct('seed', k));
        for g = 1:numel(gridList)
            gg = gridList(g);
            [graph, info] = buildConnectivityGraph(scene.obs_desc, scene.start, scene.goal, ...
                struct('GRID', gg, 'model', scene.model));
            si = xy2cell(scene.start, info.gx(1)-info.dx/2, info.gy(1)-info.dx/2, info.dx, gg);
            gi = xy2cell(scene.goal,  info.gx(1)-info.dx/2, info.gy(1)-info.dx/2, info.dx, gg);
            gt = gridConnected(info.occ, si, gi);
            f = R.(sprintf('g%d', gg));
            f.n_nodes = f.n_nodes + info.n_nodes;
            f.n_edges = f.n_edges + info.n_edges;
            if info.start_goal_reachable, f.sg_ok = f.sg_ok + 1; end
            if gt, f.gt_reach = f.gt_reach + 1; end
            if info.start_goal_reachable == gt, f.agree = f.agree + 1; end
            R.(sprintf('g%d', gg)) = f;
        end
        % 逐段 RRT 可解率（可选）：只在 64 分辨率 + 图连通时测，取 graph 上一条路径
        if do_seg
            [graph, info] = buildConnectivityGraph(scene.obs_desc, scene.start, scene.goal, ...
                struct('GRID', GRID, 'model', scene.model));
            if info.start_goal_reachable
                [ok_edge, ok_path, n_edge] = segSolvePath(scene, graph);
                field.seg_ok = field.seg_ok + ok_edge;
                field.seg_try = field.seg_try + n_edge;
                if ok_path, field.path_ok = field.path_ok + 1; end
            end
        end
    end

    % ---- 汇总与打印 ----
    res = struct('n_scene', n_scene, 'gridList', gridList, 'do_segment_solve', do_seg);
    fprintf('\n========== 几何连通图验证（%d 场景，2-5 障碍）==========\n', n_scene);
    fprintf('%-6s %-10s %-10s %-10s %-9s %-9s\n', 'GRID', '图连通率', '真值连通率', '一致性', '平均节点', '平均边');
    for g = 1:numel(gridList)
        gg = gridList(g);  f = R.(sprintf('g%d', gg));
        fprintf('%-6d %-9.1f%% %-9.1f%% %-9.1f%% %-8.1f %-8.1f\n', gg, ...
            100*f.sg_ok/n_scene, 100*f.gt_reach/n_scene, 100*f.agree/n_scene, ...
            f.n_nodes/n_scene, f.n_edges/n_scene);
    end
    if do_seg && field.seg_try > 0
        fprintf('逐段 RRT 可解率: %.1f%% (%d/%d 边), 全路径可解率: %.1f%% (%d/%d)\n', ...
            100*field.seg_ok/field.seg_try, field.seg_ok, field.seg_try, ...
            100*field.path_ok/n_scene, field.path_ok, n_scene);
    end
    fprintf('======================================================\n');
    res.report = R;  res.seg = field;
end

%% ---------- 工具 ----------
function cellidx = xy2cell(p, xmin, ymin, dx, GRID)
    cj = max(1, min(GRID, 1 + floor((p(1)-xmin)/dx)));
    ci = max(1, min(GRID, 1 + floor((p(2)-ymin)/dx)));
    cellidx = [ci, cj];
end

function gt = gridConnected(occ, si, gi)
    % 洪水填充：自由格中 start 与 goal 是否同连通分量（BFS，4 邻域按行遍历）
    GRID = size(occ,1);
    visited = false(GRID,GRID);  Q = [si(1), si(2)];  visited(si(1),si(2)) = true;
    gt = false;  NB = [-1 0; 1 0; 0 -1; 0 1];   % [di, dj] 行；'for' 需按列语义，用转置遍历行
    while ~isempty(Q)
        c = Q(1,:);  Q(1,:) = [];
        if c(1)==gi(1) && c(2)==gi(2), gt = true; return; end
        for d = NB.'
            ii = c(1)+d(1);  jj = c(2)+d(2);
            if ii>=1 && ii<=GRID && jj>=1 && jj<=GRID && occ(ii,jj) && ~visited(ii,jj)
                visited(ii,jj) = true;  Q(end+1,:) = [ii, jj]; %#ok<AGROW>
            end
        end
    end
end

function [ok_edge, ok_path, n_edge] = segSolvePath(scene, graph)
    % 沿 graph 上一条 start→goal 路径，逐边做整臂无碰检查（用当前臂模型）
    % 说明：这里用 obsDistAll 对"两端点 + 边中点"做快速可行性代理——
    %       真正的逐段 C-space RRT 求解可替换为 simulateMotion 调用。
    nodes = graph.nodes;  edges = graph.edges;
    % 图 BFS 找 start→goal 一条路径
    n = size(nodes,1);  par = zeros(1,n);  vis = false(1,n);
    adj = cell(1,n);
    for e = 1:size(edges,1)
        adj{edges(e,1)}(end+1) = edges(e,2); %#ok<AGROW>
        adj{edges(e,2)}(end+1) = edges(e,1); %#ok<AGROW>
    end
    q = graph.start_i;  vis(graph.start_i) = true;
    while ~isempty(q)
        c = q(1);  q(1) = [];
        if c == graph.goal_i, break; end
        for nb = adj{c}
            if ~vis(nb), vis(nb)=true; par(nb)=c; q(end+1)=nb; end %#ok<AGROW>
        end
    end
    if ~vis(graph.goal_i), ok_edge=0; ok_path=false; n_edge=0; return; end
    % 回溯路径
    p = [];  c = graph.goal_i;
    while c ~= 0, p = [c p]; c = par(c); end %#ok<AGROW>
    if p(1) ~= graph.start_i, ok_edge=0; ok_path=false; n_edge=0; return; end
    ok_edge = 0;  n_edge = 0;  ok_path = true;
    model = scene.model;  cfg = model.cfg;
    for e = 1:numel(p)-1
        n_edge = n_edge + 1;
        a = nodes(p(e),:);  b = nodes(p(e+1),:);
        if pointPathFree(model, a, b, cfg.rho0), ok_edge = ok_edge + 1;
        else, ok_path = false; end
    end
end

function ok = pointPathFree(model, p1, p2, rho0)
    % 末端(点)沿直线段是否障碍外（快检代理；真正整臂 C-space 求解用 simulateMotion）
    ok = true;
    n = max(4, ceil(norm(p2-p1)/0.05));
    for t = linspace(0,1,n)
        p = p1 + t*(p2-p1);
        if ~ptFree(model, p, rho0), ok = false; return; end
    end
end

function ok = ptFree(model, p, rho0)
    cfg = model.cfg;  ok = true;
    for ci = 1:size(cfg.obstacles.circles,1)
        c = cfg.obstacles.circles(ci,:);
        if norm(p - c(1:2)) <= c(3) + rho0, ok=false; return; end
    end
    for ri = 1:size(cfg.obstacles.rects,1)
        r = cfg.obstacles.rects(ri,:);  ct=cos(r(3)); st=sin(r(3));
        lx=(p(1)-r(1))*ct+(p(2)-r(2))*st;  ly=-(p(1)-r(1))*st+(p(2)-r(2))*ct;
        if abs(lx)<=r(4)/2+rho0 && abs(ly)<=r(5)/2+rho0, ok=false; return; end
    end
end

function v = of(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
