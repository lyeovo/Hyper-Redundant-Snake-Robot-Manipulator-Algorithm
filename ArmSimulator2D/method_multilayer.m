function info = method_multilayer(model, q0, target, opts)
%method_multilayer 多层运动：骨架连通图 → 候选路径 → 逐段单段求解（快速+稳健回退）
%   info = method_multilayer(model, q0, target, opts)
%   可开关：opts.use_multilayer=false → 退化为 method_auto（单段）
%
%   opts:
%     .use_multilayer  (默认 true)  开关
%     .k_paths         (默认 6)  候选路径数
%     .max_segments    (默认 9)  路径段数上限
%     .seg_method_fast (默认 'momentum') 段快速求解器
%     .seg_method_rob  (默认 'rrt')      段稳健回退求解器
%     .seg_budget      (默认 rrt_max_samples/4) 段采样预算
%     .GRID            (默认 64) 连通图分辨率
%
%   返回 info 与其它方法一致：.q_snapshot .t_seq .q_final .success
%     .error_code .method_used('multilayer') .stats(含 paths/candidates)
    if nargin < 4 || isempty(opts), opts = struct(); end
    cfg = model.cfg;
    if ~of(opts,'use_multilayer', true)
        info = fallbackSolve(model, q0, target, opts);  info.method_used = 'multilayer(off)'; return;
    end
    [~, pe0] = planarFK_L(q0, model.DH, cfg.rod_offset_arr);
    start = pe0(1:2);  goal = target(1:2);
    [graph, ginfo] = buildConnectivityGraph(cfg.obstacles, start, goal, struct('GRID',of(opts,'GRID',64),'model',model));
    if isempty(graph) || isempty(graph.edges) || ~ginfo.start_goal_reachable
        info = fallbackSolve(model, q0, target, opts);  info.method_used = 'multilayer(fallback:no-path)'; return;
    end
    K = of(opts,'k_paths',2);  maxSeg = of(opts,'max_segments',9);
    paths = graphPaths(graph, K, maxSeg);
    if isempty(paths)
        info = fallbackSolve(model, q0, target, opts);  info.method_used = 'multilayer(fallback:no-cand)'; return;
    end
    segFast = of(opts,'seg_method_fast','momentum');
    segRob  = of(opts,'seg_method_rob','rrt');
    segBud  = min(400, max(150, round(cfg.rrt_max_samples/4)));   % 受限预算防慢
    if isfield(opts,'seg_budget'), segBud = opts.seg_budget; end
    tried = 0;  lastErr = inf;  lastSnap = [];  lastQ = q0;
    for pi = 1:numel(paths)
        [snap, qf, ok] = solveSegments(model, q0, graph.nodes, paths{pi}, target, segFast, segRob, segBud);
        tried = tried + 1;
        if ~ok, continue; end
        [~, peF] = planarFK_L(qf, model.DH, cfg.rod_offset_arr);
        thF = getEndEffectorAngle_L(qf, model.DH, cfg.rod_offset_arr);
        err = norm(peF - target(1:2)) + 0.5*abs(wrapAngle(target(3) - thF));
        if err < lastErr
            lastErr = err;  lastSnap = snap;  lastQ = qf;
        end
        info = assembleInfo(model, snap, qf, target, graph, paths, tried);   % 第一条全段成功即返（提速）
        return;
    end
    if ~isempty(lastSnap)
        info = assembleInfo(model, lastSnap, lastQ, target, graph, paths, tried);
        return;
    end
    info = fallbackSolve(model, q0, target, opts);  info.method_used = 'multilayer(fallback)';
end

function [snap, qf, ok] = solveSegments(model, q0, nodes, path, target, segFast, segRob, segBud)
    cfg = model.cfg;  q = q0;  qf = q0;  snap = zeros(0, cfg.N);  ok = true;
    for i = 2:numel(path)
        wp = nodes(path(i), :);                 % [x,y]
        % 路点朝向：中间点指向前进方向；终点用目标 θ
        if i < numel(path)
            nxt = nodes(path(i+1), :);
            th = atan2(nxt(2)-wp(2), nxt(1)-wp(1));
        else
            th = target(3);
        end
        tgt = [wp(1), wp(2), th];
        m = simulateMotion(model, segFast, q, tgt, 'Snapshot', 4, 'max_iter', 150);
        if ~m.success
            m = simulateMotion(model, segRob, q, tgt, 'Snapshot', 4, 'max_samples', segBud);
        end
        if ~m.success, ok = false; return; end
        if i == 2
            snap = [snap; m.q_snapshot]; %#ok<AGROW>
        else
            snap = [snap; m.q_snapshot(2:end,:)]; %#ok<AGROW>  % 去首个重复点
        end
        q = m.q_final;
    end
    qf = q;
end

function info = assembleInfo(model, snap, qf, target, graph, paths, tried)
    cfg = model.cfg;
    [~, pe] = planarFK_L(qf, model.DH, cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(qf, model.DH, cfg.rod_offset_arr);
    dist_end = norm(pe - target(1:2));
    err_ang  = abs(wrapAngle(target(3) - th));
    success  = dist_end < 0.05 && err_ang < 0.2;   % 多层粗判据（可达 + 角度近似）
    info = struct('q_snapshot', snap, 't_seq', (1:size(snap,1)), ...
        'V_hist', [], 'q_final', qf, 'success', success, 'converged', success, ...
        'cancelled', false, 'iter', size(snap,1), ...
        'dist_end', dist_end, 'err_ang', err_ang, ...
        'error_code', ~success*2, 'method_used', 'multilayer');
    info.stats = struct('n_nodes', size(graph.nodes,1), 'n_edges', size(graph.edges,1), ...
        'n_candidates', numel(paths), 'tried', tried, 'path_len', size(snap,1));
end

function info = fallbackSolve(model, q0, target, opts)
    % 快速单段兜底（momentum→rrt 有限预算），不触发慢的 method_auto 全链
    m = simulateMotion(model, 'momentum', q0, target, 'Snapshot', 4, 'max_iter', 500);
    if ~m.success
        m = simulateMotion(model, 'rrt', q0, target, 'Snapshot', 4, 'max_samples', 600);
    end
    info = m;
end

function v = of(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
