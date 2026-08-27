function info = method_multilayer(model, q0, target, opts)
%method_multilayer 多层运动：骨架连通图 → 候选路径 → 逐段单段求解（快速+稳健回退）
%   info = method_multilayer(model, q0, target, opts)
%   可开关：opts.use_multilayer=false → 退化为快速单段兜底 fallbackSolve（不触发多层）
%
%   opts:
%     .use_multilayer  (默认 true)  开关
%     .k_paths         (默认 2)  候选路径数
%     .max_segments    (默认 9)  路径段数上限
%     .seg_method_fast (默认 'momentum') 段快速求解器
%     .seg_method_rob  (默认 'rrt')      段稳健回退求解器
%     .seg_budget      (默认 rrt_max_samples/4) 段采样预算
%     .GRID            (默认 64) 连通图分辨率
%     .candidate_idx   (默认 0)  >0 只求解候选集中的第 idx 条（供“候选集选择”）
%
%   返回 info 与其它方法一致：.q_snapshot .t_seq .q_final .success
%     .error_code .method_used('multilayer')
%     .stats(含 nodes_xy/edges_xy/seq_used 连通图叠加 + path_len)
%     .candidates（候选集：图搜索输出的每条可达路径 {nodes, wp}，供复杂运动情形选择）
%   段判据已放宽：中间段只看“不碰障 + 在空间内 + 位置宽松(<waypoint_eps)”，
%     末端角度完全放开（由段内梯度自由决定），不再强求严格贴住路点。
%   末尾用 refineRandomGreedy 做一次精确微调（pos<0.03m 判成功）。
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
    sel = of(opts,'candidate_idx', 0);   % >0：只求解指定候选（复用“候选集→选择”→逐条求解）
    if sel > 0 && sel <= numel(paths), paths = paths(sel); end
    segFast = of(opts,'seg_method_fast','momentum');
    segRob  = of(opts,'seg_method_rob','rrt');
    segBud  = min(400, max(150, round(cfg.rrt_max_samples/4)));   % 受限预算防慢
    if isfield(opts,'seg_budget'), segBud = opts.seg_budget; end
    tried = 0;
    wpEps = of(opts,'waypoint_eps', 0.4);   % 中间路点位置宽松容差（m）：不碰障+在空间内+可轻松到达
    for pi = 1:numel(paths)
        [snap, qf, ok] = solveSegments(model, q0, graph.nodes, paths{pi}, target, segFast, segRob, segBud, wpEps);
        tried = tried + 1;
        if ~ok, continue; end
        % 末尾精确微调：从 qf 朝精确目标 [x,y,θ] 做严格收敛（位置+角度收紧）
        [snap, qf] = refineFinal(model, snap, qf, target);
        info = assembleInfo(model, snap, qf, target, graph, paths, tried, paths{pi});   % 第一条全段成功即返（提速）
        return;
    end
    info = fallbackSolve(model, q0, target, opts);  info.method_used = 'multilayer(fallback)';
end

function [snap, qf, ok] = solveSegments(model, q0, nodes, path, ~, segFast, segRob, segBud, wpEps)
    cfg = model.cfg;  q = q0;  qf = q0;  snap = zeros(0, cfg.N);  ok = true;
    for i = 2:numel(path)
        wp = nodes(path(i), :);                 % [x,y]
        % 角度完全放开：目标 θ = 当前末端角度（梯度只推位置，不强制转向）
        th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
        tgt = [wp(1), wp(2), th];
        m = simulateMotion(model, segFast, q, tgt, 'Snapshot', 4, 'max_iter', 150);
        if ~nearWP(m, wp, wpEps)
            m = simulateMotion(model, segRob, q, tgt, 'Snapshot', 4, 'max_samples', segBud);
        end
        if ~nearWP(m, wp, wpEps), ok = false; return; end   % 软位置判据：只看末端到路点距离
        if i == 2
            snap = [snap; m.q_snapshot]; %#ok<AGROW>
        else
            snap = [snap; m.q_snapshot(2:end,:)]; %#ok<AGROW>  % 去首个重复点
        end
        q = m.q_final;
    end
    qf = q;
end

function isNear = nearWP(m, ~, eps)
    % 宽松段判据：不碰障 + 在空间内即可，位置只需足够宽松（< eps）
    %   ① 结果有效：求解器返回了末端构型（求解器已保证关节极限/工作空间内）
    %   ② 不碰障：段轨迹全程无碰撞（safety_ok）
    %   ③ 位置宽松：末端到路点距离 < eps（默认 0.4m，对应 ~1.2m 路点间距）
    %   —— 不再强求严格贴住路点、完全放开末端角度（θ 由段内自由决定）
    okQ  = isstruct(m) && isfield(m,'q_final') && ~isempty(m.q_final) && isfield(m,'safety_ok') && m.safety_ok;
    okD  = isstruct(m) && isfield(m,'dist_end') && ~isempty(m.dist_end) && m.dist_end < eps;
    isNear = okQ && okD;
end

function [snap, qf] = refineFinal(model, snap, qf, target)
    % 末尾精确微调：无梯度随机贪心精修（L2 管线同款，绕开梯度局部极小）
    [q_rf, p_rf, ~] = refineRandomGreedy(model, qf, target, ...
        struct('layers', 4, 'steps_per_layer', 150, 'goal_eps', 0.006));
    if p_rf < 0.03   % 精修后位置足够精确（<0.03m）
        qf = q_rf;
        snap = [snap; qf];   % 精修无轨迹，追加末点与相邻快照一致
    end
end

function info = assembleInfo(model, snap, qf, target, graph, paths, tried, usedPath)
    cfg = model.cfg;
    [~, pe] = planarFK_L(qf, model.DH, cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(qf, model.DH, cfg.rod_offset_arr);
    dist_end = norm(pe - target(1:2));
    err_ang  = abs(wrapAngle(target(3) - th));
    success  = dist_end < 0.03;   % 精确微调后：末端位置 < 0.03m（角度作为报告项）
    info = struct('q_snapshot', snap, 't_seq', (1:size(snap,1)), ...
        'V_hist', [], 'q_final', qf, 'success', success, 'converged', success, ...
        'cancelled', false, 'iter', size(snap,1), ...
        'dist_end', dist_end, 'err_ang', err_ang, ...
        'error_code', ~success*2, 'method_used', 'multilayer');
    if isempty(usedPath), usedPath = paths{1}; end
    info.stats = struct('n_nodes', size(graph.nodes,1), 'n_edges', size(graph.edges,1), ...
        'n_candidates', numel(paths), 'tried', tried, 'path_len', size(snap,1));
    % 连通图叠加显示数据（供 GUI 走廊图叠加，与 method_graph 同格式）
    info.stats.nodes_xy = graph.nodes;
    if ~isempty(graph.edges)
        info.stats.edges_xy = [graph.nodes(graph.edges(:,1),:), graph.nodes(graph.edges(:,2),:)];
    else
        info.stats.edges_xy = zeros(0,4);
    end
    info.stats.seq_used = usedPath(:).';   % 实际采用的路径（节点索引序列）
    % 候选集：图搜索输出的所有可达齐整路径，供复杂运动情形选择
    info.candidates = cell(1, numel(paths));
    for i = 1:numel(paths)
        info.candidates{i} = struct('nodes', paths{i}(:).', ...
            'wp', graph.nodes(paths{i}, :));   % 每条候选的路点序列 [K×2]
    end
end

function info = fallbackSolve(model, q0, target, ~)
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
