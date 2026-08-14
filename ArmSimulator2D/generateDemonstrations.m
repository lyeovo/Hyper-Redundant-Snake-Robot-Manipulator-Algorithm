function stats = generateDemonstrations(opts)
%generateDemonstrations 示范数据生成器（L2 模仿学习数据管线，维度无关）
%   stats = generateDemonstrations(opts)
%
%   流程：任务族采样（sampleTask2D）→ RRT* 规划（渐近最优）→ 平滑 → 碰撞校验
%         → 分片保存（断点续生成）→ manifest 汇总。
%
%   opts:
%     .n_demo          目标示范条数（默认 1000）
%     .out_dir         输出目录（默认 'demonstrations'）
%     .rrt_max_samples RRT* 采样预算（默认 3000，离线可大）
%     .per_file        每分片条数（默认 100）
%     .smooth_window   平滑窗口（默认 3，1=不平滑）
%     .seed            随机种子（可复现）
%     .N / .L_seg      机械臂参数（默认 4 / 1.0）
%
%   数据格式（3D 演进不破坏格式，只换采样器与几何适配层）：
%     每个分片文件 demo_XXXX.mat 内含 struct 数组 demos(1:n)，字段：
%       .q0       [1×D]    起始构型（D = C-space 维度；3D 时为 3N）
%       .target   [1×K]    目标（2D: [x,y,θ]；3D: SE(3) 表示）
%       .traj     [T×D]    关节轨迹（原始路径节点，含 q0 与终态）
%       .cost     路径代价（C-space 加权长度，渐近最优对象）
%       .obstacles struct('circles',...,'rects',...) 障碍描述
%       .meta     struct 生成参数（rrt 预算/平滑窗口/规划耗时）
%     manifest.mat：n_total、schema_version、生成参数、统计
%
%   示例：generateDemonstrations();                 % 1000 条到 demonstrations/
%         generateDemonstrations(struct('n_demo',100,'rrt_max_samples',2000));
    if nargin < 1 || isempty(opts), opts = struct(); end
    n_demo  = of(opts, 'n_demo', 1000);
    out_dir = of(opts, 'out_dir', 'demonstrations');
    rrt_ms  = of(opts, 'rrt_max_samples', 3000);
    per_file= of(opts, 'per_file', 100);
    swin    = of(opts, 'smooth_window', 3);
    diff_lv = of(opts, 'difficulty', 0);   % 0=随机 1-3；1-3=固定难度（专项生成用）
    seed    = of(opts, 'seed', []);
    N = of(opts, 'N', 4);
    L = of(opts, 'L_seg', 1.04393);
    if ~isempty(seed), rng(seed); end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    schema_version = 'demo-v1';   % 维度无关：traj=[T×D]，D 由采样器决定
    t_start = tic;
    % 续生成：检测已有分片（断点续跑，覆盖同目录）
    existing = dir(fullfile(out_dir, 'demo_*.mat'));
    n_shard0 = numel(existing);            % 已有分片数（假设每片满 per_file）
    n_ok = n_shard0 * per_file;
    n_try = 0;
    t_plan = [];  t_smooth = [];
    cost_hist = [];
    demos = struct('q0', [], 'target', [], 'traj', [], 'cost', [], ...
        'obstacles', [], 'meta', []);
    demos = repmat(demos, 1, 0);

    while n_ok < n_demo
        n_try = n_try + 1;
        % 1. 任务采样（2D 采样器；3D 演进替换此调用）
        % difficulty=0 → 随机 1-3（混合难度：覆盖稀疏到多障碍密集布局）
        [m, q0, tgt, obs] = sampleTask2D(struct('N', N, 'L_seg', L, 'difficulty', diff_lv));
        % 2. RRT* 规划（渐近最优示范）
        t0 = tic;
        info = method_rrtstar(m, q0, tgt, struct('max_samples', rrt_ms));
        t_plan(end+1) = toc(t0); %#ok<AGROW>
        if ~info.success, continue; end
        % 3. 轨迹优化（随机短切缩短 + 平滑去折角；碰撞回退由 optimizeTraj 内部保证）
        traj = info.q_snapshot;
        t0 = tic;
        if swin > 1 && size(traj, 1) > 3
            traj = optimizeTraj(m, traj, struct('n_shortcut', 600, 'smooth_win', swin, 'seed', []));
        end
        t_smooth(end+1) = toc(t0); %#ok<AGROW>
        % 4. 记录（校验已在规划/平滑中保证：全程无碰撞 + 终态达标）
        d = struct('q0', q0(:)', 'target', tgt(:)', 'traj', traj, ...
            'cost', sum(vecnorm(diff(traj, 1, 1), 2, 2)), ...
            'obstacles', obs, ...
            'meta', struct('rrt_max_samples', rrt_ms, 'smooth_window', swin, ...
                           'plan_s', t_plan(end), 'tree_nodes', info.stats.tree_nodes));
        demos(end+1) = d; %#ok<AGROW>
        cost_hist(end+1) = d.cost; %#ok<AGROW>
        n_ok = n_ok + 1;
        % 5. 分片保存（断点续生成）
        if mod(n_ok, per_file) == 0
            fid = fullfile(out_dir, sprintf('demo_%04d.mat', n_ok / per_file));
            save(fid, 'demos');
            demos = repmat(struct('q0',[],'target',[],'traj',[],'cost',[],'obstacles',[],'meta',[]), 1, 0);
            if mod(n_ok, per_file*10) == 0 || n_ok == n_demo
                fprintf('[demo] %d/%d | 尝试 %d | 平均规划 %.2fs | 平均代价 %.3f | %.0fs\n', ...
                    n_ok, n_demo, n_try, mean(t_plan), mean(cost_hist), toc(t_start));
            end
        end
    end
    % 收尾：保存未满一片的余量 + manifest
    if ~isempty(demos)
        fid = fullfile(out_dir, sprintf('demo_%04d.mat', ceil(n_ok / per_file)));
        save(fid, 'demos');
    end
    manifest = struct('schema_version', schema_version, 'n_total', n_ok, ...
        'n_try', n_try, 'N', N, 'L_seg', L, 'rrt_max_samples', rrt_ms, ...
        'smooth_window', swin, 'mean_cost', mean(cost_hist), ...
        'mean_plan_s', mean(t_plan), 'time_s', toc(t_start), 'date', datestr(now));
    save(fullfile(out_dir, 'manifest.mat'), '-struct', 'manifest');

    stats = manifest;
    stats.cost_std = std(cost_hist);
    fprintf('[demo] 完成: %d 条示范 → %s | 平均代价 %.3f±%.3f | 总耗时 %.0fs\n', ...
        n_ok, out_dir, mean(cost_hist), std(cost_hist), toc(t_start));
end

%% ---------- 工具 ----------
function ts = smoothTraj(traj, w)
    % 滑动均值平滑（沿时间维，保形，端点保持）
    ts = movmean(traj, w, 1);
    ts(1, :) = traj(1, :);   ts(end, :) = traj(end, :);
end

function ok = isFreeAll(model, traj)
    % 全程逐帧碰撞检查（含端点）
    ok = true;
    for k = 1:size(traj, 1)
        g = obsDistAll(model, traj(k, :));
        if ~isempty(g) && min(g) < model.cfg.rho0
            ok = false; return;
        end
    end
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
