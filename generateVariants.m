% generateVariants.m — 失败例变体示范生成（P0 可行性验证核心）
%   输入: failures_YYYYmmdd.mat（collectFailures 输出，含可解性诊断）
%   输出: demonstrations_var/（变体示范，可解才入库）
%   配置常量见下方（fail_file/n_var/rrt_ms/out_dir/seed 可改）
cd('D:/thuedu/26夏'); addpath('ArmSimulator2D');

fail_file = 'failures_20260810.mat';
n_var     = 40;        % 每失败例变体数
rrt_ms    = 20000;     % 变体可解性验证预算
out_dir   = 'demonstrations_var';
seed      = 1;
fi_start  = 1;         % 起始失败例索引（并行时按实例分片）
fi_count  = 0;         % 处理失败例数（0 = 全部）

% 变体扰动参数（可解性导向：失败域对齐——目标不扰动，只扰动 q0/障碍）
dq_sig  = 0.30;         % q0 关节扰动 rad
dt_sig  = 0.0;          % 目标位置扰动 m（0 = 原失败例目标不动，防分布漂移）
dt_ang  = 0.0;          % 目标角度扰动 rad
do_sig  = 0.03;         % 障碍位置扰动 m（微小，保场景相近）

rng(seed);
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
S = load(fail_file);
fails = S.fails; solvable = S.solvable;
sol_idx = find(solvable);                       % 只处理"可解但算法失败"的失败例
if fi_count == 0, fi_count = numel(sol_idx); end
sol_idx = sol_idx(fi_start:min(fi_start+fi_count-1, numel(sol_idx)));
fprintf('[var] 可解失败例 %d 个（%d-%d），每例 %d 变体（预算 %d）\n', ...
    numel(sol_idx), fi_start, fi_start+numel(sol_idx)-1, n_var, rrt_ms);

n_ok = 0; n_try = 0; t0 = tic; per_file = 100; shard = 0;
var_ex = [];  % 当前分片
for fi = 1:numel(sol_idx)
    f = fails{sol_idx(fi)};
    made = 0; tries = 0;
    while made < n_var && tries < n_var * 6      % 每例最多尝试 6× 变体数
        tries = tries + 1; n_try = n_try + 1;
        % ---- 变体采样 ----
        q0v = f.q0 + dq_sig * randn(1, 6);
        q0v = max(min(q0v, 3.0), -3.0);
        tv = f.target + [dt_sig*randn(1,2), dt_ang*randn(1,1)];
        % 障碍扰动（圆/矩形位置微移）
        obs = f.obs;
        if isfield(obs, 'circles') && size(obs.circles,1) > 0
            obs.circles(:, 1:2) = obs.circles(:, 1:2) + do_sig * randn(size(obs.circles,1), 2);
        end
        if isfield(obs, 'rects') && size(obs.rects,1) > 0
            obs.rects(:, 1:2) = obs.rects(:, 1:2) + do_sig * randn(size(obs.rects,1), 2);
        end
        m = createArmModel(struct('N',6,'L_seg',1.04393,'obstacles',obs));
        % ---- 可解性验证（巨型预算 RRT*）+ 示范 ----
        i = method_rrtstar(m, q0v, tv, struct('max_samples', rrt_ms));
        if ~i.success, continue; end
        % 后处理：短切 + 平滑
        traj = optimizeTraj(m, i.q_snapshot, struct('n_shortcut', 600, 'smooth_win', 3));
        var_ex(end+1).q0 = q0v;           %#ok<AGROW>
        var_ex(end).target = tv;
        var_ex(end).obstacles = obs;  % 字段名与现有示范一致（load_demos 读 obstacles）
        var_ex(end).traj = traj;
        made = made + 1; n_ok = n_ok + 1;
        % ---- 分片保存 ----
        if numel(var_ex) >= per_file
            shard = shard + 1;
            demos = var_ex; save(sprintf('%s/demo_%04d.mat', out_dir, shard), 'demos');
            fprintf('[var] 分片 %d 完成（%d 条, %.0fs, 累计可解率 %.0f%%）\n', ...
                shard, numel(var_ex), toc(t0), 100*n_ok/n_try);
            var_ex = [];
        end
    end
    fprintf('[var] 失败例 %d/%d → 变体 %d 条（尝试 %d）\n', fi, numel(sol_idx), made, tries);
end
if ~isempty(var_ex)
    shard = shard + 1;
    demos = var_ex; save(sprintf('%s/demo_%04d.mat', out_dir, shard), 'demos');
end
% manifest
manifest = struct('n_total', n_ok, 'n_try', n_try, 'rrt_max_samples', rrt_ms, ...
    'time_s', toc(t0), 'mean_plan_s', toc(t0)/max(n_ok,1));
save(sprintf('%s/manifest.mat', out_dir), 'manifest');
fprintf('=== 变体示范完成: 成功=%d 尝试=%d 可解率=%.0f%% 耗时=%.0fs (%s)\n', ...
    n_ok, n_try, 100*n_ok/n_try, toc(t0), out_dir);
