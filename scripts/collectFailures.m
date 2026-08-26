% collectFailures.m — 失败任务采样器：批量评估当前部署闭环，收集失败例并诊断可解性
%   输出 failures_YYYYmmdd.mat：failures（q0/target/obs_desc/solvable 字段）
cd(fileparts(fileparts(mfilename('fullpath')))); addpath('ArmSimulator2D');
rng(5);
N_TASK  = 300;              % 采样任务数
POLICY  = 'rl_pipeline/policy_cvae_c1_ft.mat';
VERIFY_MS = 20000;          % 可解性诊断：巨型预算 RRT* 样本数
VERIFY_SEEDS = 3;           % 多 seed 诊断

fails = {};                 % 失败例
stats = struct('n_task', 0, 'succ', 0, 'fail', 0, 'n_solvable', 0, 'n_unsolvable', 0);
for k = 1:N_TASK
    [m, q0, tgt, obs] = sampleTask2D(struct('N',6,'L_seg',1.04393,'difficulty',0));
    d = cvaePolicyDeploy(POLICY, m, q0, tgt);
    [~, pe] = planarFK_L(d.q_final, m.DH, m.cfg.rod_offset_arr);
    err = norm(pe - tgt(1:2));
    gmin = inf;
    for j = 1:size(d.traj,1)
        g = obsDistAll(m, d.traj(j,:));
        if ~isempty(g), gmin = min(gmin, min(g)); end
    end
    ok = err < 0.01 && gmin >= 0;
    stats.n_task = stats.n_task + 1;
    if ok
        stats.succ = stats.succ + 1;
    else
        stats.fail = stats.fail + 1;
        fails{end+1} = struct('q0', q0, 'target', tgt, 'obs', obs, ...
            'via', d.via, 'err', err, 'coll', gmin < 0); %#ok<AGROW>
    end
    if mod(k, 50) == 0
        fprintf('[eval] %d/%d 成功=%d 失败=%d 直出率诊断中…\n', k, N_TASK, stats.succ, stats.fail);
    end
end

% ---- 失败例可解性诊断（巨型预算 RRT* 多 seed） ----
fprintf('[diag] %d 个失败例可解性诊断（%d seed × %d 样本）…\n', numel(fails), VERIFY_SEEDS, VERIFY_MS);
solvable = false(1, numel(fails));
for k = 1:numel(fails)
    f = fails{k};
    m = createArmModel(struct('N',6,'L_seg',1.04393,'obstacles',f.obs));
    for s = 1:VERIFY_SEEDS
        i = method_rrtstar(m, f.q0, f.target, struct('max_samples', VERIFY_MS));
        if i.success
            solvable(k) = true;
            break;
        end
    end
    stats.n_solvable = stats.n_solvable + solvable(k);
    stats.n_unsolvable = stats.n_unsolvable + ~solvable(k);
    if mod(k, 5) == 0
        fprintf('[diag] %d/%d 可解=%d 疑似不可解=%d\n', k, numel(fails), ...
            stats.n_solvable, stats.n_unsolvable);
    end
end

fname = fullfile('data', sprintf('failures_%s.mat', datestr(now, 'yyyymmdd')));
save(fname, 'fails', 'stats', 'solvable');
fprintf('=== 汇总: 任务=%d 成功=%d(%d%%) 失败=%d\n', stats.n_task, stats.succ, ...
    round(100*stats.succ/stats.n_task), stats.fail);
fprintf('    失败例中: 可解=%d 疑似不可解=%d（%s）\n', stats.n_solvable, ...
    stats.n_unsolvable, fname);
