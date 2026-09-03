function res = evalPolicyCompare(modelFiles, specs, opts)
%evalPolicyCompare CVAE 策略模型批量对比（scripts/ 实验脚本公共层）
%   res = evalPolicyCompare(modelFiles, specs, opts)
%
%   modelFiles : 策略 .mat 路径 cell（如 {'rl_pipeline/policy_cvae_c1.mat'}）
%   specs      : 测试集规格 struct 数组，每项
%                  .name       分组名（打印用，如 '混合40例'）
%                  .seed       rng 种子
%                  .n          样本数
%                  .difficulty 难度档（0=混合 / 3=极限密集障碍）
%                  .N .L_seg   可选，默认 6 / 1.04393
%   opts       : .pos_tol(0.01)      成功率末端位置判据
%                .rrt_samples(3000)  RRT* 基线采样预算
%                .with_baseline(true) 是否跑 RRT* 基线（+终态精修）
%                .verbose(true)
%
%   指标口径与历史 eval_c1/c3/v2_compare.m 完全一致：
%     成功 = 末端位置误差 < pos_tol 且 全程最小障碍距离 >= 0（无碰撞）
%     直出 = cvaePolicyDeploy 返回 via=='cvae'（策略直出，未回退采样层）
%     耗时 = cvaePolicyDeploy 单次调用墙钟均值（ms）
%
%   返回 res：.models（每模型每分组一行）/ .baseline（每分组一行，可选）
%
%   历史：eval_c1_compare / eval_c3_compare / eval_v2_compare 三个脚本各 50 行、
%   主循环完全相同，仅 rng 种子 / 样本数 / 模型列表不同。现统一到本函数，
%   三个脚本保留为薄入口以维持"调用方式即实验定义"的可重现性。
    if nargin < 3 || isempty(opts), opts = struct(); end
    pos_tol    = optget(opts, 'pos_tol', 0.01);
    rrt_samples= optget(opts, 'rrt_samples', 3000);
    with_base  = optget(opts, 'with_baseline', true);
    verbose    = optget(opts, 'verbose', true);
    if ischar(modelFiles), modelFiles = {modelFiles}; end

    % ---- 采样测试集（每组独立 rng） ----
    groups = cell(1, numel(specs));
    for gi = 1:numel(specs)
        sp = specs(gi);
        N  = optget(sp, 'N', 6);
        L  = optget(sp, 'L_seg', 1.04393);
        dif= optget(sp, 'difficulty', 0);
        rng(sp.seed);
        tasks = cell(1, sp.n);
        for k = 1:sp.n
            [m, q0, tgt] = sampleTask2D(struct('N',N, 'L_seg',L, 'difficulty',dif));
            tasks{k} = struct('m',m, 'q0',q0, 'tgt',tgt);
        end
        groups{gi} = struct('name', optget(sp,'name',sprintf('组%d',gi)), 'tasks', {tasks});
    end

    % ---- 逐模型评估 ----
    res = struct('models', [], 'baseline', []);
    rows = [];
    for mi = 1:numel(modelFiles)
        fn = modelFiles{mi};
        for gi = 1:numel(groups)
            r = runModel(fn, groups{gi}.tasks, pos_tol);
            r.model = fn;  r.group = groups{gi}.name;
            rows = [rows, r]; %#ok<AGROW>
            if verbose
                fprintf('模型 %s | %s: succ=%d/%d 直出=%d coll=%d pos=%.4f 耗时=%.0fms\n', ...
                    fn, r.group, r.succ, r.n, r.direct, r.coll, r.pos, r.t*1000);
            end
        end
    end
    res.models = rows;

    % ---- RRT* 基线（同样终态精修，公平对比） ----
    if with_base
        b = [];
        for gi = 1:numel(groups)
            r = runBaseline(groups{gi}.tasks, pos_tol, rrt_samples);
            r.group = groups{gi}.name;
            b = [b, r]; %#ok<AGROW>
            if verbose
                fprintf('RRT*基线(精修) | %s: succ=%d/%d pos=%.4f 耗时=%.0fms\n', ...
                    r.group, r.succ, r.n, r.pos, r.t*1000);
            end
        end
        res.baseline = b;
    end
end

%% ---------- 内部 ----------
function r = runModel(model_file, tasks, pos_tol)
% 策略部署闭环（CVAE 直出 + 失败回退 RRT*），统计成功/直出/碰撞/末端精度/耗时
    ns=0; nd=0; nc=0; pos=[]; tt=[];
    for k = 1:numel(tasks)
        tk = tasks{k};
        t0 = tic;
        d  = cvaePolicyDeploy(model_file, tk.m, tk.q0, tk.tgt);
        tt(end+1) = toc(t0);
        [~, pe] = planarFK_L(d.q_final, tk.m.DH, tk.m.cfg.rod_offset_arr);
        err = norm(pe - tk.tgt(1:2));
        gmin = trajMinGap(tk.m, d.traj);
        coll = gmin < 0;
        if strcmp(d.via, 'cvae'), nd = nd + 1; end
        if err < pos_tol && ~coll, ns = ns + 1; end
        if coll, nc = nc + 1; end
        pos(end+1) = err;
    end
    r = struct('succ',ns, 'n',numel(tasks), 'direct',nd, 'coll',nc, ...
               'pos',mean(pos), 't',mean(tt));
end

function r = runBaseline(tasks, pos_tol, rrt_samples)
% RRT* 基线 + refineRandomGreedy 终态精修
    ns=0; pos=[]; tt=[];
    for k = 1:numel(tasks)
        tk = tasks{k};
        t0 = tic;
        i = method_rrtstar(tk.m, tk.q0, tk.tgt, struct('max_samples', rrt_samples));
        if i.success
            qf = refineRandomGreedy(tk.m, i.q_final, tk.tgt, ...
                struct('layers',3, 'steps_per_layer',120, 'goal_eps',0.008));
        else
            qf = i.q_final;
        end
        tt(end+1) = toc(t0);
        [~, pe] = planarFK_L(qf, tk.m.DH, tk.m.cfg.rod_offset_arr);
        err = norm(pe - tk.tgt(1:2));
        if err < pos_tol, ns = ns + 1; end
        pos(end+1) = err;
    end
    r = struct('succ',ns, 'n',numel(tasks), 'direct',0, 'coll',0, ...
               'pos',mean(pos), 't',mean(tt));
end

function gmin = trajMinGap(m, traj)
% 整条轨迹的最小障碍间隙（侵入为负）
    gmin = inf;
    if isempty(traj), return; end
    for j = 1:size(traj, 1)
        g = obsDistAll(m, traj(j,:));
        if ~isempty(g), gmin = min(gmin, min(g)); end
    end
end
