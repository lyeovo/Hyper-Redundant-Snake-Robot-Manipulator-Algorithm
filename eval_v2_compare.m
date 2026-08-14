% eval_v2_compare.m — P0 可行性验证：新模型(v2) vs C1_ft vs RRT* 基线
%   同测试集（rng(5) 重采样）对比 直出率 / 成功率 / 末端精度
root = fileparts(mfilename('fullpath'));  cd(root);  addpath(fullfile(root, 'ArmSimulator2D'));
rng(5); T = 150; tasks = cell(1, T);
for k = 1:T
    [m, q0, tgt] = sampleTask2D(struct('N',6,'L_seg',1.04393,'difficulty',0));
    tasks{k} = struct('m',m,'q0',q0,'tgt',tgt);
end

models = {'rl_pipeline/policy_cvae_c1_ft.mat', 'rl_pipeline/policy_cvae_v2.mat'};
for mi = 1:numel(models)
    fn = models{mi};
    ns=0; nd=0; nc=0; pos=[]; tt=[];
    for k = 1:T
        t0 = tic;
        d = cvaePolicyDeploy(fn, tasks{k}.m, tasks{k}.q0, tasks{k}.tgt);
        tt(end+1) = toc(t0);
        [~, pe] = planarFK_L(d.q_final, tasks{k}.m.DH, tasks{k}.m.cfg.rod_offset_arr);
        err = norm(pe - tasks{k}.tgt(1:2));
        gmin = inf;
        for j = 1:size(d.traj,1)
            g = obsDistAll(tasks{k}.m, d.traj(j,:));
            if ~isempty(g), gmin = min(gmin, min(g)); end
        end
        if strcmp(d.via,'cvae'), nd = nd+1; end
        if err < 0.01 && gmin >= 0, ns = ns+1; end
        if gmin < 0, nc = nc+1; end
        pos(end+1) = err;
    end
    fprintf('模型 %s: succ=%d/%d 直出=%d coll=%d pos=%.4f 耗时=%.0fms\n', ...
        fn, ns, T, nd, nc, mean(pos), mean(tt)*1000);
end
% RRT* 基线（3000 样本 + 精修）
ns=0; pos=[]; tt=[];
for k = 1:T
    t0 = tic;
    i = method_rrtstar(tasks{k}.m, tasks{k}.q0, tasks{k}.tgt, struct('max_samples',3000));
    if i.success
        qf = refineRandomGreedy(tasks{k}.m, i.q_final, tasks{k}.tgt, ...
            struct('layers', 3, 'steps_per_layer', 120, 'goal_eps', 0.008));
    else
        qf = i.q_final;
    end
    tt(end+1) = toc(t0);
    [~, pe] = planarFK_L(qf, tasks{k}.m.DH, tasks{k}.m.cfg.rod_offset_arr);
    err = norm(pe - tasks{k}.tgt(1:2));
    if err < 0.01, ns = ns+1; end
    pos(end+1) = err;
end
fprintf('RRT*基线: succ=%d/%d pos=%.4f 耗时=%.0fms\n', ns, T, mean(pos), mean(tt)*1000);
