% eval_c1_compare.m — C 档三方对比（40 例混合难度）
root = fileparts(mfilename('fullpath'));  cd(root);  addpath(fullfile(root, 'ArmSimulator2D'));
rng(11); T = 40; tasks = cell(1, T);
for k = 1:T
    [m, q0, tgt] = sampleTask2D(struct('N',6,'L_seg',1.04393,'difficulty',0));
    tasks{k} = struct('m',m,'q0',q0,'tgt',tgt);
end

% 部署闭环（策略直出 + 回退 RRT*）
for fn = {'rl_pipeline/policy_cvae_c1.mat', 'rl_pipeline/policy_cvae_n6.mat'}
    ns=0; nd=0; nc=0; pos=[]; tt=[];
    for k = 1:T
        t0 = tic;
        d = cvaePolicyDeploy(fn{1}, tasks{k}.m, tasks{k}.q0, tasks{k}.tgt);
        tt(end+1) = toc(t0);
        [~, pe] = planarFK_L(d.q_final, tasks{k}.m.DH, tasks{k}.m.cfg.rod_offset_arr);
        err = norm(pe - tasks{k}.tgt(1:2));
        gmin = inf;
        for j = 1:size(d.traj,1)
            g = obsDistAll(tasks{k}.m, d.traj(j,:));
            if ~isempty(g), gmin = min(gmin, min(g)); end
        end
        coll = gmin < 0;
        if strcmp(d.via,'cvae'), nd = nd+1; end
        if err < 0.01 && ~coll, ns = ns+1; end
        if coll, nc = nc+1; end
        pos(end+1) = err;
    end
    fprintf('模型 %s: succ=%d/40 直出=%d coll=%d pos=%.4f 耗时=%.0fms\n', ...
        fn{1}, ns, nd, nc, mean(pos), mean(tt)*1000);
end

% RRT* 基线（+ 同样终态精修，公平对比）
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
fprintf('RRT*基线(精修): succ=%d/40 pos=%.4f 耗时=%.0fms\n', ns, mean(pos), mean(tt)*1000);
