% eval_c3_compare.m — 难度 3 专项微调评估（40 例混合 + 20 例难度 3）
root = fileparts(mfilename('fullpath'));  cd(root);  addpath(fullfile(root, 'ArmSimulator2D'));

% 40 例混合难度
rng(11); T=40; mix=cell(1,T);
for k = 1:T
    [m, q0, tgt] = sampleTask2D(struct('N',6,'L_seg',1.04393,'difficulty',0));
    mix{k} = struct('m',m,'q0',q0,'tgt',tgt);
end
% 20 例难度 3（极限密集障碍）
rng(23); T3=20; hard=cell(1,T3);
for k = 1:T3
    [m, q0, tgt] = sampleTask2D(struct('N',6,'L_seg',1.04393,'difficulty',3));
    hard{k} = struct('m',m,'q0',q0,'tgt',tgt);
end

for fn = {'rl_pipeline/policy_cvae_c1.mat', 'rl_pipeline/policy_cvae_c1_ft.mat'}
    r1 = runEval(fn{1}, mix);  r2 = runEval(fn{1}, hard);
    fprintf('模型 %s\n  混合40例: succ=%d/%d 直出=%d coll=%d pos=%.4f 耗时=%.0fms\n 难度3 20例: succ=%d/%d 直出=%d coll=%d pos=%.4f 耗时=%.0fms\n', ...
        fn{1}, r1.succ, r1.n, r1.direct, r1.coll, r1.pos, r1.t*1000, ...
        r2.succ, r2.n, r2.direct, r2.coll, r2.pos, r2.t*1000);
end

function r = runEval(model_file, tasks)
    ns=0; nd=0; nc=0; pos=[]; tt=[];
    for k = 1:numel(tasks)
        t0 = tic;
        d = cvaePolicyDeploy(model_file, tasks{k}.m, tasks{k}.q0, tasks{k}.tgt);
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
    r = struct('succ',ns,'n',numel(tasks),'direct',nd,'coll',nc,'pos',mean(pos),'t',mean(tt));
end
