% eval_failures.m — 失败例专项验证：C1_ft vs v2 在 37 个失败例上的救回率
%   变体训练的目标 = 救回"可解失败例"（failures_20260810.mat 中 solvable=1 的 11 个）
root = fileparts(fileparts(mfilename('fullpath')));  cd(root);  addpath(fullfile(root, 'ArmSimulator2D'));
S = load(fullfile(root, 'data', 'failures_20260810.mat'));
fails = S.fails; solvable = S.solvable;
sol_idx = find(solvable);   % 只测 11 个可解失败例（变体训练的直接目标，回退耗时可控）
models = {'rl_pipeline/policy_cvae_c1_ft.mat', 'rl_pipeline/policy_cvae_v2.mat', 'rl_pipeline/policy_cvae_v3.mat'};
for mi = 1:numel(models)
    fn = models{mi};
    ns=0; nd=0;
    for k = 1:numel(sol_idx)
        f = fails{sol_idx(k)};
        m = createArmModel(struct('N',6,'L_seg',1.04393,'obstacles',f.obs));
        d = cvaePolicyDeploy(fn, m, f.q0, f.target);
        [~, pe] = planarFK_L(d.q_final, m.DH, m.cfg.rod_offset_arr);
        err = norm(pe - f.target(1:2));
        gmin = inf;
        for j = 1:size(d.traj,1)
            g = obsDistAll(m, d.traj(j,:));
            if ~isempty(g), gmin = min(gmin, min(g)); end
        end
        ok = err < 0.01 && gmin >= 0;
        if ok, ns = ns+1; end
        if strcmp(d.via,'cvae'), nd = nd+1; end
    end
    fprintf('%s: 可解失败例救回 %d/11 | 直出 %d/11\n', fn, ns, nd);
end
