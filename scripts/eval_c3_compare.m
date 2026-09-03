% eval_c3_compare.m — 难度 3 专项微调评估（40 例混合 + 20 例难度 3）
%   薄入口：实现统一到 evalPolicyCompare.m（原为 50 行独立实现，与 c1/v2 三份重复）
%   保留本文件以维持"调用方式即实验定义"的可重现性：两组测试集的种子与样本数。
root = fileparts(fileparts(mfilename('fullpath')));  cd(root);
addpath(fullfile(root, 'ArmSimulator2D'));
addpath(fullfile(root, 'scripts'));

specs = [ struct('name','混合40例',  'seed',11, 'n',40, 'difficulty',0), ...
          struct('name','难度3 20例', 'seed',23, 'n',20, 'difficulty',3) ];

evalPolicyCompare( ...
    {'rl_pipeline/policy_cvae_c1.mat', 'rl_pipeline/policy_cvae_c1_ft.mat'}, ...
    specs, ...
    struct('with_baseline', false));   % 与历史行为一致：本脚本原本不跑 RRT* 基线
