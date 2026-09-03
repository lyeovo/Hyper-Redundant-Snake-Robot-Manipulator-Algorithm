% eval_c1_compare.m — C 档三方对比（40 例混合难度）
%   薄入口：实现统一到 evalPolicyCompare.m（原为 50 行独立实现，与 c3/v2 三份重复）
%   保留本文件以维持"调用方式即实验定义"的可重现性：rng 种子 / 样本数 / 模型列表。
root = fileparts(fileparts(mfilename('fullpath')));  cd(root);
addpath(fullfile(root, 'ArmSimulator2D'));
addpath(fullfile(root, 'scripts'));

evalPolicyCompare( ...
    {'rl_pipeline/policy_cvae_c1.mat', 'rl_pipeline/policy_cvae_n6.mat'}, ...
    struct('name','混合40例', 'seed',11, 'n',40, 'difficulty',0), ...
    struct('with_baseline', true));
