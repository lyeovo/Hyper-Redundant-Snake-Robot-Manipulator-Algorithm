% eval_v2_compare.m — P0 可行性验证：新模型(v2) vs C1_ft vs RRT* 基线
%   同测试集（rng(5) 重采样）对比 直出率 / 成功率 / 末端精度
%   薄入口：实现统一到 evalPolicyCompare.m（原为 50 行独立实现，与 c1/c3 三份重复）
%   保留本文件以维持"调用方式即实验定义"的可重现性：rng 种子 / 样本数 / 模型列表。
root = fileparts(fileparts(mfilename('fullpath')));  cd(root);
addpath(fullfile(root, 'ArmSimulator2D'));
addpath(fullfile(root, 'scripts'));

evalPolicyCompare( ...
    {'rl_pipeline/policy_cvae_c1_ft.mat', 'rl_pipeline/policy_cvae_v2.mat'}, ...
    struct('name','混合150例', 'seed',5, 'n',150, 'difficulty',0), ...
    struct('with_baseline', true));
