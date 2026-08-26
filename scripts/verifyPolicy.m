function res = verifyPolicy(opts)
%verifyPolicy 交付模型一键验证（复现 §5.7.11 验收）
%   res = verifyPolicy(opts)
%
%   opts（可缺省）:
%     .policy_file  策略文件（默认 'rl_pipeline/policy_cvae_n6.mat'）
%     .N / .L_seg   机械臂参数（默认 6 / 0.5，须与训练一致）
%     .n_test       测试任务数（默认 20；40 更稳，耗时更长）
%     .seed         测试集种子（默认 5；11 为另一组验收数据）
%     .ref_max_samples 基线 RRT* 预算（默认 1500）
%     .show         单任务可视化（默认 true，仅交互会话绘图）
%
%   用法（MATLAB 命令行）：
%     verifyPolicy();                                   % 默认 20 例
%     res = verifyPolicy(struct('n_test',40,'seed',11));% 复现 40 例验收
%
%   返回 res：.succ/.coll/.mean_pos/.mean_gap/.mean_gen_s（部署闭环）
%             .base_succ/.base_coll/.base_pos（RRT* 基线）
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'ArmSimulator2D'));
    if nargin < 1 || isempty(opts), opts = struct(); end
    policy_file = of(opts, 'policy_file', 'rl_pipeline/policy_cvae_n6.mat');
    N = of(opts, 'N', 6);
    L = of(opts, 'L_seg', 0.5);
    n_test  = of(opts, 'n_test', 20);
    seed    = of(opts, 'seed', 5);
    ref_ms  = of(opts, 'ref_max_samples', 1500);
    show    = of(opts, 'show', false);

    if exist(policy_file, 'file') ~= 2
        error('verifyPolicy:file', '策略文件不存在: %s（先训练或调整 policy_file）', policy_file);
    end

    fprintf('=== 交付模型验证 ===\n');
    fprintf('策略: %s | 机械臂 N=%d L=%.2f | 测试 %d 例 seed=%d\n\n', ...
        policy_file, N, L, n_test, seed);

    % ---- 1. 单任务演示（首例） ----
    rng(seed);
    [m, q0, tgt] = sampleTask2D(struct('N', N, 'L_seg', L));
    t0 = tic;
    out = cvaePolicyDeploy(policy_file, m, q0, tgt);
    dt = toc(t0);
    [~, pe] = planarFK_L(out.q_final, m.DH, m.cfg.rod_offset_arr);
    fprintf('【单任务演示】via=%s | 末端 [%.3f %.3f] (目标 [%.3f %.3f]) | 误差 %.3f m | 生成 %.0f ms\n', ...
        out.via, pe(1), pe(2), tgt(1), tgt(2), norm(pe - tgt(1:2)), dt*1000);
    if show && ~isempty(out.traj)
        figure('Name','CVAE 策略轨迹');
        hold on; axis equal; grid on;
        for k = 1:size(m.cfg.obstacles.circles, 1)
            c = m.cfg.obstacles.circles(k,:);
            viscircles(c(1:2), c(3), 'Color', [1 0.4 0.3]);
        end
        for k = 1:size(m.cfg.obstacles.rects, 1)
            r = m.cfg.obstacles.rects(k,:);
            ct = cos(r(3)); st = sin(r(3));
            corners = [r(1)-r(4)/2*ct+r(5)/2*st, r(2)-r(4)/2*st-r(5)/2*ct;
                       r(1)+r(4)/2*ct+r(5)/2*st, r(2)+r(4)/2*st-r(5)/2*ct;
                       r(1)+r(4)/2*ct-r(5)/2*st, r(2)+r(4)/2*st+r(5)/2*ct;
                       r(1)-r(4)/2*ct-r(5)/2*st, r(2)-r(4)/2*st+r(5)/2*ct];
            patch(corners(:,1), corners(:,2), [1 0.5 0.3], 'FaceAlpha', 0.25);
        end
        xs = zeros(size(out.traj,1),1); ys = zeros(size(out.traj,1),1);
        for k = 1:size(out.traj,1)
            [~, p] = planarFK_L(out.traj(k,:), m.DH, m.cfg.rod_offset_arr);
            xs(k) = p(1); ys(k) = p(2);
        end
        plot(xs, ys, '-', 'Color', [0.3 0.7 1], 'LineWidth', 1.5);
        plot(tgt(1), tgt(2), 'g+', 'MarkerSize', 12, 'LineWidth', 2);
        plot(q0(1)*0+pe(1), q0(2)*0+pe(2), 'ro', 'MarkerSize', 8);
        title(sprintf('CVAE 策略轨迹 (via=%s, 末端误差 %.3f m)', out.via, norm(pe-tgt(1:2))));
    end

    % ---- 2. 完整评估（与基线同测试集同预算） ----
    ev_opts = struct('n_test', n_test, 'ref_max_samples', ref_ms, ...
        'seed', seed, 'N', N, 'L_seg', L, 'verbose', false);
    rb = evaluatePolicy('rrtstar', ev_opts);
    rd = evaluatePolicy(@(mm,qq,tt) cvaePolicyDeploy(policy_file, mm, qq, tt), ev_opts);

    fprintf('\n【验收指标】\n');
    fprintf('%-24s %10s %10s %10s %10s %10s\n', '', '成功率', '碰撞率', '末端误差', '最优性差距', '生成耗时');
    fprintf('%-24s %9.0f%% %9.0f%% %9.3fm %11s %9.0fms\n', ...
        'RRT* 基线', rb.success_rate*100, rb.collision_rate*100, rb.mean_end_pos, '-', rb.mean_gen_s*1000);
    fprintf('%-24s %9.0f%% %9.0f%% %9.3fm %+10.1f%% %9.0fms\n', ...
        '部署闭环(CVAE+回退)', rd.success_rate*100, rd.collision_rate*100, rd.mean_end_pos, ...
        rd.mean_gap*100, rd.mean_gen_s*1000);

    res = struct('succ', rd.success_rate, 'coll', rd.collision_rate, ...
        'mean_pos', rd.mean_end_pos, 'mean_gap', rd.mean_gap, ...
        'mean_gen_s', rd.mean_gen_s, ...
        'base_succ', rb.success_rate, 'base_coll', rb.collision_rate, 'base_pos', rb.mean_end_pos);
    fprintf('\n结论: 碰撞率 %s | 成功率 %s | 最优性差距 %+.1f%%\n', ...
        iif(rd.collision_rate == 0, '0% ✓', sprintf('%.0f%% ✗', rd.collision_rate*100)), ...
        iif(rd.success_rate >= rb.success_rate - 0.1, ...
            sprintf('%.0f%% ≈ 基线 %.0f%% ✓', rd.success_rate*100, rb.success_rate*100), ...
            sprintf('%.0f%% < 基线 %.0f%%', rd.success_rate*100, rb.success_rate*100)), ...
        rd.mean_gap*100);
end

function v = of(s, f, d)
    if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
function r = iif(c, a, b)
    if c, r = a; else, r = b; end
end
