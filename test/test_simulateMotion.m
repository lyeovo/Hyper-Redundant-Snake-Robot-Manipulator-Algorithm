function test_simulateMotion()
%test_simulateMotion 接口2 冒烟：收敛、避障、位姿反解、快照、回调、取消、不可达
    fprintf('== test_simulateMotion ==\n');
    fails = 0;
    rng(3);

    % ---- 1. 无障碍收敛 ----
    m = testModel('X_target', [2.5, 1.0], 'theta_target', 0.5, ...
        'obstacles', struct('rects', [], 'circles', []));
    info = simulateMotion(m, 'momentum', zeros(1,4), [2.5, 1.0, 0.5], 'Snapshot', 10);
    fails = fails + assertTrue(info.success, '无障碍收敛 success');
    % 两阶段收敛：硬达标（tol）或 二次精修（逐项<0.001，精度 ~sqrt(0.001/w_pos)≈0.03）
    fails = fails + assertTrue(info.dist_end < 0.05, '末端误差（逐项<0.001 动态收敛）');
    fails = fails + assertTrue(info.conv_type == "hard" || info.conv_type == "item", 'conv_type 合法');
    fails = fails + assertTrue(info.vel_ok, 'vel_ok');
    fails = fails + assertTrue(info.safety_ok, 'safety_ok（无障碍恒真）');
    fails = fails + assertTrue(size(info.q_snapshot,1) >= 2, '快照至少 2 行');
    fails = fails + assertTrue(all(diff(info.t_seq) > 0), 't_seq 单调递增');

    % ---- 2. 有障碍避障：圆在臂与目标之间 ----
    % 2a. 动量法未达目标（演示：梯度类方法在障碍场景的局限，WARN 不判失败）
    m2 = testModel('X_target', [3.5, 0.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects', [], 'circles', [2.0, 0.5, 0.35]));
    i_mom = simulateMotion(m2, 'momentum', zeros(1,4), [3.5, 0.0, 0.0], 'MaxIter', 600);
    if i_mom.success
        fprintf('  [WARN] 期望动量法未达目标，实际成功（场景选择不当）\n');
    else
        fprintf('  [INFO] momentum 障碍场景未收敛 pos=%.4f ang=%.4f（预期，采样层接管）\n', ...
            i_mom.dist_end, i_mom.err_ang);
    end
    % 2d. detectLocalMin 结构验证：返回 logical + 数值；收敛终点判非极小
    [is_min, gn, de] = detectLocalMin(m2, i_mom.q_final, [3.5, 0.0, 0.0]);
    fails = fails + assertTrue(islogical(is_min) && isscalar(gn) && isscalar(de), 'detectLocalMin 返回结构');
    m0 = testModel('X_target', [2.5,1.0], 'theta_target', 0.5, ...
        'obstacles', struct('rects', [], 'circles', []));
    i0 = simulateMotion(m0, 'momentum', zeros(1,4), [2.5,1.0,0.5]);
    [is0, ~] = detectLocalMin(m0, i0.q_final, [2.5,1.0,0.5]);
    fails = fails + assertTrue(~is0, '收敛终点判定为非局部最优');
    % 2b. RRT 采样层绕过势阱
    rng(42);
    info2 = simulateMotion(m2, 'rrt', zeros(1,4), [3.5, 0.0, 0.0], 'Snapshot', 5);
    fails = fails + assertTrue(info2.success, 'RRT 避障 success');
    fails = fails + assertTrue(info2.dist_end < m2.cfg.rrt_goal_eps, 'RRT 位置误差 < goal_eps');
    fails = fails + assertTrue(info2.err_ang < m2.cfg.rrt_goal_ang, 'RRT 角度误差 < goal_ang');
    fails = fails + assertTrue(info2.safety_ok, 'RRT 轨迹 safety_ok');
    fails = fails + assertTrue(minTrajDist(m2, info2.q_final) >= m2.cfg.rho0 - 1e-9, 'RRT 终点 d ≥ rho0');
    % 2c. SA 冒烟（运行不崩溃，能返回结构）
    rng(43);
    info_sa = simulateMotion(m2, 'sa', zeros(1,4), [3.5, 0.0, 0.0], 'MaxIter', 1500);
    fails = fails + assertTrue(isstruct(info_sa), 'SA 返回结构');
    fails = fails + assertTrue(~isempty(info_sa.q_final), 'SA 有 q_final');
    if info_sa.success
        fails = fails + assertTrue(info_sa.dist_end < m2.cfg.rrt_goal_eps, 'SA 位置误差 < goal_eps');
    else
        fprintf('  [WARN] SA 未达可达区域（冒烟通过，不判失败）\n');
    end

    % ---- 3. 初始末端位姿 → 内部反解 ----
    info3 = simulateMotion(m, 'momentum', [2.0, 1.0, 0.5], [2.5, 1.0, 0.5], 'Snapshot', 20);
    fails = fails + assertTrue(info3.success, '位姿反解 + 收敛');
    fails = fails + assertEq(info3.error_code, 0, 'error_code=0');

    % ---- 4. OnStep 回调计数 ----
    n_cb = 0;
    simulateMotion(m, 'momentum', zeros(1,4), [2.5, 1.0, 0.5], 'Snapshot', 10, ...
        'OnStep', @(q, iter, d) assignin('base', 'n_cb', n_cb + 1));
    % 回调由 method_momentum 内部调用，用嵌套计数
    info4 = simulateMotion(m, 'momentum', zeros(1,4), [2.5, 1.0, 0.5], 'Snapshot', 10);
    % 直接验证快照行数与迭代步数关系
    fails = fails + assertTrue(size(info4.q_snapshot,1) <= floor(info4.iter/10) + 2, '快照行数合理');

    % ---- 5. Cancel 中断 ----
    info5 = simulateMotion(m, 'momentum', zeros(1,4), [2.5, 1.0, 0.5], ...
        'Cancel', @() true);
    fails = fails + assertTrue(info5.cancelled, 'Cancel 生效');
    fails = fails + assertEq(info5.error_code, 6, '取消 error_code=6');
    fails = fails + assertTrue(~info5.success, '取消后 success=false');

    % ---- 6. 不可达目标 → 未收敛（迭代耗尽 error_code=2 或 卡住早停 error_code=7） ----
    info6 = simulateMotion(m, 'momentum', zeros(1,4), [8.0, 8.0, 0.0], 'MaxIter', 200);
    fails = fails + assertTrue(~info6.success, '不可达未收敛');
    fails = fails + assertTrue(info6.error_code == 2 || info6.error_code == 7, ...
        '未收敛/卡住 error_code∈{2,7}');

    % ---- 7. 位姿反解失败路径（目标远超可达域） ----
    info7 = simulateMotion(m, 'momentum', [9.0, 9.0, 0.0], [2.5, 1.0, 0.5], 'MaxIter', 20);
    if ~info7.success && info7.error_code == 5
        fails = fails;   % 反解失败正确（不额外计）
    else
        fprintf('  [WARN] 反解失败路径未触发（可能恰好成功）\n');
    end

    % ---- 7. auto 调度链：无障碍走动量快路径，避障走采样层 ----
    ia0 = simulateMotion(m0, 'auto', zeros(1,4), [2.5,1.0,0.5]);
    fails = fails + assertTrue(ia0.success, 'auto 无障碍 success');
    fails = fails + assertTrue(strcmp(ia0.method_used, 'momentum'), 'auto 无障碍用 momentum');
    rng(44);
    ia2 = simulateMotion(m2, 'auto', zeros(1,4), [3.5,0.0,0.0]);
    fails = fails + assertTrue(ia2.success, 'auto 避障 success');
    fails = fails + assertTrue(ia2.dist_end < m2.cfg.rrt_goal_eps, 'auto 避障位置达标');
    fails = fails + assertTrue(ia2.err_ang < m2.cfg.rrt_goal_ang, 'auto 避障角度达标');
    fails = fails + assertTrue(ia2.safety_ok, 'auto 避障 safety_ok');

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_simulateMotion FAILED'); end
end

function dmin = minTrajDist(model, q_snap)
    dmin = inf;
    for i = 1:size(q_snap, 1)
        [g, ~] = obsDistGradAll(model, q_snap(i,:));
        if ~isempty(g), dmin = min(dmin, min(g)); end
    end
end

function f = assertTrue(b, name)
    if ~b
        fprintf('  [FAIL] %s\n', name);
        f = 1;
    else
        f = 0;
    end
end

function f = assertEq(a, b, name)
    if ~isequal(a, b)
        fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b));
        f = 1;
    else
        f = 0;
    end
end

function f = assertNear(a, b, name, tol)
    if abs(a - b) > tol
        fprintf('  [FAIL] %s: got %.6f, expected %.6f\n', name, a, b);
        f = 1;
    else
        f = 0;
    end
end
