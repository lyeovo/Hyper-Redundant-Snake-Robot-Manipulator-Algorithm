function test_errorModel()
%test_errorModel 误差模型 / 鲁棒性评估 / 反馈修正
    fprintf('== test_errorModel ==\n');
    fails = 0;
    rng(0);

    % ---- 1. 默认关闭：原样返回 ----
    m = testModel('X_target', [2.5,1.0], 'theta_target', 0.5);
    q0 = [0.3, -0.2, 0.5, -0.1];
    qa = errorModel(m, q0, zeros(1,4));
    fails = fails + assertTrue(isequal(qa, q0), '误差默认关闭原样返回');

    % ---- 2. 开启后噪声统计 ----
    m2 = testModel('error', struct('on', true, 'sigma_motor', 0.01, ...
        'backlash', 0.005, 'kappa', 0));
    dq_all = zeros(1000, 4);
    qp = zeros(1,4);
    for k = 1:1000
        qa = errorModel(m2, q0, qp);
        dq_all(k,:) = qa - q0;
        qp = qa;
    end
    est_sigma = std(dq_all(:));
    fails = fails + assertNear(est_sigma, 0.01, '噪声 σ 统计', 0.002);

    % ---- 3. assessRobustness 结构 ----
    rep = assessRobustness(m2, q0, [2.5,1.0,0.5], 500);
    fails = fails + assertTrue(isstruct(rep), 'assessRobustness 返回结构');
    fails = fails + assertTrue(rep.pos_err(1) <= rep.pos_err(2) + 1e-9 && ...
        rep.pos_err(2) <= rep.pos_err(3) + 1e-9, '误差分布 mean≤p95≤max');
    fails = fails + assertTrue(rep.collision_prob >= 0 && rep.collision_prob <= 1, '碰撞概率在 [0,1]');

    % ---- 4. feedbackCorrect 迭代收敛：重复增量修正应显著逼近目标 ----
    %   注：若目标方向沿 JJT 小特征值（臂近奇异方向），修正慢是物理特性，
    %   闭环修正与轨迹重规划配合使用（每 m 步小幅补偿）
    m3 = testModel('X_target', [2.5,1.0], 'theta_target', 0.5, ...
        'obstacles', struct('rects', [], 'circles', []));
    q = [0.4, -0.3, 0.6, -0.2];
    [~, pe] = planarFK_L(q, m3.DH, m3.cfg.rod_offset_arr);
    X_target = pe + [0.2, 0.1];         % 目标在实测前方 0.2,0.1
    d0 = norm(X_target - pe);
    q_cur = q;
    for k = 1:25
        [~, pe_k] = planarFK_L(q_cur, m3.DH, m3.cfg.rod_offset_arr);
        dq = feedbackCorrect(m3, q_cur, pe_k, X_target, 0.5);
        q_cur = q_cur + dq;
    end
    [~, pe_f] = planarFK_L(q_cur, m3.DH, m3.cfg.rod_offset_arr);
    d_end = norm(X_target - pe_f);
    fails = fails + assertTrue(d_end < d0 * 0.63, '反馈修正 25 步显著改善');

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_errorModel FAILED'); end
end

function f = assertTrue(b, name)
    if ~b
        fprintf('  [FAIL] %s\n', name);
        f = 1;
    else
        f = 0;
    end
end

function f = assertNear(a, b, name, tol)
    if abs(a - b) > tol
        fprintf('  [FAIL] %s: got %.6f, expected %.6f (±%.4f)\n', name, a, b, tol);
        f = 1;
    else
        f = 0;
    end
end
