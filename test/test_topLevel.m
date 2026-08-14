function test_topLevel()
%test_topLevel 顶层闭环估计测试（S1-S3：手眼在线估计 + Kalman 目标估计 + 任务触发）
    addpath('ArmSimulator2D');
    n_pass = 0;  n_fail = 0;
    chk = @(cond, name) deal(chk_impl(cond, name, n_pass, n_fail));

    % 1. 手眼估计合成收敛（无噪声 + yaw 约束 → 精确恢复）
    rng(0);
    se2m = @(p) [cos(p(3)) -sin(p(3)) p(1); sin(p(3)) cos(p(3)) p(2); 0 0 1];
    inv2 = @(T) [T(1:2,1:2)' -T(1:2,1:2)'*T(1:2,3); 0 0 1];
    wrapA = @(a) mod(a + pi, 2*pi) - pi;
    m = createArmModel(struct('N', 6, 'L_seg', 1.04393));
    Xh_true = [0.05, -0.02, 0.3];  Xg = [2.5, 1.0, 0.3];
    K = 12; pose = zeros(K, 3); obs = zeros(K, 3);
    for k = 1:K
        th = [0.4*sin(k), 0.3*cos(k*0.8), 0.2*sin(k*1.3), 0.1*cos(k*0.6), 0.05*sin(k*2), 0.02*cos(k*0.4)];
        [~, pe] = planarFK_L(th, m.DH, m.cfg.rod_offset_arr);
        teh = getEndEffectorAngle_L(th, m.DH, m.cfg.rod_offset_arr);
        pose(k, :) = [pe(1), pe(2), teh];
        Tc = se2m(pose(k, :)) * se2m(Xh_true);
        xc = inv2(Tc) * [Xg(1:2)'; 1];
        obs(k, :) = [xc(1), xc(2), wrapA(Xg(3) - pose(k, 3) - Xh_true(3))];
    end
    [Xh, st] = handEyeEstimate2D(pose, obs, struct('yaw_target', Xg(3)));
    err_h = norm(Xh(1:2) - Xh_true(1:2)) + 0.5*abs(wrapA(Xh(3) - Xh_true(3)));
    ok1 = err_h < 1e-3 && st.yaw_used;
    if ok1, n_pass = n_pass + 1; else, n_fail = n_fail + 1; end
    fprintf('[%s] 手眼合成收敛: 误差=%.5f\n', tern(ok1), err_h);

    % 2. Kalman 目标估计（偏差首帧后收敛）
    kf = kalmanTarget2D();
    kf = kalmanTargetUpdate(kf, [10, 10, 0.3]);
    for i = 1:8, kf = kalmanTargetUpdate(kf, [2.5, 1.0, 0.3]); end
    ok2 = norm(kf.x(1:2)' - [2.5, 1.0]) < 0.2;
    if ok2, n_pass = n_pass + 1; else, n_fail = n_fail + 1; end
    fprintf('[%s] Kalman 收敛: 状态=(%.3f,%.3f)\n', tern(ok2), kf.x(1), kf.x(2));

    % 3. 顶层闭环端到端（mock：手眼收敛 + 目标估计 + 任务触发）
    rng(7);
    top = runTopLevel(struct('target_true', Xg, 'T_ee_cam_true', Xh_true, 'n_frames', 30));
    ok3 = ~isempty(top.converged_frame) && top.err_target(end) < 0.05 && ...
          top.task.success && top.task.dist_end < 0.05;
    if ok3, n_pass = n_pass + 1; else, n_fail = n_fail + 1; end
    fprintf('[%s] 顶层闭环: 手眼收敛帧=%s 目标末误差=%.4f 任务成功=%d 末端误差=%.4f\n', ...
        tern(ok3), num2str(top.converged_frame), top.err_target(end), ...
        top.task.success, top.task.dist_end);

    fprintf('--- test_topLevel: %d 通过, %d 失败 ---\n', n_pass, n_fail);
    if n_fail > 0, error('test_topLevel:fail', '%d 项断言失败', n_fail); end
end

function s = tern(c)
    if c, s = 'PASS'; else, s = 'FAIL'; end
end
function [s] = chk_impl(cond, name, n_pass, n_fail) %#ok<DEFNU> 未用（保留模板）
    s = '';
end
