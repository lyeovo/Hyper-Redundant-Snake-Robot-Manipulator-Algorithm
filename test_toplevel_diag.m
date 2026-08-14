% test_toplevel_diag.m — runTopLevel 目标估计偏差诊断
cd('D:/thuedu/26夏'); addpath('ArmSimulator2D');
se2m = @(p) [cos(p(3)) -sin(p(3)) p(1); sin(p(3)) cos(p(3)) p(2); 0 0 1];
inv_se2m = @(T) [T(1:2,1:2)' -T(1:2,1:2)'*T(1:2,3); 0 0 1];
wrapA = @(a) mod(a + pi, 2*pi) - pi;
rng(7);
m = createArmModel(struct('N', 6, 'L_seg', 1.04393));
Xh_true = [0.05, -0.02, 0.3];  Xg = [2.5, 1.0, 0.3];
K = 30;
th_seq = [0.2*sin(2*pi*linspace(0,1,K)'*1.5)+0.5, 0.3*cos(2*pi*linspace(0,1,K)'*1.0)+0.2, ...
          0.25*sin(2*pi*linspace(0,1,K)'*0.8), 0.15*cos(2*pi*linspace(0,1,K)'*1.2), ...
          0.1*sin(2*pi*linspace(0,1,K)'*1.8), 0.05*cos(2*pi*linspace(0,1,K)'*0.5)];
Xh_est = [0,0,0];
for k = 1:K
    th = th_seq(k,:) + 0.01*randn(1,6);
    [~,pe] = planarFK_L(th, m.DH, m.cfg.rod_offset_arr);
    teh = getEndEffectorAngle_L(th, m.DH, m.cfg.rod_offset_arr);
    pose_ee = [pe(1), pe(2), teh];
    Tc = se2m(pose_ee) * se2m(Xh_true);
    xc = inv_se2m(Tc) * [Xg(1:2)'; 1];
    x_cam = xc(1:2)' + 0.005*randn(1,2);
    yaw_cam = wrapA(Xg(3) - pose_ee(3) - Xh_true(3)) + 0.01*randn;
    if k >= 2
        pose_h(k,:) = pose_ee; obs_h(k,:) = [x_cam, yaw_cam];
        i0 = max(1, k-19);
        [Xh_est, st] = handEyeEstimate2D(pose_h(i0:k,:), obs_h(i0:k,:), struct('yaw_target', Xg(3)));
    else
        pose_h(1,:) = pose_ee; obs_h(1,:) = [x_cam, yaw_cam];
    end
    T_cam_est = se2m(pose_ee) * se2m(Xh_est);
    Xb = T_cam_est * [x_cam(1), x_cam(2), 1]';
    tgt_est = [Xb(1), Xb(2), wrapA(yaw_cam + pose_ee(3) + Xh_est(3))];
    if k == K || mod(k,10)==0
        fprintf('帧%d Xh_est=(%.3f,%.3f,%.3f) 目标估计=(%.3f,%.3f,%.3f) 目标真值=(%.3f,%.3f,%.3f) 位置差=%.3f\n', ...
            k, Xh_est, tgt_est, Xg, norm(tgt_est(1:2)-Xg(1:2)));
    end
end
