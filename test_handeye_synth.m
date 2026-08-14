% test_handeye_synth.m — 手眼估计合成验证（无噪声 + 含噪；yaw 约束 + 无约束）
cd('D:/thuedu/26夏'); addpath('ArmSimulator2D');
se2m = @(p) [cos(p(3)) -sin(p(3)) p(1); sin(p(3)) cos(p(3)) p(2); 0 0 1];
inv_se2m = @(T) [T(1:2,1:2)' -T(1:2,1:2)'*T(1:2,3); 0 0 1];
wrapA = @(a) mod(a + pi, 2*pi) - pi;
rng(0);
m = createArmModel(struct('N', 6, 'L_seg', 1.04393));
Xh_true = [0.05, -0.02, 0.3];
Xg = [2.5, 1.0, 0.3];
K = 12; pose = zeros(K, 3); obs = zeros(K, 3);
for k = 1:K
    th = [0.4*sin(k), 0.3*cos(k*0.8), 0.2*sin(k*1.3), 0.1*cos(k*0.6), 0.05*sin(k*2), 0.02*cos(k*0.4)];
    [~, pe] = planarFK_L(th, m.DH, m.cfg.rod_offset_arr);
    teh = getEndEffectorAngle_L(th, m.DH, m.cfg.rod_offset_arr);
    pose(k, :) = [pe(1), pe(2), teh];
    Tc = se2m(pose(k, :)) * se2m(Xh_true);
    xc = inv_se2m(Tc) * [Xg(1:2)'; 1];
    obs(k, :) = [xc(1), xc(2), wrapA(Xg(3) - pose(k, 3) - Xh_true(3))];
end
% 有 yaw 约束
[Xh, st] = handEyeEstimate2D(pose, obs, struct('yaw_target', Xg(3)));
fprintf('无噪声+yaw约束: 真值=(%.3f,%.3f,%.3f) 估计=(%.3f,%.3f,%.3f) 误差=%.5f 残差中位=%.6f\n', ...
    Xh_true, Xh, norm(Xh(1:2)-Xh_true(1:2)) + 0.5*abs(wrapA(Xh(3)-Xh_true(3))), st.resid_med);
% 含噪 + yaw 约束
rng(1);
obsN = obs + [0.005*randn(K, 2), 0.01*randn(K, 1)];
[Xhn, stn] = handEyeEstimate2D(pose, obsN, struct('yaw_target', Xg(3)));
fprintf('含噪+yaw约束:   估计=(%.3f,%.3f,%.3f) 误差=%.4f 残差中位=%.5f\n', ...
    Xhn, norm(Xhn(1:2)-Xh_true(1:2)) + 0.5*abs(wrapA(Xhn(3)-Xh_true(3))), stn.resid_med);
% 无 yaw 约束（θ_X 假设 0）——旋转不可观，仅平移评估
[Xh0, st0] = handEyeEstimate2D(pose, obs);
fprintf('无yaw约束:      估计=(%.3f,%.3f,%.3f) 位置误差=%.5f（θ_X 不可观，预期仅平移可估）\n', ...
    Xh0, norm(Xh0(1:2)-Xh_true(1:2)), st0.resid_med);
