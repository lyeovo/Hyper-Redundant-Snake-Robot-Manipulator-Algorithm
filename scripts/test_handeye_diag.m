% test_handeye_diag.m — 手眼估计逐帧诊断（A/B 旋转一致性）
cd(fileparts(fileparts(mfilename('fullpath')))); addpath('ArmSimulator2D');
se2m = @(p) [cos(p(3)) -sin(p(3)) p(1); sin(p(3)) cos(p(3)) p(2); 0 0 1];
inv_se2m = @(T) [T(1:2,1:2)' -T(1:2,1:2)'*T(1:2,3); 0 0 1];
wrapA = @(a) mod(a + pi, 2*pi) - pi;
rng(0);
m = createArmModel(struct('N', 6, 'L_seg', 1.04393));
Xh_true = [0.05, -0.02, 0.3];
Xg = [2.5, 1.0, 0.3];
K = 6; pose = zeros(K, 3); obs = zeros(K, 3);
for k = 1:K
    th = [0.4*sin(k), 0.3*cos(k*0.8), 0.2*sin(k*1.3), 0.1*cos(k*0.6), 0.05*sin(k*2), 0.02*cos(k*0.4)];
    [~, pe] = planarFK_L(th, m.DH, m.cfg.rod_offset_arr);
    teh = getEndEffectorAngle_L(th, m.DH, m.cfg.rod_offset_arr);
    pose(k, :) = [pe(1), pe(2), teh];
    Tc = se2m(pose(k, :)) * se2m(Xh_true);
    xc = inv_se2m(Tc) * [Xg(1:2)'; 1];
    obs(k, :) = [xc(1), xc(2), wrapA(Xg(3) - pose(k, 3) - Xh_true(3))];
    fprintf('帧%d 末端位姿=(%.3f,%.3f,%.3f) 观测=(%.3f,%.3f,%.3f)\n', k, pose(k,:), obs(k,:));
end
% 逐帧 A/B 旋转
A1 = [cos(pose(1,3)) -sin(pose(1,3)) pose(1,1); sin(pose(1,3)) cos(pose(1,3)) pose(1,2); 0 0 1];
P1 = [obs(1,1), obs(1,2); obs(1,1)+cos(obs(1,3)), obs(1,2)+sin(obs(1,3))];
for i = 2:K
    Ai = [cos(pose(i,3)) -sin(pose(i,3)) pose(i,1); sin(pose(i,3)) cos(pose(i,3)) pose(i,2); 0 0 1];
    A = inv_se2m(A1) * Ai;
    thA = atan2(A(2,1), A(1,1));
    Pi = [obs(i,1), obs(i,2); obs(i,1)+cos(obs(i,3)), obs(i,2)+sin(obs(i,3))];
    c1 = mean(P1,1); c2 = mean(Pi,1);
    W1 = P1 - c1; W2 = Pi - c2;
    [U,~,V] = svd(W2' * W1);
    RB = U * V';
    thB = atan2(RB(2,1), RB(1,1));
    fprintf('帧%d: thA=%.4f thB=%.4f 差=%.4f\n', i, thA, thB, thA-thB);
end
