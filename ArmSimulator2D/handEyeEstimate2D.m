function [X_hand, stats] = handEyeEstimate2D(pose_base_ee, obs_cam, opts)
%handEyeEstimate2D 在线手眼标定（2D 平面）
%   [X_hand, stats] = handEyeEstimate2D(pose_base_ee, obs_cam, opts)
%   输入：
%     pose_base_ee [K×3]  每帧"基座系→末端"位姿 [x,y,θ]（FK(编码器反馈)）
%     obs_cam      [K×3]  每帧"目标在相机系"位姿 [x,y,θ]（视觉模块输出）
%     opts.yaw_target     目标在基座系的姿态角（已知时用 yaw 约束解手眼旋转；缺省 0 假设）
%     opts.verbose
%   输出：
%     X_hand = [tx, ty, θ_X]   T_ee_cam（2D 手眼）
%     stats  = .n_used .resid_med .yaw_used(是否用了 yaw 约束) .resid(各帧)
%
%   数学（平面任务本质：手眼旋转 θ_X 不可由 AX=XB 观测——平面旋转可交换）：
%   1) θ_X 用目标姿态约束：yaw_X = yaw_cam + θ_ee + θ_X（目标基座系姿态已知时）
%      否则 θ_X = 0 假设（末端相机同向安装）。
%   2) t_X 用帧对线性方程最小二乘（目标静止，消去未知目标位置 p_base）。
    if nargin < 3 || isempty(opts), opts = struct(); end
    K = min(size(pose_base_ee, 1), size(obs_cam, 1));
    if K < 2, error('handEyeEstimate2D:frames', '至少需要 2 帧观测'); end
    yaw_target = getopt2(opts, 'yaw_target', []);

    % ---- 1. 手眼旋转 θ_X ----
    if ~isempty(yaw_target)
        thX_f = wrapAngle(yaw_target - obs_cam(:, 3) - pose_base_ee(:, 3));  % 各帧
        thX = atan2(mean(sin(thX_f)), mean(cos(thX_f)));
        yaw_used = true;
    else
        thX = 0;  yaw_used = false;   % 假设末端-相机同向
    end
    RX = [cos(thX) -sin(thX); sin(thX) cos(thX)];

    % ---- 2. 平移 t_X：帧对线性方程 ----
    % 目标在末端系: p_ee_i = R_X·p_cam_i + t_X；目标世界系常数:
    % p_base = R_ee_i·p_ee_i + t_ee_i  →  p_ee_i = R_ee_i'·(p_base - t_ee_i)
    % 消 p_base（帧 i vs j）:
    % (I - M_ij)·t_X = M_ij·R_X·p_cam_j - R_X·p_cam_i + R_ee_i'·(t_ee_j - t_ee_i)
    %   M_ij = R_ee_i'·R_ee_j
    A = []; b = [];
    for i = 2:K
        Rei = [cos(pose_base_ee(i,3)) -sin(pose_base_ee(i,3)); sin(pose_base_ee(i,3)) cos(pose_base_ee(i,3))];
        tej = pose_base_ee(1, 1:2)';  tei = pose_base_ee(i, 1:2)';
        Mij = Rei' * [cos(pose_base_ee(1,3)) -sin(pose_base_ee(1,3)); sin(pose_base_ee(1,3)) cos(pose_base_ee(1,3))];
        pc_i = obs_cam(i, 1:2)';  pc_j = obs_cam(1, 1:2)';
        A = [A; eye(2) - Mij]; %#ok<AGROW>
        b = [b; Mij*RX*pc_j - RX*pc_i + Rei'*(tej - tei)]; %#ok<AGROW>
    end
    tX = (A' * A) \ (A' * b);   % 最小二乘

    % ---- 3. 残差（目标世界系常数性）----
    resid = zeros(1, K);
    p_base_est = zeros(2, 1);
    for k = 1:K
        Rek = [cos(pose_base_ee(k,3)) -sin(pose_base_ee(k,3)); sin(pose_base_ee(k,3)) cos(pose_base_ee(k,3))];
        p_ee = RX * obs_cam(k, 1:2)' + tX;
        p_base_k = Rek * p_ee + pose_base_ee(k, 1:2)';
        if k == 1, p_base_est = p_base_k; end
        resid(k) = norm(p_base_k - p_base_est);
    end

    X_hand = [tX(1), tX(2), thX];
    stats = struct('n_used', K-1, 'resid', resid, 'resid_med', median(resid(2:end)), ...
        'yaw_used', yaw_used);
    if getopt2(opts, 'verbose', false)
        fprintf('[handeye] θ_X=%.3f (yaw约束=%d) t=(%.3f,%.3f) 残差中位=%.4f\n', ...
            thX, yaw_used, tX(1), tX(2), stats.resid_med);
    end
end

function v = getopt2(s, f, d)
    v = d;
    if isfield(s, f) && ~isempty(s.(f)), v = s.(f); end
end
