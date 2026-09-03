function top = runTopLevel(opts)
%runTopLevel 顶层实时闭环估计（S1 骨架 + mock 电控/视觉反馈）
%   循环：读电控关节角反馈 → FK → 相机观测 → 手眼在线估计 → 目标 Kalman → 任务触发
%   opts:
%     .model           createArmModel 输出（默认 6×1.04393m）
%     .target_true     [x,y,yaw] 目标在基座系真值（mock 场景设定）
%     .T_ee_cam_true   [tx,ty,th] 手眼真值（mock 生成观测用）
%     .n_frames        巡视帧数（姿态遍历产生手眼可观测性）
%     .motor_noise     电机编码器噪声 σ rad
%     .vis_noise       视觉噪声 σ（位置 m / 角度 rad）
%     .plan_method     任务规划方法（默认 'auto'）
%     .verbose         (false)
%   返回 top：
%     .handeye_hist    [K×3] 手眼估计轨迹（每帧）
%     .target_hist     [K×3] 目标基座系估计轨迹（Kalman 后）
%     .err_handeye     [K×1] 手眼估计误差（vs 真值，mock 诊断）
%     .err_target      [K×1] 目标估计误差（vs 真值）
%     .converged_frame 手眼收敛帧号（误差 < 阈值后首帧）
%     .task           任务规划结果（simulateMotion info）
%     .log            过程日志 cell
    if nargin < 1 || isempty(opts), opts = struct(); end
    target_true  = optget(opts, 'target_true',   [2.5, 1.0, 0.3]);
    Xh_true      = optget(opts, 'T_ee_cam_true', [0.05, 0.0, 0.0]);
    n_frames     = optget(opts, 'n_frames', 24);
    motor_noise  = optget(opts, 'motor_noise', 0.01);
    vis_noise    = optget(opts, 'vis_noise', [0.005, 0.01]);
    plan_method  = optget(opts, 'plan_method', 'auto');
    verbose      = optget(opts, 'verbose', false);
    if ~isfield(opts, 'model') || isempty(opts.model)
        model = createArmModel(struct('N', 6, 'L_seg', 1.04393));
    else
        model = opts.model;
    end

    % ---- mock 巡视序列：姿态各异的位形（产生手眼可观测性）----
    patrol = linspace(0, 1, n_frames)';
    th_seq = [0.2*sin(2*pi*patrol*1.5) + 0.5,  0.3*cos(2*pi*patrol*1.0) + 0.2, ...
              0.25*sin(2*pi*patrol*0.8),       0.15*cos(2*pi*patrol*1.2), ...
              0.1*sin(2*pi*patrol*1.8),        0.05*cos(2*pi*patrol*0.5)];

    % ---- 状态 ----
    kf = kalmanTarget2D();
    win = min(20, n_frames);          % 手眼滑动窗口
    handeye_hist = nan(n_frames, 3);
    target_hist  = nan(n_frames, 3);
    err_handeye  = nan(n_frames, 1);
    err_target   = nan(n_frames, 1);
    converged_frame = [];
    logc = {};
    Xh_est = [0, 0, 0];               % 手眼初始估计（0 = 无先验）

    % ---- 主循环 ----
    for k = 1:n_frames
        % 1. 读电控反馈（mock：巡视位形 + 编码器噪声）
        th_real = th_seq(k, :) + motor_noise * randn(1, 6);
        % 2. 正运动学 → 末端位姿
        [~, pe] = planarFK_L(th_real, model.DH, model.cfg.rod_offset_arr);
        th_ee = getEndEffectorAngle_L(th_real, model.DH, model.cfg.rod_offset_arr);
        pose_ee = [pe(1), pe(2), th_ee];
        % 3. 相机观测（mock：目标真值经 真手眼+末端 反算到相机系 + 视觉噪声）
        T_ee = se2m(pose_ee);
        T_cam_true = T_ee * se2m(Xh_true);
        x_cam = (inv_se2m(T_cam_true) * [target_true(1:2)'; 1]);
        x_cam = x_cam(1:2)' + vis_noise(1) * randn(1, 2);
        yaw_cam = wrapAngle(target_true(3) - pose_ee(3) - Xh_true(3)) + vis_noise(2) * randn;
        pose_hist(k, :) = pose_ee;    %#ok<AGROW>
        obs_hist(k, :)  = [x_cam, yaw_cam];   %#ok<AGROW>  [x,y,θ] 3 列
        % 4. 手眼在线估计（滑动窗口）
        if k >= 2
            i0 = max(1, k - win + 1);
            [Xh_est, st] = handEyeEstimate2D(pose_hist(i0:k, :), obs_hist(i0:k, :), ...
                struct('yaw_target', target_true(3)));
            handeye_hist(k, :) = Xh_est;
            err_handeye(k) = norm(Xh_est(1:2) - Xh_true(1:2)) + ...
                0.5 * abs(wrapAngle(Xh_est(3) - Xh_true(3)));
            if isempty(converged_frame) && err_handeye(k) < 0.05 && st.resid_med < 0.02
                converged_frame = k;
                logc{end+1} = sprintf('[top] 手眼收敛于帧 %d: θ=%.3f t=(%.3f,%.3f) 误差=%.4f', ...
                    k, Xh_est(3), Xh_est(1), Xh_est(2), err_handeye(k));
            end
        else
            handeye_hist(k, :) = Xh_est;
            err_handeye(k) = nan;
        end
        % 5. 目标基座系估计：X_base = T_base_cam · x_cam（手眼估计 + 末端位姿）
        T_cam_est = T_ee * se2m(Xh_est);
        Xb = T_cam_est * [x_cam(1), x_cam(2), 1]';
        target_est = [Xb(1), Xb(2), wrapAngle(yaw_cam + pose_ee(3) + Xh_est(3))];
        kf = kalmanTargetUpdate(kf, target_est);
        target_hist(k, :) = kf.x(1:3)';
        err_target(k) = norm(kf.x(1:2)' - target_true(1:2)) + ...
            0.5 * abs(wrapAngle(kf.x(3) - target_true(3)));
        if verbose
            fprintf('[top] 帧%d 目标误差=%.4f 手眼误差=%.4f\n', k, err_target(k), err_handeye(k));
        end
    end

    % ---- 6. 任务触发：用最终 Kalman 目标位姿规划任务 ----
    final_tgt = kf.x(1:3)';
    q0 = zeros(1, 6);
    task = simulateMotion(model, plan_method, q0, final_tgt);
    logc{end+1} = sprintf('[top] 任务触发: 目标估计=(%.3f,%.3f,%.3f) 方法=%s 成功=%d 末端误差=%.4f', ...
        final_tgt, task.method_used, task.success, task.dist_end);

    top = struct('handeye_hist', handeye_hist, 'target_hist', target_hist, ...
        'err_handeye', err_handeye, 'err_target', err_target, ...
        'converged_frame', converged_frame, 'task', task, ...
        'target_true', target_true, 'Xh_true', Xh_true, 'log', {logc});
end

%% ---- 工具 ----
function T = se2m(p)
    c = cos(p(3));  s = sin(p(3));
    T = [c -s p(1); s c p(2); 0 0 1];
end
function Ti = inv_se2m(T)
    R = T(1:2,1:2);  t = T(1:2,3);
    Ti = [R' -R'*t; 0 0 1];
end