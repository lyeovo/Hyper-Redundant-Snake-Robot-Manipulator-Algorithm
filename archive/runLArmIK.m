%% 主入口：封装迭代求解函数（三维通用版）
function runLArmIK(params, print_step)
    % params: 结构体，所有配置参数
    %   .N          - 关节数
    %   .DH         - N×4 标准DH参数表 [a, alpha, d, theta_offset]
    %   .rod_offset_arr - N×3 局部垂直偏移 [ox, oy, oz]
    %   .X_target   - 3×1 目标位置 [x; y; z]
    %   .q_target   - 1×4 目标姿态四元数 [w, x, y, z]
    %   .q_init     - 1×N 初始关节角
    %   （其余参数同上二维版本）
    % print_step: 每迭代print_step步输出一次参数状态
    clearvars -except params print_step;

    stall_counter   = 0;
    flag_plot_map   = true;

    %% 1. 从结构体读取全局参数
    N               = params.N;
    DH              = params.DH;            % N×4 DH表
    rod_offset_arr  = params.rod_offset_arr;% N×3
    X_target        = params.X_target(:);   % 3×1
    q_target        = params.q_target;      % 1×4 目标四元数
    q_min           = params.q_min;
    q_max           = params.q_max;
    dq_step_max     = params.dq_step_max;
    dq_step_min     = params.dq_step_min;
    kappa           = params.kappa;
    m_arr           = params.m_arr;
    sig0_arr        = params.sig0_arr;
    tau_arr         = params.tau_arr;
    sig_min2        = params.sig_min2;
    lambda_damp     = params.lambda_damp;
    gamma_soft      = params.gamma_soft;
    gamma_ang_base  = params.gamma_ang_base;
    gamma_ang_peak  = params.gamma_ang_peak;
    sigma_weight    = params.sigma_weight;
    dq_stall_thresh = params.dq_stall_thresh;
    stall_count_max = params.stall_count_max;
    obs             = params.obs;           % M×4: [x, y, z, r]
    rho0            = params.rho0;
    safe_margin     = params.safe_margin;
    lambdaM         = params.lambdaM;
    lambda_m        = params.lambda_m;
    max_iter        = params.max_iter;
    q               = params.q_init;

    %% 初始化占位变量（避免第一个收敛检查时未定义）
    dq_norm_round = 0;
    avg_joint_err = 0;

    %% 绘图窗口初始化（3D）
    figure('Color',[0.6,0.7,0.6]);
    set(groot,'DefaultAxesFontName','SimHei');
    ax = gca;
    ax.Color = [0.9,0.9,0.9];
    ax.XColor = [0.1,0.1,0.1];
    ax.YColor = [0.1,0.1,0.1];
    ax.ZColor = [0.1,0.1,0.1];
    ax.GridColor = [0.3 0.3 0.3];
    hold on; axis equal; grid on;
    view(3); rotate3d on;

    % 计算3D绘图边界
    max_reach = max(abs(DH(:,1))) + max(abs(DH(:,3))) + max(abs(rod_offset_arr(:)));
    pad = params.plot_pad;
    x_lim = [-pad, max_reach + pad];
    y_lim = [-max_reach - pad, max_reach + pad];
    z_lim = [-pad, max_reach + pad];
    xlim(x_lim); ylim(y_lim); zlim(z_lim);

    xlabel('X'); ylabel('Y'); zlabel('Z');

    %% 主迭代循环
    for iter = 1:max_iter
        [p_all, p_end, q_curr] = spatialFK(q, DH, rod_offset_arr);
        X_curr = p_end(:);
        err_X = X_target - X_curr;
        dist_end = norm(err_X);

        % 姿态误差（四元数 → 轴角）
        [err_ori, theta_err] = quatPoseError(q_curr, q_target);

        if dist_end < lambda_m && theta_err < lambda_m
            disp("迭代收敛，到达目标！");
            printCurrentStatus3D(iter, q, dist_end, err_X, theta_err, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
            flag_plot_map = false;
            plotCostDepthMap3D(q, params, X_target, q_target);
            break;
        end

        % 自适应步长
        s_base = min(lambdaM, dist_end);
        if dist_end > 0.2
            speed_coeff = 1.8;
        elseif dist_end > 0.05
            speed_coeff = 1.2;
        else
            speed_coeff = 1.0;
        end
        s = s_base * speed_coeff;

        % 位置目标方向
        if dist_end > 1e-12
            u_pos = err_X / dist_end;
        else
            u_pos = [0;0;0];
        end

        % 姿态权重：距离越远，姿态权重越小
        gamma_ang = gamma_ang_base + (gamma_ang_peak - gamma_ang_base) * exp(-dist_end^2 / (2*sigma_weight^2));

        % 障碍物梯度
        [g_all, dg_all] = obsSegGradient3D(q, DH, obs, rho0+safe_margin, p_all, rod_offset_arr);
        if ~isempty(g_all)
            min_g = min(g_all);
            critical_dist = 1.5 * (rho0 + safe_margin);
            if min_g < critical_dist
                scale = min_g / critical_dist;
                s = s * scale;
            end
        end

        % 方差矩阵
        Sigma = zeros(N,N);
        joint_sigma2 = zeros(1,N);
        for i = 1:N
            s2 = errVar(q(i), m_arr(i), tau_arr(i), sig0_arr(i));
            s2 = max(s2, sig_min2);
            Sigma(i,i) = s2;
            joint_sigma2(i) = s2;
        end
        avg_joint_err = mean(joint_sigma2);

        % 几何雅可比 6×N
        J = spatialJac(q, DH, rod_offset_arr);


        % 自适应阻尼：接近目标时降低阻尼以加速精细收敛
        if dist_end < 0.01
            lambda_damp_eff = lambda_damp * 0.1;
        elseif dist_end < 0.05
            lambda_damp_eff = lambda_damp * 0.5;
        else
            lambda_damp_eff = lambda_damp;
        end

        % QP求解增量（三维版）
        dq = solveQP_Damped3D(J, Sigma, s, u_pos, g_all, dg_all, ...
            rho0+safe_margin, dq_step_min, dq_step_max, ...
            q, q_min, q_max, lambda_damp_eff, gamma_soft, ...
            gamma_ang, err_ori);

        % 碰撞校验回退
        q_test = q + dq';
        [p_test_all,~] = spatialFK(q_test, DH, rod_offset_arr);
        [g_check,~] = obsSegGradient3D(q_test, DH, obs, rho0, p_test_all, rod_offset_arr);
        if ~isempty(g_check) && min(g_check) < rho0
            disp("检测碰撞风险，缩小关节增量");
            dq = dq * 0.5;
        end

        % 停滞判断（距离感知：远处停滞真问题，近处微调正常）
        dq_norm = norm(dq);
        % 近目标时放宽停滞阈值
        stall_thresh_eff = dq_stall_thresh;
        if dist_end < 0.01
            stall_thresh_eff = dq_stall_thresh * 0.01;
        elseif dist_end < 0.05
            stall_thresh_eff = dq_stall_thresh * 0.1;
        end
        if dq_norm < stall_thresh_eff
            stall_counter = stall_counter + 1;
            if stall_counter >= stall_count_max
                disp("优化停滞：多轮关节增量极小，提前终止迭代");
                printCurrentStatus3D(iter, q, dist_end, err_X, theta_err, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
                plotCostDepthMap3D(q, params, X_target, q_target);
                flag_plot_map = false;
                break;
            end
        else
            stall_counter = 0;
        end

        % 精细收敛阶段：接近目标时绕过电机舍入，允许任意微调
        in_fine_phase = (dist_end < 10 * lambda_m && theta_err < 10 * lambda_m);
        if in_fine_phase
            dq_round = dq;  % 直接使用QP原始解，不做舍入
            dq_norm_round = norm(dq_round);
        else
            % 电机最小步长舍入（已消除死区）
            dq_round = motorStepRounding(dq, kappa);
            dq_norm_round = norm(dq_round);
            if dq_norm_round < 1e-12
                disp("所有电机增量小于最小精确转动量κ，无有效动作，终止迭代");
                printCurrentStatus3D(iter, q, dist_end, err_X, theta_err, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
                plotCostDepthMap3D(q, params, X_target, q_target);
                flag_plot_map = false;
                break;
            end
        end
        q_new = q + dq_round';

        q = q_new;

        %% 每print_step步输出一次完整参数状态
        if mod(iter, print_step) == 0
            printCurrentStatus3D(iter, q, dist_end, err_X, theta_err, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
        end

        %% 绘图刷新（3D）
        cla;
        plot3(p_all(:,1), p_all(:,2), p_all(:,3), 'y-o', 'LineWidth', 2, 'MarkerSize', 6);
        if ~isempty(obs)
            for o = 1:size(obs,1)
                [x_s, y_s, z_s] = sphere(16);
                surf(obs(o,1) + obs(o,4)*x_s, obs(o,2) + obs(o,4)*y_s, obs(o,3) + obs(o,4)*z_s, ...
                     'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            end
        end
        plot3(X_target(1), X_target(2), X_target(3), 'g*', 'MarkerSize', 14, 'MarkerFaceColor', 'g');

        % 绘制末端姿态坐标系
        R_end = quat2rotm(q_curr);
        axis_len = 0.15;
        plot3([X_curr(1), X_curr(1)+axis_len*R_end(1,1)], ...
              [X_curr(2), X_curr(2)+axis_len*R_end(2,1)], ...
              [X_curr(3), X_curr(3)+axis_len*R_end(3,1)], 'r-', 'LineWidth', 2);
        plot3([X_curr(1), X_curr(1)+axis_len*R_end(1,2)], ...
              [X_curr(2), X_curr(2)+axis_len*R_end(2,2)], ...
              [X_curr(3), X_curr(3)+axis_len*R_end(3,2)], 'g-', 'LineWidth', 2);
        plot3([X_curr(1), X_curr(1)+axis_len*R_end(1,3)], ...
              [X_curr(2), X_curr(2)+axis_len*R_end(2,3)], ...
              [X_curr(3), X_curr(3)+axis_len*R_end(3,3)], 'b-', 'LineWidth', 2);

        param_text = sprintf(...
            '3D臂 N=%d 目标[%.2f,%.2f,%.2f] k=%.5f',...
            N, X_target(1), X_target(2), X_target(3), kappa);
        text(x_lim(1)+0.03, y_lim(2)-0.10, z_lim(2)-0.05, param_text, 'Color',[0,0.6,0],'FontSize',9);

        title_str = sprintf('迭代:%d | 末端位置误差:%.4f | 姿态误差:%.3f rad | 平均方差:%.6f | 停滞:%d/%d',...
            iter, dist_end, theta_err, avg_joint_err, stall_counter, stall_count_max);
        title(title_str);
        drawnow limitrate;
    end

    if flag_plot_map
        disp("迭代达最大轮数未收敛，生成代价深度热力图");
        plotCostDepthMap3D(q, params, X_target, q_target);
    end

    hold off;
    disp("===== 迭代流程全部结束 =====");
end


%% ===================== 辅助函数 =====================

function [p_all, p_end, q_end] = spatialFK(q, DH, rod_offset_arr)
    % 三维正运动学
    % q: 1×N 关节角度
    % DH: N×4 [a, alpha, d, theta_offset]
    % rod_offset_arr: N×3 局部垂直偏移
    % 返回:
    %   p_all: (2N+1)×3 所有节点坐标
    %   p_end: 1×3 末端位置
    %   q_end: 1×4 末端姿态四元数 [w, x, y, z]
    n = length(q);
    p_all = zeros(2*n+1, 3);
    T = eye(4);
    p_all(1,:) = [0, 0, 0];
    idx = 2;

    for i = 1:n
        a_i     = DH(i,1);
        alpha_i = DH(i,2);
        d_i     = DH(i,3);
        theta_i = DH(i,4) + q(i);
        T_i = dhTransform(a_i, alpha_i, d_i, theta_i);
        T = T * T_i;

        % 连杆终点 M
        M = T(1:3,4)';
        p_all(idx,:) = M;
        idx = idx + 1;

        % 垂直偏移（局部坐标 → 世界坐标）
        off_local = rod_offset_arr(i,:)';
        off_world = (T(1:3,1:3) * off_local)';
        P_next = M + off_world;
        p_all(idx,:) = P_next;
        idx = idx + 1;
    end

    % 末端包含最后一根杆的偏移（真实末端位置）
    off_local_end = rod_offset_arr(n,:)';
    off_world_end = (T(1:3,1:3) * off_local_end)';
    p_end = T(1:3,4)' + off_world_end;
    R_end = T(1:3,1:3);
    q_end = rotm2quat(R_end);  % [w, x, y, z]
end


function T = dhTransform(a, alpha, d, theta)
    % 标准 DH 齐次变换矩阵
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end


function p_nodes = spatialFK_SimpleNode(q, DH, rod_offset_arr)
    % 仅返回每个关节连接点的坐标 (n+1)×3
    n = length(q);
    p_nodes = zeros(n+1, 3);
    T = eye(4);
    p_nodes(1,:) = [0, 0, 0];

    for i = 1:n
        a_i     = DH(i,1);
        alpha_i = DH(i,2);
        d_i     = DH(i,3);
        theta_i = DH(i,4) + q(i);
        T_i = dhTransform(a_i, alpha_i, d_i, theta_i);
        T = T * T_i;

        off_local = rod_offset_arr(i,:)';
        off_world = (T(1:3,1:3) * off_local)';
        P_curr = T(1:3,4)' + off_world;
        p_nodes(i+1,:) = P_curr;
    end
end


function J = spatialJac(q, DH, rod_offset_arr)
    % 几何雅可比 6×N：[Jv; Jw]
    % Jv: 3×N 线速度雅可比
    % Jw: 3×N 角速度雅可比
    % 标准DH：关节i绕z_{i-1}旋转，参考点为o_{i-1}
    n = length(q);
    J = zeros(6, n);

    % 存储z_0~z_n 和 o_0~o_n（索引1对应frame 0）
    z_all = zeros(3, n+1);
    o_all = zeros(3, n+1);
    z_all(:,1) = [0; 0; 1];   % z_0 = 基座z轴
    o_all(:,1) = [0; 0; 0];   % o_0 = 基座原点
    T = eye(4);

    for i = 1:n
        a_i = DH(i,1); alpha_i = DH(i,2);
        d_i = DH(i,3); theta_i = DH(i,4) + q(i);
        T_i = dhTransform(a_i, alpha_i, d_i, theta_i);
        T = T * T_i;
        z_all(:,i+1) = T(1:3,3);
        o_all(:,i+1) = T(1:3,4);
    end

    % 末端位置（含最后一根杆偏移）
    off_local_end = rod_offset_arr(n,:)';
    off_world_end = (T(1:3,1:3) * off_local_end)';
    p_end = T(1:3,4)' + off_world_end;
    p_end = p_end';

    for i = 1:n
        z_i = z_all(:,i);    % z_{i-1}
        o_i = o_all(:,i);    % o_{i-1}
        % 线速度：z_{i-1} × (p_end - o_{i-1})
        J(1:3,i) = cross(z_i, p_end - o_i);
        % 角速度：z_{i-1}
        J(4:6,i) = z_i;
    end
end


function Jp = spatialJacPoint(q, DH, idx, rod_offset_arr)
    % 某一点 idx 的几何雅可比（6×N）
    % 标准DH：关节i绕z_{i-1}旋转，参考点为o_{i-1}
    n = length(q);
    Jp = zeros(6, n);
    [p_all, ~] = spatialFK(q, DH, rod_offset_arr);
    p_nodes = spatialFK_SimpleNode(q, DH, rod_offset_arr);

    if idx <= n+1
        pt = p_nodes(idx,:)';
    else
        pt = p_all(idx,:)';
    end

    % 存储z_0~z_n 和 o_0~o_n（索引1对应frame 0）
    z_all = zeros(3, n+1);
    o_all = zeros(3, n+1);
    z_all(:,1) = [0; 0; 1];   % z_0
    o_all(:,1) = [0; 0; 0];   % o_0
    T = eye(4);

    for i = 1:n
        a_i = DH(i,1); alpha_i = DH(i,2);
        d_i = DH(i,3); theta_i = DH(i,4) + q(i);
        T_i = dhTransform(a_i, alpha_i, d_i, theta_i);
        T = T * T_i;
        z_all(:,i+1) = T(1:3,3);
        o_all(:,i+1) = T(1:3,4);
    end

    for i = 1:min(idx, n)
        z_i = z_all(:,i);    % z_{i-1}
        o_i = o_all(:,i);    % o_{i-1}
        Jp(1:3,i) = cross(z_i, pt - o_i);
        Jp(4:6,i) = z_i;
    end
end


function [err_ori, theta_err] = quatPoseError(q_curr, q_target)
    % 四元数姿态误差 → 轴角表示
    % q_curr, q_target: 1×4 [w, x, y, z]
    % 返回:
    %   err_ori: 3×1 轴角误差向量
    %   theta_err: 标量旋转角度误差
    q_err = quatmultiply(q_target, quatconj(q_curr));
    % 确保实部非负（处理双覆盖）
    if q_err(1) < 0
        q_err = -q_err;
    end
    w = q_err(1);
    % 从四元数提取轴角
    theta_err = 2 * acos(max(-1, min(1, w)));
    if theta_err < 1e-12
        err_ori = [0; 0; 0];
    else
        sin_half = sin(theta_err / 2);
        if abs(sin_half) < 1e-12
            err_ori = [0; 0; 0];
        else
            k = q_err(2:4)' / sin_half;
            err_ori = theta_err * k;  % 3×1
        end
    end
end


function sigma2 = errVar(x, m, tau, sig0)
    sigma2 = sig0^2 * exp( -(x - m).^2 / (2*tau^2) );
end


function [g_total, dg_total] = obsSegGradient3D(q, DH, obs, rho0, p_all, rod_offset_arr)
    % 3D球体障碍物梯度
    g_total = [];
    dg_total = [];
    if isempty(obs)
        return;
    end
    n_seg = size(p_all,1) - 1;
    n_obs = size(obs,1);
    filter_dist = 2 * rho0;
    for o = 1:n_obs
        xo = obs(o,1); yo = obs(o,2); zo = obs(o,3);
        for seg = 1:n_seg
            p0 = p_all(seg,:);
            p1 = p_all(seg+1,:);
            [dist, grad_dist] = segSphereDistGrad(p0, p1, xo, yo, zo, q, DH, seg, rod_offset_arr);
            if dist > filter_dist
                continue;
            end
            g_total = [g_total; dist];
            dg_total = [dg_total; grad_dist];
        end
    end
end


function [dist, dg] = segSphereDistGrad(p0, p1, xo, yo, zo, q, DH, seg_idx, rod_offset_arr)
    % 线段到球体的最近距离及梯度
    dx_seg = p1 - p0;
    seg_len_sq = dx_seg * dx_seg';
    if seg_len_sq < 1e-16
        p_near = p0;
        t_val = 0;
    else
        t_val = clamp(dot([xo-p0(1), yo-p0(2), zo-p0(3)], dx_seg) / seg_len_sq, 0, 1);
        p_near = p0 + t_val * dx_seg;
    end
    dx_near = p_near - [xo, yo, zo];
    dist = norm(dx_near);
    n = length(q);
    dg = zeros(1, n);

    if dist < 1e-12
        return;
    end

    grad_dir = dx_near / dist;
    J0_full = spatialJacPoint(q, DH, seg_idx, rod_offset_arr);
    J1_full = spatialJacPoint(q, DH, seg_idx+1, rod_offset_arr);
    grad_pnear = (1-t_val) * grad_dir * J0_full(1:3,:) + t_val * grad_dir * J1_full(1:3,:);
    dg = grad_pnear;
end


function val = clamp(x, low, high)
    val = min(max(x, low), high);
end


function dq = solveQP_Damped3D(J, Sigma, s, u_pos, g_all, dg_all, rho0, dq_min, dq_max, ...
    q, q_min, q_max, lambda_damp, gamma_soft, gamma_ang, err_ori)
    
    n = size(J, 2);
    Jv = J(1:3,:);  % 线速度部分 3×N
    Jw = J(4:6,:);  % 角速度部分 3×N

    invSigma = inv(Sigma);

    H = 2 * invSigma ...
        + 2 * lambda_damp^2 * eye(n) ...
        + 2 * gamma_soft * (Jv' * Jv) ...
        + 2 * gamma_ang  * (Jw' * Jw);

    f = -2 * gamma_soft * s * Jv' * u_pos ...
        - 2 * gamma_ang  * Jw' * err_ori;

    % 障碍物不等式约束
    if isempty(dg_all)
        Aineq = [];
        bineq = [];
    else
        A_obs = -dg_all;
        b_obs = -(rho0 - g_all);
        Aineq = A_obs;
        bineq = b_obs;
    end

    % 关节限位
    A_qmin = -eye(n);
    b_qmin = -(q_min' - q');
    A_qmax = eye(n);
    b_qmax = q_max' - q';
    Aineq = [Aineq; A_qmin; A_qmax];
    bineq = [bineq; b_qmin; b_qmax];

    lb = dq_min * ones(n,1);
    ub = dq_max * ones(n,1);

    opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    dq = quadprog(H, f, Aineq, bineq, [], [], lb, ub, [], opts);
    if isempty(dq)
        disp("QP无可行解，放大阻尼重试");
        H = 2 * invSigma ...
            + 2 * (lambda_damp*2)^2 * eye(n) ...
            + 2 * gamma_soft * (Jv' * Jv) ...
            + 2 * gamma_ang  * (Jw' * Jw);
        dq = quadprog(H, f, Aineq, bineq, [], [], lb, ub, [], opts);
        if isempty(dq)
            dq = zeros(n, 1);
        end
    end
end


function dq_round = motorStepRounding(dq, kappa)
    n = length(dq);
    dq_round = zeros(n, 1);
    for i = 1:n
        val = dq(i);
        abs_val = abs(val);
        if abs_val < kappa
            dq_round(i) = sign(val) * kappa;  % 消除死区：至少执行最小步长
        elseif abs_val < 2*kappa
            dq_round(i) = sign(val) * kappa;
        else
            dq_round(i) = val;
        end
    end
end


function printCurrentStatus3D(iter, q, dist_end, err_X, theta_err, avg_joint_err, dq_norm_round, stall_counter, stall_count_max)
    fprintf("\n==================== 迭代 %d 状态输出 ====================\n", iter);
    fprintf("当前关节角度 q = [");
    fprintf("%.4f ", q);
    fprintf("]\n");
    fprintf("末端当前坐标误差 ΔX = [%.4f, %.4f, %.4f]，距离范数 = %.4f\n", err_X(1), err_X(2), err_X(3), dist_end);
    fprintf("末端姿态误差（轴角幅值） = %.3f rad\n", theta_err);
    fprintf("关节平均方差 avg_joint_err = %.6f\n", avg_joint_err);
    fprintf("本轮舍入后关节增量范数 ||dq_round|| = %.6f\n", dq_norm_round);
    fprintf("停滞计数 %d / %d\n", stall_counter, stall_count_max);
    fprintf("==========================================================\n");
end


function V = calcCostValue3D(q, params, X_target, q_target)
    N = params.N;
    DH = params.DH;
    rod_offset_arr = params.rod_offset_arr;

    % 1. 位置误差代价
    [~, p_end, q_curr] = spatialFK(q, DH, rod_offset_arr);
    errX = X_target - p_end(:);
    cost_pos = norm(errX)^2;

    % 2. 姿态误差代价
    [~, theta_err] = quatPoseError(q_curr, q_target);
    cost_ang = params.gamma_ang_base * theta_err^2;

    % 3. 关节方差正则项
    cost_sigma = 0;
    for i = 1:N
        s2 = errVar(q(i), params.m_arr(i), params.tau_arr(i), params.sig0_arr(i));
        s2 = max(s2, params.sig_min2);
        cost_sigma = cost_sigma + s2;
    end

    % 4. 碰撞惩罚代价
    cost_obs = 0;
    obs = params.obs;
    rho0 = params.rho0;
    [p_all, ~] = spatialFK(q, DH, rod_offset_arr);
    if ~isempty(obs)
        for o = 1:size(obs,1)
            xo = obs(o,1); yo = obs(o,2); zo = obs(o,3);
            for seg = 1:size(p_all,1)-1
                p0 = p_all(seg,:); p1 = p_all(seg+1,:);
                dx_seg = p1 - p0;
                seg_len_sq = dx_seg * dx_seg';
                t_val = 0;
                if seg_len_sq > 1e-16
                    t_val = clamp(dot([xo-p0(1), yo-p0(2), zo-p0(3)], dx_seg)/seg_len_sq, 0, 1);
                end
                p_near = p0 + t_val * dx_seg;
                dist = norm(p_near - [xo, yo, zo]);
                if dist < rho0
                    cost_obs = cost_obs + 1e6 * (rho0 - dist)^2;
                end
            end
        end
    end

    V = cost_pos + cost_ang + params.sigma_weight * cost_sigma + cost_obs;
end


function plotCostDepthMap3D(q_opt, params, X_target, q_target)
    % 在 q1-q2 平面扰动，绘制 3D 代价热力图
    DH = params.DH;
    rod_offset_arr = params.rod_offset_arr;

    perturb_range = 0.3;
    sample_num = 30;

    q1_list = linspace(q_opt(1)-perturb_range, q_opt(1)+perturb_range, sample_num);
    q2_list = linspace(q_opt(2)-perturb_range, q_opt(2)+perturb_range, sample_num);

    X_grid = zeros(sample_num, sample_num);
    Y_grid = zeros(sample_num, sample_num);
    Z_grid = zeros(sample_num, sample_num);
    V_grid = zeros(sample_num, sample_num);

    for i = 1:sample_num
        for j = 1:sample_num
            q_sample = q_opt;
            q_sample(1) = q1_list(i);
            q_sample(2) = q2_list(j);
            [~, p_end, ~] = spatialFK(q_sample, DH, rod_offset_arr);
            X_grid(i,j) = p_end(1);
            Y_grid(i,j) = p_end(2);
            Z_grid(i,j) = p_end(3);
            V_grid(i,j) = calcCostValue3D(q_sample, params, X_target, q_target);
        end
    end

    figure('Name', '末端3D代价深度图｜局部最优检测', 'Color', 'w');
    surf(X_grid, Y_grid, Z_grid, V_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    hold on; grid on;
    colormap(jet);
    cb = colorbar;
    cb.Label.String = '总代价 V(q) (越小越优)';
    cb.Label.FontSize = 10;

    plot3(X_target(1), X_target(2), X_target(3), 'g*', 'MarkerSize', 16, 'MarkerFaceColor', 'g', 'DisplayName', '目标位姿');
    [~, p_opt_end, ~] = spatialFK(q_opt, DH, rod_offset_arr);
    plot3(p_opt_end(1), p_opt_end(2), p_opt_end(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '当前收敛解');

    % 绘制障碍物
    obs = params.obs;
    if ~isempty(obs)
        for o = 1:size(obs,1)
            [x_s, y_s, z_s] = sphere(16);
            surf(obs(o,1)+obs(o,4)*x_s, obs(o,2)+obs(o,4)*y_s, obs(o,3)+obs(o,4)*z_s, ...
                 'FaceColor', 'k', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        end
    end

    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('代价深度图（3D）｜关节扰动±%.2f rad | 红圈=当前解 绿星=目标', perturb_range));
    legend('Location', 'best');
    view(3);
    drawnow;
end


%% 单点IK求解子函数（三维版）
function q_out = solveSinglePointIK3D(q_init, X_goal, q_goal, params)
    N = params.N;
    DH = params.DH;
    rod_offset_arr = params.rod_offset_arr;

    q_min = params.q_min;
    q_max = params.q_max;
    dq_step_max = params.dq_step_max;
    dq_step_min = params.dq_step_min;
    kappa = params.kappa;
    m_arr = params.m_arr;
    sig0_arr = params.sig0_arr;
    tau_arr = params.tau_arr;
    sig_min2 = params.sig_min2;
    lambda_damp = params.lambda_damp;
    gamma_soft = params.gamma_soft;
    gamma_ang_base = params.gamma_ang_base;
    gamma_ang_peak = params.gamma_ang_peak;
    sigma_weight = params.sigma_weight;
    rho0 = params.rho0;
    safe_margin = params.safe_margin;
    lambdaM = params.lambdaM;
    lambda_m = params.lambda_m;
    max_iter_single = 60;

    q = q_init;
    for iter = 1:max_iter_single
        [p_all, p_end, q_curr] = spatialFK(q, DH, rod_offset_arr);
        err_X = X_goal(:) - p_end(:);
        dist_end = norm(err_X);
        [err_ori, theta_err] = quatPoseError(q_curr, q_goal);

        if dist_end < lambda_m && theta_err < lambda_m
            break;
        end

        gamma_ang = gamma_ang_base + (gamma_ang_peak - gamma_ang_base) * exp(-dist_end^2 / (2*sigma_weight^2));
        s_base = min(lambdaM, dist_end);
        if dist_end > 0.2
            speed_coeff = 1.8;
        elseif dist_end > 0.05
            speed_coeff = 1.2;
        else
            speed_coeff = 1.0;
        end
        s = s_base * speed_coeff;

        if dist_end > 1e-12
            u_pos = err_X / dist_end;
        else
            u_pos = [0;0;0];
        end

        [g_all, dg_all] = obsSegGradient3D(q, DH, params.obs, rho0+safe_margin, p_all, rod_offset_arr);
        if ~isempty(g_all)
            min_g = min(g_all);
            critical_dist = 1.5 * (rho0 + safe_margin);
            if min_g < critical_dist
                scale = min_g / critical_dist;
                s = s * scale;
            end
        end

        Sigma = zeros(N,N);
        for i = 1:N
            s2 = errVar(q(i), m_arr(i), tau_arr(i), sig0_arr(i));
            s2 = max(s2, sig_min2);
            Sigma(i,i) = s2;
        end

        J = spatialJac(q, DH, rod_offset_arr);
        dq = solveQP_Damped3D(J, Sigma, s, u_pos, g_all, dg_all, ...
            rho0+safe_margin, dq_step_min, dq_step_max, ...
            q, q_min, q_max, lambda_damp, gamma_soft, ...
            gamma_ang, err_ori);

        q_test = q + dq';
        [p_test_all,~] = spatialFK(q_test, DH, rod_offset_arr);
        [g_check,~] = obsSegGradient3D(q_test, DH, params.obs, rho0, p_test_all, rod_offset_arr);
        if ~isempty(g_check) && min(g_check) < rho0
            dq = dq * 0.5;
        end

        dq_round = motorStepRounding(dq, kappa);
        q = q + dq_round';
    end
    q_out = q;
end