%% 主入口：封装迭代求解函数（二维平面版）
%  ======================================================================
%  【已废弃】本文件为旧完整版求解器，新核心见 ArmSimulator2D/
%  （createArmModel + simulateMotion，5 种方法 + auto 调度链）。
%  保留仅供 PlanarDrawApp/TopLevelSystem 迁移过渡，新代码勿用。
%  完整设计见 文档/实现方案.md。
%  ======================================================================
%  简化接口：
%    q = solveIK([x, y])                    % 仅目标位置
%    q = solveIK([x, y], theta)             % 含目标角度
%    q = solveIK([x, y], theta, mode)       % mode: 1=RRT 2=RRT* 3=PRM*
%    q = solveIK([x, y], theta, mode, N)    % 自定义输出间隔
%  完整接口：
%    q = runLArmIK_2D(params, print_step)
function q_final = runLArmIK_2D(params, print_step)
    % params: 结构体，所有配置参数
    % print_step: 每迭代print_step步输出一次参数状态
    % q_final: (可选输出) 最终关节角 [1×N]
    % --- 全局搜索：1=多启动RRT, 2=RRT*, 3=PRM* ---
    if isfield(params, 'use_global_search') && params.use_global_search > 0
        % --- mode 4: FMM+DAG ---
        if params.use_global_search == 4
            fprintf('========== FMM+DAG ==========\n');
            [q_fmmdag, ok_fmm] = runLArmIK_2D_FMMDAG(params, print_step);
            if ok_fmm, q_final = q_fmmdag; return;
            else, fprintf('[FMM+DAG] fallback QP\n'); end
        end

        if params.use_global_search >= 3
            fprintf('========== PRM* 路线图搜索模式 ==========\n');
            [q_prm, ok, ~, path_prm] = runPRMStar(params);
            if ok
                q_refined = refinePath(path_prm, params);
                params.q_init = q_refined(end, :);
                params.prm_path = path_prm;
                params.skip_global = true;
            else
                fprintf('[PRM*] 查询失败，回退 RRT\n');
                params.use_global_search = 1;
            end
        end
        if ~(isfield(params,'skip_global') && params.skip_global)
            n_trees = max(params.rrt_num_trees, 1);
            best_q = params.q_init;
            best_cost = Inf;
            success = false;
            if params.use_global_search == 1
                fprintf('========== 多启动 RRT (%d 树) ==========\n', n_trees);
            else
                fprintf('========== RRT* 渐进最优模式 ==========\n');
            end
            for t = 1:n_trees
                if t > 1, params.q_init = randomQInit(params); end
                [q_rrt, ~, ok] = runLArmIK_2D_RRT(params);
                if ok
                    c = calcCostValue(q_rrt, params, params.X_target, params.theta_end_target);
                    if c < best_cost
                        best_cost = c; best_q = q_rrt; success = true;
                    end
                end
            end
            if success
                params.q_init = best_q;
                fprintf('[全局搜索] 最佳 RRT 解 cost=%.4f\n', best_cost);
            else
                params.q_init = best_q;
                fprintf('[全局搜索] 未达目标，取最近节点\n');
            end
            if params.use_global_search >= 2 && success
                fprintf('[RRT*] 椭圆采样精修 (%d iter)...\n', params.rrt_star_max_iter);
                q_star = runLArmIK_2D_RRTStar(params, best_q, true);
                params.q_init = q_star;
            end
        end
    end

    q_final = [];
    clearvars -except params print_step q_final;
    
    %% 1. 从结构体读取全局参数
    N               = params.N;
    L_seg           = params.L_seg;
    theta_end_target= params.theta_end_target;
    X_target        = params.X_target;
    rod_offset_arr  = params.rod_offset_arr;
    
    DH = zeros(N,4);
    DH(:,3) = L_seg;
    DH(:,2) = 0;
    DH(:,4) = 0;
    
    max_reach = N * L_seg;
    plot_pad = params.plot_pad;
    bottom_pad = params.bottom_pad;
    offset_max = max(rod_offset_arr);
    offset_min = min(rod_offset_arr);
    y_max_extra = N * abs(offset_max);
    y_min_extra = N * abs(offset_min);
    x_min = -0.02;
    x_max = max_reach + plot_pad;
    y_min = -bottom_pad - y_min_extra;
    y_max = max_reach + plot_pad + y_max_extra;

    q_min           = params.q_min;
    q_max           = params.q_max;
    dq_step_max     = params.dq_step_max;
    dq_step_min     = params.dq_step_min;
    kappa           = params.kappa;

    m_arr           = params.m_arr;
    sig0_arr        = params.sig0_arr;
    tau_arr         = params.tau_arr;
    mu_e_arr        = params.mu_e_arr;
    sig_min2        = params.sig_min2;

    lambda_damp     = params.lambda_damp;
    gamma_soft      = params.gamma_soft;

    gamma_ang_base  = params.gamma_ang_base;
    gamma_ang_peak  = params.gamma_ang_peak;
    sigma_weight    = params.sigma_weight;

    dq_stall_thresh = params.dq_stall_thresh;
    stall_count_max = params.stall_count_max;
    stall_counter   = 0;

    obs             = params.obs;
    rho0            = params.rho0;
    safe_margin     = params.safe_margin;
    obs_lines       = [];
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)
        obs_lines = params.obs_lines;
    end

    % 高斯软约束参数
    obs_sigma = 0.15;
    if isfield(params, 'obs_sigma') && ~isempty(params.obs_sigma)
        obs_sigma = params.obs_sigma;
    end
    gamma_obs = 500;
    if isfield(params, 'gamma_obs') && ~isempty(params.gamma_obs)
        gamma_obs = params.gamma_obs;
    end


    % 动量动力学参数
    use_momentum = true;
    if isfield(params, 'use_momentum'), use_momentum = params.use_momentum; end
    momentum_beta = 0.9;
    if isfield(params, 'momentum_beta'), momentum_beta = params.momentum_beta; end
    barrier_C = 200;
    if isfield(params, 'barrier_C'), barrier_C = params.barrier_C; end
    barrier_eps = 0.001;
    if isfield(params, 'barrier_eps'), barrier_eps = params.barrier_eps; end
    dt_base = 0.08;
    if isfield(params, 'dt_base'), dt_base = params.dt_base; end
    rho_critical = 0.15;
    if isfield(params, 'rho_critical'), rho_critical = params.rho_critical; end

    lambdaM         = params.lambdaM;
    lambda_m        = params.lambda_m;
    max_iter        = params.max_iter;
    q               = params.q_init;

    lambda_part      = params.lambda_part;
    w_part           = params.w_part;
    lambda_activate  = params.lambda_activate;
    lambda_motor     = params.lambda_motor;
    if isempty(w_part)
        w_part = ones(1, N);
    end

    joint_activated = false(1, N);

    dq_norm_round = 0;
    avg_joint_err = 0;
    theta_curr = 0;
    err_ang = 0;

    %% 预创建图形对象（独立窗口，避免 PlanarDrawApp 定时器 cla 冲突）
    fig_ik = figure('Name','L型臂2D IK','Color','w','HandleVisibility','off');
    ax = axes('Parent',fig_ik);
    hold(ax, 'on');
    xlim(ax, [x_min, x_max]); ylim(ax, [y_min, y_max]);
    axis(ax, 'equal'); grid(ax, 'on');

    % 静态障碍物（圆 + 线段）
    circle_theta = linspace(0, 2*pi, 40);
    h_obs_circles = gobjects(0);
    if ~isempty(obs)
        for o = 1:size(obs,1)
            xc = obs(o,1) + obs(o,3)*cos(circle_theta);
            yc = obs(o,2) + obs(o,3)*sin(circle_theta);
            h_obs_circles(o) = plot(ax, xc, yc, 'r-', 'LineWidth', 1.5);
        end
    end
    h_obs_line_segs = gobjects(0);
    if ~isempty(obs_lines)
        for li = 1:length(obs_lines)
            ln = obs_lines{li};
            h_obs_line_segs(li) = plot(ax, ln(:,1), ln(:,2), 'm-', 'LineWidth', 2.5);
        end
    end
    h_target = plot(ax, X_target(1), X_target(2), 'g*', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
    h_arm = plot(ax, NaN, NaN, 'y-o', 'LineWidth', 2, 'MarkerSize', 6);
    h_txt_total = text(ax, 0, 0, '', 'Color', 'r', 'FontSize', 8, 'Visible', 'off');
    h_txt_angle = text(ax, 0, 0, '', 'Color', 'm', 'FontSize', 8, 'Visible', 'off');
    h_txt_param = text(ax, x_min+0.03, y_min+0.015, '', 'Color', [0,0.6,0], 'FontSize', 9, ...
        'HorizontalAlignment', 'left', 'Visible', 'off');
    h_title = title(ax, '');
    % 保存 Cleanup 对象用于迭代结束后关闭窗口
    cleanup_fig = onCleanup(@() closeIfValid(fig_ik));

    % 碰撞检测开关
    has_obs = ~isempty(obs) || ~isempty(obs_lines);

    %% 主迭代循环
    % —— 预计算初始动量（初速度），加速首轮迭代 ——
    [p_all_init, p_end_init] = planarFK_L(q, DH, rod_offset_arr);
    err_X_init = X_target - p_end_init;
    dist_init = max(norm(err_X_init), 1e-8);
    u_init = err_X_init / dist_init;  u_init = u_init(:);
    J_init = planarJac_L(q, DH, rod_offset_arr);
    [g_init, dg_init] = obsSegGradient(q, DH, obs, rho0+safe_margin, p_all_init, rod_offset_arr, obs_lines);
    f_bar_init = zeros(N,1);
    if has_obs && ~isempty(g_init)
        g_eff_init = max(g_init - rho0, barrier_eps);
        dbar_dg_init = -min(1 ./ g_eff_init, barrier_C);
        f_bar_init = sum(dbar_dg_init .* dg_init, 1)';
    end
    G_momentum = gamma_soft * J_init' * u_init - f_bar_init;
    if norm(G_momentum) < 1e-6, G_momentum = gamma_soft * J_init' * u_init; end

    for iter = 1:max_iter
        [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr);
        X_curr = p_end;
        err_X = X_target - X_curr;
        dist_end = norm(err_X);
        if dist_end < lambda_m
            disp("迭代收敛，到达目标！");
            printCurrentStatus(iter, q, dist_end, err_X, theta_curr, err_ang, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
            % 绘制代价深度热力图，判断局部最优
            plotCostDepthMap(q, params, X_target, theta_end_target);
            break;
        end
%{
        if dist_end < lambda_m
            fprintf('弱收敛: dist=%.5f mm, 梯度=%.4f\n', dist_end*1000, norm(grad_total));
            break;
        end

%}

        theta_curr = getEndEffectorAngle_L(q, DH, rod_offset_arr);
        err_ang = theta_end_target - theta_curr;
        gamma_ang = gamma_ang_base + (gamma_ang_peak - gamma_ang_base) * exp( - dist_end^2 / (2 * sigma_weight^2) );
        J_ang = jacEndAngle_L(q, DH, rod_offset_arr);

        % === 物理动力学：自适应时间步长 Δt = Δt₀ / λ(g) ===
        [g_all, dg_all] = obsSegGradient(q, DH, obs, rho0+safe_margin, p_all, rod_offset_arr, obs_lines);
%{
        % 障碍邻近度 λ(g) — 自适应时间缩放
        end
        if false  % -- dead: old adaptive step + obstacle code --
        elseif dist_end > 0.05
            speed_coeff = 1.2;
        else
            speed_coeff = 1.0;
        end
        s = s_base * speed_coeff;
        u = err_X / dist_end;
        u = u(:);

        [g_all, dg_all] = obsSegGradient(q, DH, obs, rho0+safe_margin, p_all, rod_offset_arr, obs_lines);
        if ~isempty(g_all)
            min_g = min(g_all);
            phi = erfc(min_g / (sqrt(2) * obs_sigma));
            scale = max(0.05, 1 - phi);
            if true
            s = s * scale;
            end
        end
        end  % -- end dead block --
        elseif dist_end > 0.05
            speed_coeff = 1.2;
        else
            speed_coeff = 1.0;
        end
        s = s_base * speed_coeff;
        u = err_X / dist_end;
        u = u(:);

        [g_all, dg_all] = obsSegGradient(q, DH, obs, rho0+safe_margin, p_all, rod_offset_arr, obs_lines);
        if ~isempty(g_all)
            min_g = min(g_all);
            % erfc 势场速度缩放：距离越近，φ→1，速度越低
            phi = erfc(min_g / (sqrt(2) * obs_sigma));
            scale = max(0.05, 1 - phi);
            if true  % placeholder for removed inner if
            s = s * scale;
            end
        end

%}
        % ====== 新物理引擎：对数屏障 + 动量 + 自适应 Δt ======
        % 1. 对数屏障梯度（障碍排斥力 f_barrier）
        f_barrier = zeros(N,1);
        if has_obs && ~isempty(g_all)
            g_eff = g_all - rho0;                        % 纯间隙
            g_eff(g_eff < 1e-8) = 1e-8;                  % 防除零
            dbarrier_dg = -1 ./ g_eff;                   % 纯对数导数 ∂V/∂g = -1/g
            f_barrier = gamma_soft * sum(dbarrier_dg .* dg_all, 1)';
        end
        % 梯度死区破对称：臂伸直时 dg≡0，添加偏近端扰动产生弯曲
        if has_obs && ~isempty(g_all) && min(g_all) < rho_critical && norm(f_barrier) < 1e-3
            pert = randn(N,1) .* (1:N)';
            pert = pert / max(norm(pert), 1e-8);
            f_barrier = f_barrier + gamma_soft * 0.5 * pert;
        end
        
        % 2. 目标方向（单位向量）
        u = err_X / max(dist_end, 1e-8);
        u = u(:);
        
        J = planarJac_L(q, DH, rod_offset_arr);  % 提前计算以供物理引擎使用
        % 3. 总梯度（加速度 a = -∇V = 目标吸引 + 障碍排斥）
        grad_total = gamma_soft * J' * u - f_barrier;
        
        % 4. 动量累积（速度 v_k = β·v_{k-1} + (1-β)·a_k）
        %    靠近目标时 β → 0，消除震荡
        beta_eff = momentum_beta * erf(dist_end / (sqrt(2) * 0.02));
        % 靠近障碍时削弱动量，避免惯性穿透软屏障
        if ~isempty(g_all)
            min_g = min(g_all);
            if min_g < rho_critical
                beta_eff = beta_eff * max(0.05, (min_g - rho0) / (rho_critical - rho0 + 1e-8));
            end
            if min_g < rho0 * 1.5
                G_momentum = grad_total;  % 贴脸重置动量
            end
        end
        if isempty(G_momentum)
            G_momentum = grad_total;
        else
            G_momentum = beta_eff * G_momentum + (1 - beta_eff) * grad_total;
        end
        
        % 5. 自适应时间步长 Δt = Δt₀ / λ(g)
        min_g_val = 1.0;
        if ~isempty(g_all), min_g_val = min(g_all); end
        lambda_g = max(1, rho_critical / max(min_g_val, barrier_eps));
        dt = dt_base / lambda_g;
        
        % 6. 合成速度向量与步长 (含目标距离收敛缩放)
        s_dir = G_momentum / max(norm(G_momentum), 1e-8);  % 单位方向
        s = dt * min(norm(G_momentum), barrier_C);         % 基础步长 = Δt × |v|
        % 靠近障碍时缩步（防穿透软屏障后被高位势卡住）
        if ~isempty(g_all) && min_g_val < rho_critical
            s = s * max(0.1, (min_g_val - rho0) / (rho_critical - rho0 + 1e-8));
        end
        % ====== 新物理引擎结束 ======


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

        J = planarJac_L(q, DH, rod_offset_arr);
        total_tr_err = trace(J * Sigma * J');

        % QP求解增量（传入运动经济性参数）
        dq = solveQP_Damped(J, Sigma, s, u, g_all, dg_all, ...
            rho0+safe_margin, dq_step_min, dq_step_max, ...
            q, q_min, q_max, lambda_damp, gamma_soft, ...
            J_ang, err_ang, gamma_ang, ...
            lambda_part, w_part, lambda_activate, joint_activated, lambda_motor, 0, 0, f_barrier);

        % 硬安全边界校验：迭代缩半步长直至安全或归零
        if has_obs
            dq_backoff = dq;
            for backoff = 1:6
                q_test = q + dq_backoff';
                [p_test_all,~] = planarFK_L(q_test,DH,rod_offset_arr);
                [g_check,~] = obsSegGradient(q_test, DH, obs, rho0, p_test_all, rod_offset_arr, obs_lines);
                if isempty(g_check) || min(g_check) >= rho0
                    dq = dq_backoff; break;
                end
                dq_backoff = dq_backoff * 0.5;
                if norm(dq_backoff) < 1e-6, dq = zeros(N,1); break; end
            end
        end
        % 反射逃生：缩半全部失败（臂被卡在障碍内），沿障碍法向弹射
        if has_obs && norm(dq) < 1e-6 && ~isempty(g_all) && min(g_all) < rho0
            [~, idx] = min(g_all);
            dg_escape = dg_all(idx, :)';
            % 梯度死区检测：臂直伸时障碍梯度可能为零，加随机扰动破对称
            if norm(dg_escape) < 1e-4
                dg_escape = randn(N,1) .* (1:N)';  % 偏近端关节更易改变臂形
                dg_escape = dg_escape / norm(dg_escape);
            end
            dq = dq_step_max * 0.5 * (dg_escape / dg_norm);
            dg_norm = max(norm(dg_escape), 1e-8);
        end

        % 停滞判断
        dq_norm = norm(dq);
        if dq_norm < dq_stall_thresh
            stall_counter = stall_counter + 1;
            % 靠近目标(<1mm)时放宽停滞容忍至 3×
            eff_max = stall_count_max;
            if dist_end < 0.001, eff_max = stall_count_max * 3; end
            if stall_counter >= eff_max
                disp("优化停滞：多轮关节增量极小，提前终止迭代");
                printCurrentStatus(iter, q, dist_end, err_X, theta_curr, err_ang, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
                % 绘制代价深度热力图
                plotCostDepthMap(q, params, X_target, theta_end_target);
                break;
            end
        else
            stall_counter = 0;
        end

        % 电机最小步长舍入
        dq_round = motorStepRounding(dq, kappa);
        dq_norm_round = norm(dq_round);
        if dq_norm_round < 1e-12
            disp("所有电机增量小于最小精确转动量κ，无有效动作，终止迭代");
            printCurrentStatus(iter, q, dist_end, err_X, theta_curr, err_ang, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
            % 绘制代价深度热力图
            plotCostDepthMap(q, params, X_target, theta_end_target);
            break;
        end
        q_new = q + dq_round';
        q = q_new;

        % 更新全局激活状态（项②：标记本轮有实际运动的关节）
        joint_activated = joint_activated | (abs(dq_round') > kappa);

        %% 每print_step步输出一次完整参数状态
        if mod(iter, print_step) == 0
            printCurrentStatus(iter, q, dist_end, err_X, theta_curr, err_ang, avg_joint_err, dq_norm_round, stall_counter, stall_count_max);
        end

        %% 绘图刷新（仅更新数据，不重建对象）
        if mod(iter, print_step) == 0
            set(h_arm, 'XData', p_all(:,1), 'YData', p_all(:,2));
            set(h_target, 'XData', X_target(1), 'YData', X_target(2));
            x_end = p_end(1); y_end = p_end(2);
            set(h_txt_total, 'Position', [x_end+0.025, y_end, 0], 'String', ...
                sprintf('总误差tr(JΣJ^T):%.5f', total_tr_err), 'Visible', 'on');
            set(h_txt_angle, 'Position', [x_end+0.025, y_end-0.028, 0], 'String', ...
                sprintf('末端角:%.2f|目标:%.2f|权重:%.1f', theta_curr, theta_end_target, gamma_ang), 'Visible', 'on');
            set(h_txt_param, 'String', sprintf(...
                'L型臂 N=%d 水平长%.2f 相对角串联+局部垂直偏移 目标[%.2f,%.2f] κ=%.5f',...
                N, L_seg, X_target(1), X_target(2), kappa), 'Visible', 'on');
            set(h_title, 'String', sprintf(...
                '迭代:%d | 末端误差:%.4f | 平均方差:%.6f | 角度误差:%.3f | 停滞计数:%d/%d | 舍入后步长范数:%.6f',...
                iter, dist_end, avg_joint_err, abs(err_ang), stall_counter, stall_count_max, dq_norm_round));
            drawnow limitrate;
        end
    end  % end for iter

    hold off;
    disp("===== 迭代流程全部结束 =====");
    q_final = q;
end

%% ===================== RRT 全局搜索求解器 =====================
function [q_best, V_best, success] = runLArmIK_2D_RRT(params)
    % RRT-based global search for IK with collision avoidance.
    % Builds a tree in joint space from q_current = params.q_init,
    % sampling toward the goal. Once the goal region is reached,
    % path is extracted and refined.
    %
    % Returns: q_best (1×N), V_best (cost), success (bool)

    N = params.N;
    L = params.L_seg;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    obs = params.obs;
    rho0 = params.rho0;
    safe_margin = params.safe_margin;
    X_target = params.X_target;
    theta_target = params.theta_end_target;
    rod_offset_arr = params.rod_offset_arr;
    
    DH = zeros(N,4);
    DH(:,3) = L;
    DH(:,2) = 0; DH(:,4) = 0;
    
    max_samples  = params.rrt_max_samples;
    max_step     = params.rrt_max_step;
    goal_bias    = params.rrt_goal_bias;
    goal_eps     = params.rrt_goal_eps;
    
    q_current = params.q_init(:)';
    max_reach = N * L;  % 用于任务空间引导采样
    
    % 读取线段障碍物
    obs_lines_rrt = {};
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)
        obs_lines_rrt = params.obs_lines;
    end
    
    % DH used for FK/collision checks
    % bounding box of all obstacles (for fast culling)
    bb_min = [inf inf]; bb_max = [-inf -inf];
    if size(obs,1) > 0
        bb_min = min(bb_min, [min(obs(:,1)-obs(:,3)), min(obs(:,2)-obs(:,3))]);
        bb_max = max(bb_max, [max(obs(:,1)+obs(:,3)), max(obs(:,2)+obs(:,3))]);
    end
    for li = 1:length(obs_lines_rrt)
        ln = obs_lines_rrt{li};
        bb_min = min(bb_min, min(ln,[],1));
        bb_max = max(bb_max, max(ln,[],1));
    end
    has_obs = ~isempty(obs) || ~isempty(obs_lines_rrt);
    bb_margin = rho0 * 3;
    
    % --- collision-free check ---
    function ok = isCollisionFree(qq)
        [p_all, ~] = planarFK_L(qq, DH, rod_offset_arr);
        % 快速剔除：如果机械臂包围盒远离障碍物，跳过碰撞检测
        if has_obs
            p_min = min(p_all,[],1) - bb_margin;
            p_max = max(p_all,[],1) + bb_margin;
            if p_max(1) < bb_min(1) || p_min(1) > bb_max(1) || ...
               p_max(2) < bb_min(2) || p_min(2) > bb_max(2)
                ok = true; return;
            end
        end
        [g_check, ~] = obsSegGradient(qq, DH, obs, rho0, p_all, rod_offset_arr, obs_lines_rrt);
        if ~isempty(g_check) && min(g_check) < rho0
            ok = false;
        else
            ok = true;
        end
    end

    % --- FK for end-effector ---
    function [p_end, theta] = fkEndpoint(qq)
        [~, p_end] = planarFK_L(qq, DH, rod_offset_arr);
        theta = getEndEffectorAngle_L(qq, DH, rod_offset_arr);
    end

    % --- steer toward sample with step limit ---
    function q_new = steer(q_from, q_to, step)
        delta = q_to - q_from;
        d_norm = norm(delta);
        if d_norm < 1e-12
            q_new = q_from; return;
        end
        if d_norm <= step
            q_new = q_to;
        else
            q_new = q_from + (step / d_norm) * delta;
        end
        % clamp to joint limits
        q_new = max(q_min, min(q_max, q_new));
    end

    % --- goal-directed heuristic (end-effector distance) ---
    function d = goalDist(qq)
        [pe, th] = fkEndpoint(qq);
        d = norm(pe - X_target) + 0.1 * abs(th - theta_target);
    end

    %% RRT main (goal-biased nearest-neighbor)
    tree = q_current';         % N×M matrix
    parent = 0;
    goal_vals = goalDist(q_current);
    
    fprintf('[RRT] Starting global search (%d samples, step=%.3f)...\n', max_samples, max_step);
    fprintf('[RRT] 当前末端: [%.2f, %.2f], 目标: [%.2f, %.2f]\n', ...
        fkEndpoint(q_current), X_target);
    
    for iter = 1:max_samples
        % 任务空间引导采样：偏向末端能接近目标的构型
        q_rand = sampleTaskSpaceGuided(N, q_min, q_max, X_target, DH, rod_offset_arr, ...
            obs, rho0, obs_lines_rrt, max_reach, @isCollisionFree);
        if false  % dead: old uniform sampling
        q_rand = q_min + rand(1,N) .* (q_max - q_min);
        end  % end if false
        
        % Nearest neighbor: joint distance (for tree connectivity)
        [~, idx] = min(vecnorm(tree - q_rand', 2, 1));
        q_near = tree(:, idx)';
        
        % Steer
        q_new = steer(q_near, q_rand, max_step);
        if ~isCollisionFree(q_new), continue; end
        
        % Prefer nodes that progress toward goal
        g_new = goalDist(q_new);
        if g_new > goal_vals(idx) + 0.1
            continue;  % reject moves that worsen goal distance significantly
        end
        
        tree(:, end+1) = q_new';
        if size(tree,1) == 1 && length(q_new) > 1
            tree(end+1:length(q_new), :) = 0;
            tree(:, end) = q_new';
        end
        parent(end+1) = idx;
        goal_vals(end+1) = g_new;
        
        % Check goal reached
        [p_end, th] = fkEndpoint(q_new);
        if norm(p_end - X_target) < goal_eps && abs(th - theta_target) < 0.1
            fprintf('[RRT] 目标区域已到达 (样本 %d/%d, e=%.4f, a=%.3f)\n', iter, max_samples, ...
                norm(p_end-X_target), abs(th-theta_target));
            path_q = q_new;
            p_idx = length(parent);
            while parent(p_idx) > 0
                p_idx = parent(p_idx);
                path_q = [tree(:,p_idx)'; path_q];
            end
            q_refined = refinePath(path_q, params);
            q_best = q_refined(end, :);
            V_best = calcCostValue(q_best, params, X_target, theta_target);
            success = true;
            return;
        end
        
        if mod(iter, 1000) == 0
            fprintf('[RRT] 已采样 %d/%d, 树节点: %d\n', iter, max_samples, size(tree,2));
        end
    end
    
    % If not reached, pick the node closest to goal
    fprintf('[RRT] 未到达目标区域，选择最近节点\n');
    best_idx = 1;
    best_dist = Inf;
    for i = 1:size(tree,2)
        [p_end, ~] = fkEndpoint(tree(:,i)');
        d = norm(p_end - X_target);
        if d < best_dist
            best_dist = d;
            best_idx = i;
        end
    end
    q_best = tree(:, best_idx)';
    V_best = goal_vals(best_idx);
    success = false;
    fprintf('[RRT] 最佳节点误差: %.4f m (树节点数: %d)\n', best_dist, size(tree,2));


%% ===================== 任务空间引导采样 =====================
function q_sample = sampleTaskSpaceGuided(N, q_min, q_max, X_target, DH, rod_offset_arr, ...
        obs, rho0, obs_lines_ts, max_reach, isCollisionFreeFunc)
    % sampleTaskSpaceGuided: 偏向末端能接近目标的关节构型采样
    %
    % 策略：
    %   1. 以概率 p_ws 在任务空间（末端 XY 平面）采样目标附近点
    %   2. 用少量 IK 迭代将末端拉向采样点，得到关节构型
    %   3. 回退：若失败/碰撞，改用均匀关节空间采样
    %
    % 输入：
    %   isCollisionFreeFunc - 函数句柄 @(q) bool 碰撞检测
    
    p_ws = 0.4;          % 任务空间采样概率
    ws_sigma = max_reach * 0.3;  % 任务空间采样半径（目标附近）
    ik_iters = 8;        % 快速 IK 迭代次数
    ik_damp = 0.05;      % IK 阻尼
    ik_step = 0.1;       % IK 步长
    
    if rand < p_ws && ~isempty(X_target)
        % 1. 采样末端附近的工作空间点
        ws_sample = X_target + ws_sigma * randn(1,2);
        % 限制在可达范围
        if norm(ws_sample) > max_reach * 0.95
            ws_sample = ws_sample / norm(ws_sample) * max_reach * 0.95;
        end
        
        % 2. 快速 IK：从随机初始构型用梯度下降逼近采样点
        q_ik = q_min + rand(1,N) .* (q_max - q_min);
        for k = 1:ik_iters
            [p_all, p_end] = planarFK_L(q_ik, DH, rod_offset_arr);
            err = ws_sample - p_end;
            if norm(err) < 0.05, break; end
            J = planarJac_L(q_ik, DH, rod_offset_arr);
            u = err / max(norm(err), 1e-8);
            s = min(ik_step, norm(err));
            JJT = J * J';
            damped_inv = J' / (JJT + ik_damp^2 * eye(2));
            dq = s * damped_inv * u(:);
            q_ik = q_ik + dq';
            q_ik = max(q_min, min(q_max, q_ik));
        end
        
        % 3. 碰撞检测
        if isCollisionFreeFunc(q_ik)
            q_sample = q_ik;
            return;
        end
        % IK 失败 → 回退
    end
    
    % 回退：均匀关节空间采样
    q_sample = q_min + rand(1,N) .* (q_max - q_min);
end

end

function q_refined = refinePath(path_q, params)
    % Refine each waypoint using gradient IK (damped least squares)
    % with collision avoidance.
    k_refine = 10;  % iterations per waypoint
    N = size(path_q, 2);
    q_refined = path_q;
    DH = dh_refine(params.N, params.L_seg);
    rod = params.rod_offset_arr;
    obs = params.obs;
    rho0 = params.rho0;
    safe_margin = params.safe_margin;
    obs_lines_ref = {};
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)
        obs_lines_ref = params.obs_lines;
    end
    has_obs_ref = ~isempty(obs) || ~isempty(obs_lines_ref);
    
    for i = 2:size(path_q, 1)
        q_local = path_q(i, :);
        for k = 1:k_refine
            [p_all, p_end] = planarFK_L(q_local, DH, rod);
            err = params.X_target - p_end;
            dist = norm(err);
            if dist < 1e-4, break; end
            
            J = planarJac_L(q_local, DH, rod);
            u = err / max(dist, 1e-8);
            s_base = min(params.lambdaM, dist);
            
            % Collision check: if close to obstacle, reduce step and skip potentially dangerous moves
            if has_obs_ref
                [g_ref, ~] = obsSegGradient(q_local, DH, obs, rho0+safe_margin, p_all, rod, obs_lines_ref);
                if ~isempty(g_ref) && min(g_ref) < rho0 + safe_margin
                    s_base = s_base * 0.3;
                    if min(g_ref) < rho0
                        q_refined(i, :) = q_local;
                        break;  % skip refinement if already in collision
                    end
                end
            end
            
            % Damped least squares
            JJT = J * J';
            damped_inv = J' / (JJT + params.lambda_damp^2 * eye(2));
            dq = params.gamma_soft * s_base * damped_inv * u(:);
            dq = max(params.dq_step_min, min(params.dq_step_max, dq));
            q_local = q_local + dq';
            q_local = max(params.q_min(:)', min(params.q_max(:)', q_local));
            
            % Verify refined state is collision-free
            if has_obs_ref
                [p_chk, ~] = planarFK_L(q_local, DH, rod);
                [g_chk, ~] = obsSegGradient(q_local, DH, obs, rho0, p_chk, rod, obs_lines_ref);
                if ~isempty(g_chk) && min(g_chk) < rho0
                    q_local = path_q(i, :);  % revert to original waypoint
                    break;
                end
            end
        end
        q_refined(i, :) = q_local;
    end
end

function DH = dh_refine(N, L)
    DH = zeros(N,4);
    DH(:,3) = L;
end

%% 辅助打印函数：输出当前全套关键参数
function printCurrentStatus(iter, q, dist_end, err_X, theta_curr, err_ang, avg_joint_err, dq_norm_round, stall_counter, stall_count_max)
    fprintf("\n==================== 迭代 %d 状态输出 ====================\n", iter);
    fprintf("当前关节角度 q = [");
    fprintf("%.4f ", q);
    fprintf("]\n");
    fprintf("末端当前坐标误差 ΔX = [%.4f, %.4f]，距离范数 = %.4f\n", err_X(1), err_X(2), dist_end);
    fprintf("当前末端角 = %.3f rad，角度误差 = %.3f rad\n", theta_curr, err_ang);
    fprintf("关节平均方差 avg_joint_err = %.6f\n", avg_joint_err);
    fprintf("本轮舍入后关节增量范数 ||dq_round|| = %.6f\n", dq_norm_round);
    fprintf("停滞计数 %d / %d\n", stall_counter, stall_count_max);
    fprintf("==========================================================\n");
end

%% 计算总代价函数 V(q) 用于热力图
function V = calcCostValue(q, params, X_target, theta_target)
    N = params.N;
    DH = zeros(N,4);
    DH(:,3) = params.L_seg;
    DH(:,2) = 0; DH(:,4) = 0;
    rod_offset_arr = params.rod_offset_arr;

    % 1. 位置误差代价
    [~, p_end] = planarFK_L(q, DH, rod_offset_arr);
    errX = X_target - p_end;
    cost_pos = norm(errX)^2;

    % 2. 末端角度代价
    theta_curr = getEndEffectorAngle_L(q, DH, rod_offset_arr);
    cost_ang = params.gamma_ang_base * (theta_target - theta_curr)^2;

    % 3. 关节方差正则项
    cost_sigma = 0;
    for i = 1:N
        s2 = errVar(q(i), params.m_arr(i), params.tau_arr(i), params.sig0_arr(i));
        s2 = max(s2, params.sig_min2);
        cost_sigma = cost_sigma + s2;
    end

    % 4. erfc 势场碰撞代价：φ = erfc(g/(√2·σ))，距离越近 → φ→1，代价越高
    cost_obs = 0;
    obs = params.obs;
    rho0 = params.rho0;
    obs_lines_cost = {};
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)
        obs_lines_cost = params.obs_lines;
    end
    [p_all, ~] = planarFK_L(q, DH, rod_offset_arr);
    % 读取高斯参数
    obs_sigma_cost = 0.15;
    if isfield(params, 'obs_sigma') && ~isempty(params.obs_sigma)
        obs_sigma_cost = params.obs_sigma;
    end
    gamma_obs_cost = 500;
    if isfield(params, 'gamma_obs') && ~isempty(params.gamma_obs)
        gamma_obs_cost = params.gamma_obs;
    end
    [g_all, ~] = obsSegGradient(q, DH, obs, rho0, p_all, rod_offset_arr, obs_lines_cost);
    if ~isempty(g_all)
        cost_obs = gamma_obs_cost * sum(erfc(g_all ./ (sqrt(2) * obs_sigma_cost)));
    end
    if false  % dead block: replaced by Gaussian cost above
        if ~isempty(g_all)
        min_g = min(g_all);
        if min_g < rho0
            cost_obs = 1e6 * (rho0 - min_g)^2;
        end
    end
    end  % end if false (dead block)

    % 5. 关节参与权重代价（新增①：偏离初始构型的加权 L2 代价）
    q_init = params.q_init;
    w_part = params.w_part;
    if isempty(w_part)
        w_part = ones(1, N);
    end
    dq_total = q - q_init;
    cost_part = params.lambda_part * sum(w_part .* (dq_total.^2));

    % 6. 激活关节数代价（新增②：L0 计数，改变的关节数越多代价越大）
    changed = abs(q - q_init) > params.kappa;
    cost_activate = params.lambda_activate * sum(changed);

    % 7. 单电机转动量代价（新增③：总偏离量的 L2 正则）
    cost_motor = params.lambda_motor * sum(dq_total.^2);

    % 总代价（含运动经济性三项）
    V = cost_pos + cost_ang + params.sigma_weight * cost_sigma + cost_obs ...
        + cost_part + cost_activate + cost_motor;
end

%% 绘制末端XY平面价值深度热力图（局部最优可视化）
function plotCostDepthMap(q_opt, params, X_target, theta_target)
    N = params.N;
    DH = zeros(N,4);
    DH(:,3) = params.L_seg;
    DH(:,2) = 0; DH(:,4) = 0;
    rod_offset_arr = params.rod_offset_arr;

    % 可调超参
    perturb_range = 0.2;
    sample_num = 30;

    q1_list = linspace(q_opt(1)-perturb_range, q_opt(1)+perturb_range, sample_num);
    q2_list = linspace(q_opt(2)-perturb_range, q_opt(2)+perturb_range, sample_num);

    X_grid = zeros(sample_num, sample_num);
    Y_grid = zeros(sample_num, sample_num);
    V_grid = zeros(sample_num, sample_num);

    for i = 1:sample_num
        for j = 1:sample_num
            q_sample = q_opt;
            q_sample(1) = q1_list(i);
            q_sample(2) = q2_list(j);
            [~, p_end] = planarFK_L(q_sample, DH, rod_offset_arr);
            X_grid(i,j) = p_end(1);
            Y_grid(i,j) = p_end(2);
            V_grid(i,j) = calcCostValue(q_sample, params, X_target, theta_target);
        end
    end

    figure('Name','末端XY代价深度图｜局部最优检测','Color','w');
    contourf(X_grid, Y_grid, V_grid, 50);
    hold on; grid on; axis equal;
    colormap(jet);
    cb = colorbar;
    cb.Label.String = '总代价 V(q) (越小越优)';
    cb.Label.FontSize = 10;

    % 标记目标点
    plot(X_target(1), X_target(2), 'g*', 'MarkerSize',16, 'MarkerFaceColor','g','DisplayName','目标点位');
    % 标记当前收敛解
    [~, p_opt_end] = planarFK_L(q_opt, DH, rod_offset_arr);
    plot(p_opt_end(1), p_opt_end(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r','DisplayName','当前收敛解');

    % 绘制障碍物
    obs = params.obs;
    if ~isempty(obs)
        for o = 1:size(obs,1)
            viscircles(obs(o,1:2), obs(o,3),'Color','k','LineWidth',2);
        end
    end

    xlabel('末端 X 坐标');
    ylabel('末端 Y 坐标');
    title(sprintf('代价深度热力图｜关节扰动±%.2f rad | 红圈=当前解 绿星=目标', perturb_range));
    legend('Location','best');
    drawnow;
end

%% ===================== 辅助：安全关闭窗口 =====================
function closeIfValid(h)
    if isgraphics(h) && isvalid(h)
        close(h);
    end
end

%% ===================== 底层运动学/QP/梯度子函数 =====================
function [p_all, p_end] = planarFK_L(q,DH,rod_offset_arr)
    n = length(q);
    p_all = zeros(2*n+1, 2);
    P_curr = [0, 0];
    p_all(1,:) = P_curr;
    idx = 2;
    M = [0,0];
    th_sum = 0;
    for i = 1:n
        th_rel = q(i);
        th_sum = th_sum + th_rel;
        L = DH(i,3);
        off = rod_offset_arr(i);
        dx_h = L * cos(th_sum);
        dy_h = L * sin(th_sum);
        Mx = P_curr(1) + dx_h;
        My = P_curr(2) + dy_h;
        M = [Mx, My];
        p_all(idx,:) = M;
        idx = idx + 1;
        dx_v = -off * sin(th_sum);
        dy_v =  off * cos(th_sum);
        P_next = [Mx + dx_v, My + dy_v];
        p_all(idx,:) = P_next;
        idx = idx + 1;
        P_curr = P_next;
    end
    p_end = M;
end

function p_nodes = planarFK_SimpleNode(q,DH,rod_offset_arr)
    n = length(q);
    p_nodes = zeros(n+1,2);
    P_curr = [0,0];
    p_nodes(1,:) = P_curr;
    th_sum = 0;
    for i=1:n
        th_rel = q(i);
        th_sum = th_sum + th_rel;
        L = DH(i,3);
        off = rod_offset_arr(i);
        dx_h = L * cos(th_sum);
        dy_h = L * sin(th_sum);
        Mx = P_curr(1) + dx_h;
        My = P_curr(2) + dy_h;
        dx_v = -off * sin(th_sum);
        dy_v =  off * cos(th_sum);
        P_curr = [Mx + dx_v, My + dy_v];
        p_nodes(i+1,:) = P_curr;
    end
end

function J = planarJac_L(q,DH,rod_offset_arr)
    n = length(q);
    [p_all, p_end] = planarFK_L(q,DH,rod_offset_arr);
    p_nodes = planarFK_SimpleNode(q,DH,rod_offset_arr);
    J = zeros(2,n);
    xn = p_end(1); yn = p_end(2);
    for i=1:n
        xi = p_nodes(i,1); yi = p_nodes(i,2);
        J(1,i) = -(yn - yi);
        J(2,i) = xn - xi;
    end
end

function Jp = planarJacPoint_L(q,DH,idx,rod_offset_arr)
    n = length(q);
    p_nodes = planarFK_SimpleNode(q,DH,rod_offset_arr);
    [p_all,~] = planarFK_L(q,DH,rod_offset_arr);
    if idx <= n+1
        pt = p_nodes(idx,:);
    else
        pt = p_all(idx,:);
    end
    xk = pt(1); yk = pt(2);
    Jp = zeros(2,n);
    for i=1:n
        xi = p_nodes(i,1); yi = p_nodes(i,2);
        Jp(1,i) = -(yk - yi);
        Jp(2,i) = xk - xi;
    end
end

function theta_end = getEndEffectorAngle_L(q, DH, rod_offset_arr)
    [p_all,~] = planarFK_L(q,DH,rod_offset_arr);
    x0 = p_all(end-1,1); y0 = p_all(end-1,2);
    x1 = p_all(end-2,1); y1 = p_all(end-2,2);
    dx = x1 - x0;
    dy = y1 - y0;
    theta_end = atan2(dy, dx);
end

function J_ang = jacEndAngle_L(q, DH, rod_offset_arr)
    n = length(q);
    [p_all,~] = planarFK_L(q,DH,rod_offset_arr);
    xk = p_all(end-1,1); yk = p_all(end-1,2);
    xe = p_all(end-2,1); ye = p_all(end-2,2);
    dx = xe - xk;
    dy = ye - yk;
    L2 = dx^2 + dy^2;
    Jk = planarJacPoint_L(q, DH, n+1, rod_offset_arr);
    Je = planarJacPoint_L(q, DH, n, rod_offset_arr);
    dtheta_dp = 1/L2 * [-dy, dx];
    J_ang = dtheta_dp * (Je - Jk);
end

function sigma2 = errVar(x,m,tau,sig0)
    sigma2 = sig0^2 * exp( -(x - m).^2 / (2*tau^2) );
end

function [g_total, dg_total] = obsSegGradient(q,DH,obs,rho0,p_all,rod_offset_arr,obs_lines)
    % 圆形 + 线段障碍物碰撞检测
    % obs_lines 为可选的第7参数 {[x1,y1; x2,y2], ...}
    g_total = [];
    dg_total = [];
    if nargin < 7, obs_lines = {}; end
    n_seg = size(p_all,1)-1;
    filter_dist = 4 * rho0;
    
    % --- 圆形障碍物 ---
    if ~isempty(obs)
        n_obs = size(obs,1);
        for o = 1:n_obs
            xo = obs(o,1); yo = obs(o,2);
            ro = obs(o,3);
            for seg = 1:n_seg
                p0 = p_all(seg,:); p1 = p_all(seg+1,:);
                [dist, grad_dist] = segCircleDistGrad(p0,p1,xo,yo,ro,q,DH,seg,rod_offset_arr);
                if dist > filter_dist, continue; end
                g_total = [g_total; dist];
                dg_total = [dg_total; grad_dist];
            end
        end
    end
    
    % --- 线段障碍物 ---
    if ~isempty(obs_lines)
        for li = 1:length(obs_lines)
            line = obs_lines{li};
            l0 = line(1,:); l1 = line(2,:);
            for seg = 1:n_seg
                p0 = p_all(seg,:); p1 = p_all(seg+1,:);
                [dist, grad_dist] = segLineDistGrad(q,DH,seg,p0,p1,l0,l1,rod_offset_arr);
                if dist > filter_dist, continue; end
                g_total = [g_total; dist];
                dg_total = [dg_total; grad_dist];
            end
        end
    end
end

function [dist, dg] = segCircleDistGrad(p0,p1,xo,yo,ro,q,DH,seg_idx,rod_offset_arr)
    dx_seg = p1(1)-p0(1);
    dy_seg = p1(2)-p0(2);
    t = clamp(((xo-p0(1))*dx_seg + (yo-p0(2))*dy_seg)/(dx_seg^2+dy_seg^2),0,1);
    p_near = p0 + t*[dx_seg, dy_seg];
    dx = p_near(1)-xo; dy = p_near(2)-yo;
    dist = sqrt(dx^2 + dy^2);
    n = length(q);
    dg = zeros(1,n);
    J0 = planarJacPoint_L(q,DH,seg_idx,rod_offset_arr);
    J1 = planarJacPoint_L(q,DH,seg_idx+1,rod_offset_arr);
    grad_pnear = (1-t)*[dx/dist, dy/dist]*J0 + t*[dx/dist, dy/dist]*J1;
    dg = grad_pnear;
end

function val = clamp(x,low,high)
    val = min(max(x,low),high);
end

function [dist, dg] = segLineDistGrad(q,DH,seg_idx,p0,p1,l0,l1,rod_offset_arr)
    % 精确段-段距离：机械臂段 [p0,p1] 到障碍线段 [l0,l1] 的最短距离及梯度
    % 基于论文 "Efficient Collision Detection for Segments" 的最近点算法
    dx_line = l1(1)-l0(1); dy_line = l1(2)-l0(2);
    len2_line = dx_line^2 + dy_line^2;
    dx_seg = p1(1)-p0(1); dy_seg = p1(2)-p0(2);
    len2_seg = dx_seg^2 + dy_seg^2;
    
    if len2_line < 1e-12 && len2_seg < 1e-12
        % 双退化点
        dist = norm(p0 - l0);
        dg = zeros(1,length(q));
        return;
    end
    
    if len2_line < 1e-12
        % 障碍退化为点，用 segCircleDistGrad（圆半径=0）
        [dist, dg] = segCircleDistGrad(p0,p1,l0(1),l0(2),0,q,DH,seg_idx,rod_offset_arr);
        return;
    end
    if len2_seg < 1e-12
        % 机械臂段退化为点，用点到线距离
        t = clamp(((p0(1)-l0(1))*dx_line + (p0(2)-l0(2))*dy_line)/len2_line, 0, 1);
        nearest = l0 + t*[dx_line, dy_line];
        dx = nearest(1)-p0(1); dy = nearest(2)-p0(2);
        dist = sqrt(dx^2+dy^2);
        J0 = planarJacPoint_L(q,DH,seg_idx,rod_offset_arr);
        if dist < 1e-8
            dg = zeros(1,length(q));
        else
            np = [-dx/dist, -dy/dist];
            dg = np * J0;
        end
        return;
    end
    
    % 一般情况：段到段最短距离
    % 求解双参数 (ta, tb) 使得 ‖A(ta) - B(tb)‖² 最小，其中
    %   A(ta) = p0 + ta*(p1-p0),  ta ∈ [0,1]
    %   B(tb) = l0 + tb*(l1-l0),  tb ∈ [0,1]
    % 无约束解：(ta, tb) 满足 A^T A·ta - A^T B·tb = A^T·(l0-p0)
    %                       -B^T A·ta + B^T B·tb = B^T·(p0-l0)
    dp = l0 - p0;
    ATA = len2_seg;
    BTB = len2_line;
    ATB = dx_seg*dx_line + dy_seg*dy_line;
    ATdp = dx_seg*dp(1) + dy_seg*dp(2);
    BTdp = dx_line*dp(1) + dy_line*dp(2);
    det = ATA*BTB - ATB^2;
    
    if abs(det) < 1e-12
        % 段平行
        ta = 0.5; tb = clamp(ATdp/ATB,0,1);
    else
        ta = (BTB*ATdp - ATB*BTdp) / det;
        tb = (ATB*ATdp - ATA*BTdp) / det;
    end
    ta = clamp(ta, 0, 1);
    tb = clamp(tb, 0, 1);
    
    % 两个最近点
    near_seg = p0 + ta*[dx_seg, dy_seg];
    near_line = l0 + tb*[dx_line, dy_line];
    dx = near_line(1)-near_seg(1);
    dy = near_line(2)-near_seg(2);
    dist = sqrt(dx^2 + dy^2);
    
    J0 = planarJacPoint_L(q,DH,seg_idx,rod_offset_arr);
    J1 = planarJacPoint_L(q,DH,seg_idx+1,rod_offset_arr);
    
    if dist < 1e-8
        dg = zeros(1,length(q));
    else
        % 梯度: ∂dist/∂q = (∂dist/∂near_seg)·(∂near_seg/∂q)
        % ∂near_seg/∂q = (1-ta)·J0 + ta·J1
        np = [-dx/dist, -dy/dist];  % 单位法向（从机械臂指向障碍线）
        Jnear = (1-ta)*J0 + ta*J1;
        dg = np * Jnear;
    end
end

function dq = solveQP_Damped(J,Sigma,s,u,g,dg,rho0,dq_min,dq_max,q,q_min,q_max,...
    lambda_damp,gamma_soft,J_ang,err_ang,gamma_ang,...
    lambda_part,w_part,lambda_activate,joint_activated,lambda_motor,obs_sigma,gamma_obs,f_barrier)
    % 增强版 QP 求解器：在原阻尼最小二乘基础上增加运动经济性三项代价
    %   ① lambda_part * diag(w_part)      — 关节参与权重（L2 正则）
    %   ② lambda_activate * diag(~activated) — 未激活关节额外惩罚（L2 正则）
    %   ③ lambda_motor * I                 — 单电机转动量惩罚（L2 正则）
    n = size(J,2);
    % Sigma 为对角矩阵，用逐元素倒数替代 inv(Sigma)
    invSigma_diag = 1 ./ diag(Sigma);
    w_part_col = w_part(:);
    not_activated_col = double(~joint_activated(:));
    
    % H 矩阵对角项：2/diag(Sigma) + 2*阻尼² + 2*lambda_part*w_part + 2*lambda_activate*not_activated + 2*lambda_motor
    H_diag = 2 * invSigma_diag + 2 * lambda_damp^2 + 2 * lambda_part * w_part_col + ...
             2 * lambda_activate * not_activated_col + 2 * lambda_motor;
    
    H = diag(H_diag) + 2 * gamma_soft * (J'*J) + 2 * gamma_ang * (J_ang' * J_ang);
    
    f = -2 * gamma_soft * s * J' * u ...
        - 2 * gamma_ang * err_ang * J_ang';

    % 对数屏障梯度（外部预计算，直接加入）
    f = f + f_barrier(:);
    if false  % dead: old erfc code
        coeff = -sqrt(2/pi) / obs_sigma * exp(-g.^2 / (2 * obs_sigma^2));
        dphi_total = sum(coeff .* dg, 1)';
        f_obs = gamma_obs * dphi_total;
    else
        f_obs = zeros(n,1);
    end
    % (removed extra end)
    if false  % dead: old debug output
    % 将高斯软约束梯度加入目标函数
    f = f + f_obs;
    % 调试：输出每50次迭代的梯度范数对比
    persistent iter_count_obs;
    if isempty(iter_count_obs), iter_count_obs = 0; end
    iter_count_obs = iter_count_obs + 1;
    if mod(iter_count_obs, 50) == 0 && ~isempty(g)
        fprintf('[障碍梯度] |f_pos|=%.2f |f_obs|=%.2f ratio=%.2f | min_g=%.3f\n', ...
            norm(f - f_obs), norm(f_obs), norm(f_obs)/max(norm(f - f_obs),1e-8), ...
            min(g));
    end
    end  % end if false (dead debug)

    A_qmin = -eye(n);
    b_qmin = -(q_min' - q');
    A_qmax = eye(n);
    b_qmax = q_max' - q';
    A_joint = [A_qmin; A_qmax];
    b_joint = [b_qmin; b_qmax];
    A_obs = []; b_obs = [];  % 高斯软约束替换硬不等式，清空旧变量

    Aineq = [A_obs; A_joint];
    bineq = [b_obs; b_joint];

    lb = dq_min * ones(n,1);
    ub = dq_max * ones(n,1);

    % 快速路径：无碰撞约束时用解析 DLS，避免 quadprog 开销
    if isempty(g)
        dq = -0.5 * (H \ f);
        dq = max(lb, min(ub, dq));
        return;
    end

    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    dq = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],opts);
    if isempty(dq)
        disp("QP无可行解，放大阻尼重试");
        H_diag_retry = 2 * invSigma_diag + 2 * (lambda_damp*2)^2 + 2 * lambda_part * w_part_col + ...
                       2 * lambda_activate * not_activated_col + 2 * lambda_motor;
        H = diag(H_diag_retry) + 2 * gamma_soft * (J'*J) + 2 * gamma_ang * (J_ang' * J_ang);
        dq = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],opts);
        if isempty(dq)
            dq = zeros(n,1);
        end
    end
end

function dq_round = motorStepRounding(dq, kappa)
    n = length(dq);
    dq_round = zeros(n,1);
    for i = 1:n
        val = dq(i);
        abs_val = abs(val);
        if abs_val < kappa
            dq_round(i) = sign(val) * kappa;  % 消除死区
        elseif abs_val < 2*kappa
            dq_round(i) = sign(val) * kappa;
        else
            dq_round(i) = val;
        end
    end
end

%% ===================== 姿态约束运动求解器 =====================
function [q_result, iter_count, converged] = moveWithPoseConstraint(q_current, q_target, params, rod_offset_arr)
    % moveWithPoseConstraint: 在尽量保持末端位姿不变的前提下
    %   朝目标关节构型 q_target 进行多步 QP 迭代运动。
    %
    %   输入:
    %     q_current       - 当前关节角 [1×N]
    %     q_target        - 目标关节角 [1×N]
    %     params          - 参数结构体（需含姿态约束参数）
    %     rod_offset_arr  - 垂直偏移量 [1×N]
    %
    %   输出:
    %     q_result   - 运动后的关节角 [1×N]
    %     iter_count - 实际迭代步数
    %     converged  - 是否收敛到 q_target
    %
    %   参数来源 (params):
    %     lambda_joint_pose  - 关节追踪权重 λ_joint
    %     sigma_pos_pose     - 位置偏差高斯 σ [m] → λ_pos = 1/(2*σ²)
    %     sigma_ang_pose     - 角度偏差高斯 σ [rad] → λ_ang = 1/(2*σ²)
    %     pose_move_max_iter  - 多步迭代最大步数
    %     pose_move_eps       - 关节追踪收敛阈值 [rad]
    %     pose_dq_step_max    - 单步关节增量上限 [rad]
    %     use_rrt_pose_move   - 是否启用 RRT 全局搜索
    %     以及 q_min, q_max, N, L_seg, kappa, obs, rho0, rrt_* 等

    N = params.N;
    L = params.L_seg;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    kappa = params.kappa;
    obs = params.obs;
    rho0 = params.rho0;
    
    lambda_joint = params.lambda_joint_pose;
    sigma_pos = params.sigma_pos_pose;
    sigma_ang = params.sigma_ang_pose;
    max_iter_pose = params.pose_move_max_iter;
    eps_pose = params.pose_move_eps;
    dq_max = params.pose_dq_step_max;
    use_rrt = isfield(params, 'use_rrt_pose_move') && params.use_rrt_pose_move;
    
    % 高斯权重：λ_pos = 1/(2*σ_pos²)，λ_ang = 1/(2*σ_ang²)
    lambda_pos = 1 / (2 * sigma_pos^2);
    lambda_ang = 1 / (2 * sigma_ang^2);
    
    % 读取运动经济性参数（如果有的话）
    if isfield(params, 'lambda_part') && isfield(params, 'w_part')
        lambda_part_pose = params.lambda_part;
        w_part_pose = params.w_part;
    else
        lambda_part_pose = 0;
        w_part_pose = ones(1, N);
    end
    if isfield(params, 'lambda_motor')
        lambda_motor_pose = params.lambda_motor;
    else
        lambda_motor_pose = 0;
    end
    
    DH = zeros(N,4);
    DH(:,3) = L;
    DH(:,2) = 0; DH(:,4) = 0;
    
    if isempty(rod_offset_arr)
        rod_offset_arr = zeros(1, N);
    end
    
    q_result = q_current(:)';
    
    %% --- RRT 全局搜索模式（可选） ---
    if use_rrt
        fprintf('[姿态约束运动] RRT 全局搜索模式...\n');
        % 构建 RRT 搜索的临时 params
        rrt_params = struct();
        rrt_params.N = N;
        rrt_params.L_seg = L;
        rrt_params.q_min = q_min;
        rrt_params.q_max = q_max;
        rrt_params.obs = obs;
        rrt_params.rho0 = rho0;
        rrt_params.safe_margin = params.safe_margin;
        rrt_params.X_target = [0, 0];  % 未使用但rrt函数需要
        rrt_params.theta_end_target = 0;
        rrt_params.rod_offset_arr = rod_offset_arr;
        rrt_params.rrt_max_samples = params.rrt_max_samples;
        rrt_params.rrt_max_step = params.rrt_max_step;
        rrt_params.rrt_goal_bias = params.rrt_goal_bias;
        rrt_params.rrt_goal_eps = params.rrt_goal_eps;
        rrt_params.q_init = q_result;
        rrt_params.lambda_part = 0;
        rrt_params.w_part = ones(1,N);
        rrt_params.lambda_activate = 0;
        rrt_params.lambda_motor = 0;
        rrt_params.sig_min2 = 10;
        rrt_params.sigma_weight = 0;
        rrt_params.m_arr = zeros(1,N);
        rrt_params.tau_arr = ones(1,N);
        rrt_params.sig0_arr = zeros(1,N);
        rrt_params.gamma_ang_base = 0;
        rrt_params.gamma_ang_peak = 0;
        
        % 使用 RRT 搜索从 q_current 到 q_target 的路径
        [q_rrt, ~, success] = runLArmIK_2D_RRT_Config(rrt_params, q_target);
        if success
            % 沿着 RRT 路径的每个节点作为中间目标
            fprintf('[姿态约束运动] RRT 路径成功，按路径逐步运动...\n');
            % 这里简化：如果RRT找到了到q_target附近的路径，
            % 我们以最后一个有效节点作为 warm-start
            q_result = q_rrt;
        else
            fprintf('[姿态约束运动] RRT 未找到可行路径，改用直接 QP 迭代\n');
        end
        clear rrt_params;
    end
    
    %% --- 多步 QP 迭代 ---
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    q = q_result;
    converged = false;
    
    % 记录初始末端位姿作为参考
    [~, p_start] = planarFK_L(q, DH, rod_offset_arr);
    theta_start = getEndEffectorAngle_L(q, DH, rod_offset_arr);
    
    fprintf('[姿态约束运动] 多步迭代 (max %d 步)...\n', max_iter_pose);
    fprintf('  初始末端: [%.4f, %.4f], 角: %.4f rad\n', p_start(1), p_start(2), theta_start);
    fprintf('  σ_pos=%.3f σ_ang=%.3f → λ_pos=%.1f λ_ang=%.1f\n', sigma_pos, sigma_ang, lambda_pos, lambda_ang);
    
    for iter = 1:max_iter_pose
        % 检查关节距离
        dq_total = q_target - q;
        if norm(dq_total) < eps_pose
            converged = true;
            fprintf('[姿态约束运动] 收敛于第 %d 步 | dq_norm=%.6f\n', iter, norm(dq_total));
            break;
        end
        
        % 线性化当前运动学
        [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr);
        J = planarJac_L(q, DH, rod_offset_arr);
        J_ang = jacEndAngle_L(q, DH, rod_offset_arr);
        theta_curr = getEndEffectorAngle_L(q, DH, rod_offset_arr);
        
        % 碰撞梯度（如果有障碍物）
        [g_all, dg_all] = obsSegGradient(q, DH, obs, rho0, p_all, rod_offset_arr);
        
        % 构建 H 矩阵（含运动经济性）
        n = size(J,2);
        W_part = diag(w_part_pose(:));
        H = 2 * lambda_joint * eye(n) ...                   % 关节追踪
          + 2 * lambda_pos * (J' * J) ...                   % 位置偏移惩罚
          + 2 * lambda_ang * (J_ang' * J_ang) ...           % 角度偏移惩罚
          + 2 * lambda_part_pose * W_part ...               % 关节参与权重（故障电机）
          + 2 * lambda_motor_pose * eye(n);                 % 转动量惩罚
        
        % 构建 f 向量
        f = -2 * lambda_joint * (q_target - q)';
        
        % 约束
        % 关节限位
        A_qmin = -eye(n);
        b_qmin = -(q_min' - q');
        A_qmax = eye(n);
        b_qmax = q_max' - q';
        A_joint = [A_qmin; A_qmax];
        b_joint = [b_qmin; b_qmax];
        
        % 避碰约束
        if ~isempty(g_all)
            A_obs = -dg_all;
            b_obs = -(rho0 - g_all);
            Aineq = [A_obs; A_joint];
            bineq = [b_obs; b_joint];
        else
            Aineq = A_joint;
            bineq = b_joint;
        end
        
        lb = -dq_max * ones(n,1);
        ub =  dq_max * ones(n,1);
        
        % QP 求解
        dq = quadprog(H, f, Aineq, bineq, [], [], lb, ub, [], opts);
        if isempty(dq)
            disp('[姿态约束运动] QP 无解，停止');
            break;
        end
        
        % 电机舍入
        dq_round = motorStepRounding(dq, kappa);
        if norm(dq_round) < 1e-12
            fprintf('[姿态约束运动] 步长过小 (第 %d 步), 停止\n', iter);
            break;
        end
        
        q = q + dq_round';
        q = max(q_min, min(q_max, q));  % clamp
        
        if mod(iter, 20) == 0
            [~, p_cur] = planarFK_L(q, DH, rod_offset_arr);
            th_cur = getEndEffectorAngle_L(q, DH, rod_offset_arr);
            dp = norm(p_cur - p_start);
            da = abs(th_cur - theta_start);
            fprintf('  第 %d 步 | dq_norm=%.4f | 位移偏差:%.4f m | 角度偏差:%.4f rad | 关节距:%.4f\n',...
                iter, norm(dq_round), dp, da, norm(q_target - q));
        end
    end
    
    if ~converged
        fprintf('[姿态约束运动] 未收敛，d_joint=%.4f rad\n', norm(q_target - q));
    end
    
    % 输出末端位姿变化
    [~, p_final] = planarFK_L(q, DH, rod_offset_arr);
    theta_final = getEndEffectorAngle_L(q, DH, rod_offset_arr);
    fprintf('[姿态约束运动] 完成 | 末端偏差: Δpos=%.4f m, Δang=%.4f rad | iter=%d\n',...
        norm(p_final - p_start), abs(theta_final - theta_start), iter);
    
    q_result = q;
    iter_count = iter;
end

%% ===================== RRT 关节构型搜索（适配 moveWithPoseConstraint） =====================
function [q_best, cost_best, success] = runLArmIK_2D_RRT_Config(rrt_params, q_target)
    % RRT 搜索从 q_init 到 q_target 的无碰路径
    % 
    % cost = ‖q − q_target‖² （关节空间距离）
    
    N = rrt_params.N;
    L = rrt_params.L_seg;
    q_min = rrt_params.q_min(:)';
    q_max = rrt_params.q_max(:)';
    obs = rrt_params.obs;
    rho0 = rrt_params.rho0;
    rod_offset_arr = rrt_params.rod_offset_arr;
    max_samples = rrt_params.rrt_max_samples;
    max_step = rrt_params.rrt_max_step;
    goal_eps = 0.02;  % 关节空间目标容差 [rad]
    
    DH = zeros(N,4);
    DH(:,3) = L;
    DH(:,2) = 0; DH(:,4) = 0;
    
    q_init = rrt_params.q_init(:)';
    
    % 读取线段障碍物
    obs_lines_cfg = {};
    if isfield(rrt_params, 'obs_lines') && ~isempty(rrt_params.obs_lines)
        obs_lines_cfg = rrt_params.obs_lines;
    end
    
    function ok = isCollisionFree(qq)
        [p_all, ~] = planarFK_L(qq, DH, rod_offset_arr);
        [g_check, ~] = obsSegGradient(qq, DH, obs, rho0, p_all, rod_offset_arr, obs_lines_cfg);
        if ~isempty(g_check) && min(g_check) < rho0
            ok = false;
        else
            ok = true;
        end
    end
    
    % 目标偏向采样：一定概率直接采样 q_target
    tree = q_init';              % N×1 column vector
    parent = 0;
    costs = norm(q_init - q_target)^2;
    
    for iter = 1:max_samples
        % 采样
        if rand < 0.1
            q_rand = q_target;
        else
            q_rand = q_min + rand(1,N) .* (q_max - q_min);
        end
        
        % 最近邻
        [~, idx] = min(vecnorm(tree - q_rand', 2, 1));
        q_near = tree(:, idx)';
        
        % 步进
        delta = q_rand - q_near;
        d_norm = norm(delta);
        if d_norm < 1e-12, continue; end
        if d_norm <= max_step
            q_new = q_rand;
        else
            q_new = q_near + (max_step / d_norm) * delta;
        end
        q_new = max(q_min, min(q_max, q_new));
        
        if ~isCollisionFree(q_new), continue; end
        
        tree(:, end+1) = q_new';
        parent(end+1) = idx;
        costs(end+1) = norm(q_new - q_target)^2;
        
        % 检查目标
        if norm(q_new - q_target) < goal_eps
            % 回溯路径
            path_q = q_new;
            p_idx = length(parent);
            while parent(p_idx) > 0
                p_idx = parent(p_idx);
                path_q = [tree(:,p_idx)'; path_q]; %#ok<AGROW>
            end
            q_best = path_q(end, :);
            cost_best = norm(q_best - q_target)^2;
            success = true;
            fprintf('[RRT Config] 成功: %d samples, 路径节点 %d\n', iter, size(path_q,1));
            return;
        end
    end
    
    % 未到目标，选最近节点
    [~, best_idx] = min(costs);
    q_best = tree(:, best_idx)';
    cost_best = costs(best_idx);
    success = false;
    fprintf('[RRT Config] 未达目标, 最近距离 %.4f rad (samples %d)\n', sqrt(cost_best), max_samples);
end

%% ===================== 辅助函数 =====================
function q_init = randomQInit(params)
    % 随机生成一个关节构型（多启动 RRT 用）
    N = params.N;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    q_init = q_min + rand(1,N) .* (q_max - q_min);
end

%% ===================== RRT* 渐进最优求解器 =====================
function q_best = runLArmIK_2D_RRTStar(params, warm_start, use_ellipse)
    % Informed RRT*: 在多启动 RRT 找到的解基础上
    % 通过椭圆采样 + rewire 渐进优化。
    % warm_start: 当前已知最优解 (1×N)
    % use_ellipse: 是否使用 Informed 椭圆采样
    
    N = params.N;
    L = params.L_seg;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    obs = params.obs;
    rho0 = params.rho0;
    X_target = params.X_target;
    theta_target = params.theta_end_target;
    rod_offset_arr = params.rod_offset_arr;
    max_iter = params.rrt_star_max_iter;
    max_step = params.rrt_max_step;
    rewire_r = params.rrt_star_radius;
    max_reach_star = N * L;  % 任务空间引导采样用
    
    DH = zeros(N,4); DH(:,3) = L; DH(:,2) = 0; DH(:,4) = 0;
    
    % 读取线段障碍物
    obs_lines_star = {};
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)
        obs_lines_star = params.obs_lines;
    end
    
    function ok = isCollisionFree(qq)
        [p_all, ~] = planarFK_L(qq, DH, rod_offset_arr);
        [g_check, ~] = obsSegGradient(qq, DH, obs, rho0, p_all, rod_offset_arr, obs_lines_star);
        ok = isempty(g_check) || min(g_check) >= rho0;
    end
    function [pe, th] = fkEndpoint(qq)
        [~, pe] = planarFK_L(qq, DH, rod_offset_arr);
        th = getEndEffectorAngle_L(qq, DH, rod_offset_arr);
    end
    function qn = steer(qf, qt, st)
        delta = qt - qf; dn = norm(delta);
        if dn < 1e-12, qn = qf; return; end
        if dn <= st, qn = qt; else, qn = qf + (st/dn)*delta; end
        qn = max(q_min, min(q_max, qn));
    end
    
    % 初始化树：从 warm_start 回溯到根
    q_root = params.q_init(:)';
    tree = q_root';
    parent = 0;
    node_to_root_cost = 0;  % cost from root to each node
    R = N * max_step;  % 搜索范围
    
    % 先构建一条从 root 到 warm_start 的简化路径作为初始树
    % 用直线插值
    n_seg = ceil(norm(warm_start(:)' - q_root) / max_step);
    if n_seg > 1
        prev = q_root;
        for s = 1:n_seg
            t = s/n_seg;
            qi = (1-t)*q_root + t*warm_start(:)';
            qi = max(q_min, min(q_max, qi));
            if ~isCollisionFree(qi), continue; end
            tree(:, end+1) = qi';
            parent(end+1) = size(tree,2)-1;
            node_to_root_cost(end+1) = node_to_root_cost(end) + norm(qi-prev);
            prev = qi;
        end
    end
    % 确保 warm_start 在树中
    if norm(tree(:,end)' - warm_start(:)') > 1e-6
        if isCollisionFree(warm_start(:)')
            tree(:, end+1) = warm_start(:);
            parent(end+1) = size(tree,2)-1;
            node_to_root_cost(end+1) = node_to_root_cost(end) + norm(warm_start(:)'-tree(:,end-1)');
        end
    end
    
    best_idx = size(tree,2);
    best_cost_to_goal = calcCostValue(warm_start, params, X_target, theta_target);
    fprintf('[RRT*] 初始 best_cost=%.4f, 树节点 %d\n', best_cost_to_goal, size(tree,2));
    
    for iter = 1:max_iter
        % 椭圆采样（Informed 策略）
        if use_ellipse
            % 采样限制在椭球内: ‖q - q_root‖ + ‖q - warm_start‖ ≤ c_best
            c_best = max(best_cost_to_goal, 1e-3);
        end  % end if use_ellipse (c_best computed)
        %（任务空间引导采样已移至死代码块之后，见下方）
        if false  % 死码：被下方1562行覆盖，跳过
        q_rand = sampleTaskSpaceGuided(N, q_min, q_max, X_target, DH, rod_offset_arr, ...
            obs, rho0, obs_lines_star, max_reach_star, @isCollisionFree);
        % Informed 椭圆过滤：仅接受椭圆内的样本
        if use_ellipse && norm(q_rand - q_root) + norm(q_rand - warm_start(:)') > best_cost_to_goal * 1.5
            continue;  % 超出椭圆，跳过
        end
        end  % end if false (dead WS-guided block)
        if false  % dead: old ellipse/uniform sampling
            % 简化：在关节空间均匀采样 + accept/reject
            for attempt = 1:50
                q_rand = q_min + rand(1,N).*(q_max-q_min);
                if norm(q_rand - q_root) + norm(q_rand - warm_start(:)') <= c_best * 1.5
                    break;
                end
            end
        else
            q_rand = q_min + rand(1,N).*(q_max-q_min);
        end
        % (removed extra end)
        % 任务空间引导采样（覆盖死代码的均匀采样） + Informed 椭圆过滤
        q_rand = sampleTaskSpaceGuided(N, q_min, q_max, X_target, DH, rod_offset_arr, ...
            obs, rho0, obs_lines_star, max_reach_star, @isCollisionFree);
        % ACTIVE: 椭圆过滤
        if use_ellipse && norm(q_rand - q_root) + norm(q_rand - warm_start(:)') > c_best * 1.5
            continue;
        end
        if false  % dead: old line below
        if use_ellipse && norm(q_rand - q_root) + norm(q_rand - warm_start(:)') > best_cost_to_goal * 1.5
            continue;
        end
        end  % end if false (dead ellipse filter)

        
        % 最近邻
        [~, idx] = min(vecnorm(tree - q_rand', 2, 1));
        q_near = tree(:,idx)';
        
        % 步进
        q_new = steer(q_near, q_rand, max_step);
        if ~isCollisionFree(q_new), continue; end
        
        % Rewire: 找邻居
        dists = vecnorm(tree - q_new', 2, 1);
        neighbors = find(dists <= rewire_r);
        
        % 选父节点：最小 root_cost + edge_cost
        best_parent = idx;
        best_cost = node_to_root_cost(idx) + norm(q_new - tree(:,idx)');
        for ni = neighbors
            if node_to_root_cost(ni) + norm(q_new - tree(:,ni)') < best_cost
                if isCollisionFreePath(q_new, tree(:,ni)', max_step, @isCollisionFree, q_min, q_max)
                    best_parent = ni;
                    best_cost = node_to_root_cost(ni) + norm(q_new - tree(:,ni)');
                end
            end
        end
        
        % 添加节点
        tree(:,end+1) = q_new';
        parent(end+1) = best_parent;
        node_to_root_cost(end+1) = node_to_root_cost(best_parent) + norm(q_new - tree(:,best_parent)');
        
        % Rewire 周围邻居
        for ni = neighbors
            if ni == best_parent, continue; end
            proposed_cost = node_to_root_cost(end) + norm(tree(:,ni)' - q_new);
            if proposed_cost < node_to_root_cost(ni)
                if isCollisionFreePath(tree(:,ni)', q_new, max_step, @isCollisionFree, q_min, q_max)
                    parent(ni) = size(tree,2);
                    node_to_root_cost(ni) = proposed_cost;
                end
            end
        end
        
        % 检查目标区域
        [p_end, ~] = fkEndpoint(q_new);
        if norm(p_end - X_target) < params.rrt_goal_eps
            c = calcCostValue(q_new, params, X_target, theta_target);
            if c < best_cost_to_goal
                best_cost_to_goal = c;
                best_idx = size(tree,2);
                warm_start = q_new;
            end
        end
        
        if mod(iter, 250) == 0
            fprintf('[RRT*] iter %d/%d, best_cost=%.4f, nodes=%d\n', iter, max_iter, best_cost_to_goal, size(tree,2));
        end
    end
    
    % 回溯最优路径
    path_q = tree(:,best_idx)';
    if length(parent) >= best_idx && best_idx > 0
        p_idx = parent(best_idx);
        while p_idx > 0
            path_q = [tree(:,p_idx)'; path_q]; %#ok<AGROW>
            p_idx = parent(p_idx);
        end
    end
    q_best = path_q(end, :);
    fprintf('[RRT*] 完成 | best_cost=%.4f, 路径长度 %d\n', best_cost_to_goal, size(path_q,1));
end

function ok = isCollisionFreePath(q_from, q_to, max_step, isFree, q_min, q_max)
    % 检查从 q_from 到 q_to 的直线路径是否无碰
    d = norm(q_to - q_from);
    if d < 1e-8, ok = true; return; end
    n_seg = ceil(d / max_step);
    for s = 1:n_seg
        t = s/n_seg;
        qi = (1-t)*q_from + t*q_to;
        qi = max(q_min, min(q_max, qi));
        if ~isFree(qi), ok = false; return; end
    end
    ok = true;
end

%% ===================== PRM* 渐进最优求解器（优化版 + 空间索引 + 全路径返回） =====================
function [q_best, success, prm_cache, path_q] = runPRMStar(params, prm_cache)
    % 返回:
    %   q_best   - 最优终点关节构型 [1×N]
    %   success  - 是否找到可行路径
    %   prm_cache - 路线图缓存（可保存为.mat复用）
    %   path_q   - A* 搜索到的完整路径 [M×N]，每行一个 waypoint
    path_q = [];  % 默认空
    N = params.N;
    L = params.L_seg;
    q_min = params.q_min(:)';
    q_max = params.q_max(:)';
    obs = params.obs;
    rho0 = params.rho0;
    X_tgt = params.X_target;
    th_tgt = params.theta_end_target;
    rod = params.rod_offset_arr;
    DH = zeros(N,4); DH(:,3) = L;
    obs_lines_loc = {};
    if isfield(params,'obs_lines'), obs_lines_loc = params.obs_lines; end
    use_full = isfield(params,'prm_full') && params.prm_full;
    n_nodes_base = iff(use_full, 50000, 10000);
    % PRM* asymptotic optimality radius: r(n) = γ·(log(n)/n)^(1/d), d=N
    gamma_prm = 5.0;  % tuned for N=4: ~12-15 connections at n=10,000
    q_init_use = params.q_init(:)';

    % ---- 线段障碍物空间索引 ----
    line_grid = struct('cells',{{}},'res',0,'min_xy',[0 0],'nx',0,'ny',0);
    if ~isempty(obs_lines_loc)
        line_grid = buildLineGrid(obs_lines_loc, rho0);
    end

    function ok = isFree(qq)
        [p_all,~] = planarFK_L(qq,DH,rod);
        [g_check,~] = obsSegGradient(qq,DH,obs,rho0,p_all,rod,obs_lines_loc);
        if ~isempty(g_check) && min(g_check) < rho0
            ok = false;
        else
            ok = true;
        end
    end
    function [pe,th] = fkEnd(qq)
        [~,pe] = planarFK_L(qq,DH,rod);
        th = getEndEffectorAngle_L(qq,DH,rod);
    end

    % --- Use cache if available ---
    if nargin >= 2 && ~isempty(prm_cache) && isfield(prm_cache,'nodes')
        fprintf('[PRM*] 使用缓存路线图 (%d nodes)...\n', size(prm_cache.nodes,2));
        nodes = prm_cache.nodes; adj = prm_cache.adj; costs_mtx = prm_cache.costs;
    else
        % --- Build PRM roadmap ---
        n_nodes = n_nodes_base;
        fprintf('[PRM*] Building roadmap (%d nodes, %s, 线段数=%d)...\n', n_nodes, iff(use_full,'full','lite'), length(obs_lines_loc));
        t_build = tic;
        nodes = zeros(N, n_nodes + 2);
        n_valid = 0;
        sample_batch = 1000;
        report_interval = max(1, floor(n_nodes / 10));
        while n_valid < n_nodes
            batch_qs = q_min + rand(sample_batch, N).*(q_max-q_min);
            for bi = 1:size(batch_qs,1)
                if n_valid >= n_nodes, break; end
                qs = batch_qs(bi,:);
                if isFree(qs)
                    n_valid = n_valid + 1;
                    nodes(:, n_valid) = qs';
                end
            end
            if mod(n_valid, report_interval) < sample_batch
                pct = 100 * n_valid / n_nodes;
                fprintf('[PRM*] 采样进度: %d/%d (%.1f%%) | %.1fs\n', n_valid, n_nodes, pct, toc(t_build));
            end
            if toc(t_build) > 60 && n_valid < n_nodes * 0.1
                fprintf('[PRM*] 采样困难 (自由空间占比低)，强制结束采样\n');
                break;
            end
        end
        fprintf('[PRM*] 采样完成: %d 有效节点 (%.1fs)\n', n_valid, toc(t_build));
        nodes = nodes(:, 1:n_valid);
        n_valid = size(nodes,2);
        
        % 半径连接 (PRM* asymptotic optimality)
        % r(n) = γ · (log(n)/n)^(1/N)
        rad = gamma_prm * (log(n_valid)/n_valid)^(1/N);
        range = q_max - q_min;  % C-space diameter per dim
        norm_fac = norm(range); % ~ 2π√N=~4π for N=4
        fprintf('[PRM*] 边连接中 (半径=%.3f rad, γ=%.1f)...\n', rad, gamma_prm);
        adj = cell(n_valid, 1);
        costs_mtx = cell(n_valid, 1);
        rep_edge = max(1, floor(n_valid / 10));
        t_edge = tic;
        n_edges = 0;
        for i = 1:n_valid
            dists = vecnorm(nodes - nodes(:,i), 2, 1);
            neighbors = find(dists <= rad);
            for jj = 1:length(neighbors)
                j = neighbors(jj);
                if j <= i, continue; end  % avoid duplicate edges
                if checkEdgeFreeFast(nodes(:,i)', nodes(:,j)', N, @isFree, q_min, q_max)
                    n_edges = n_edges + 1;
                    d = norm(nodes(:,i)-nodes(:,j));
                    adj{i}(end+1) = j;
                    costs_mtx{i}(end+1) = d;
                    adj{j}(end+1) = i;
                    costs_mtx{j}(end+1) = d;
                end
            end
            if mod(i, rep_edge) == 0
                fprintf('[PRM*] 边连接进度: %d/%d (%.1f%%) | %d 条边 | %.1fs\n', ...
                    i, n_valid, 100*i/n_valid, n_edges, toc(t_edge));
            end
        end
        fprintf('[PRM*] 边连接完成: %d 条边 (%.1fs)\n', n_edges, toc(t_edge));
        fprintf('[PRM*] Roadmap built total (%.1fs), nodes=%d\n', toc(t_build), n_valid);
        prm_cache = struct('nodes',nodes,'adj',{adj},'costs',{costs_mtx});
    end

    % --- Connect start + goal ---
    fprintf('[PRM*] 连接起点和终点...\n');
    nodes(:, end+1) = q_init_use';
    s_idx = size(nodes,2);
    adj{end+1} = []; costs_mtx{end+1} = [];
    q_goal = q_init_use;
    for attempt = 1:500
        q_goal = q_min + rand(1,N).*(q_max-q_min);
        [pe, th] = fkEnd(q_goal);
        if norm(pe - X_tgt) < params.rrt_goal_eps && abs(th - th_tgt) < 0.1 && isFree(q_goal)
            break;
        end
    end
    nodes(:, end+1) = q_goal';
    g_idx = s_idx + 1;
    adj{end+1} = []; costs_mtx{end+1} = [];
    for ii = [s_idx, g_idx]
        dists = vecnorm(nodes(:, 1:g_idx) - nodes(:,ii), 2, 1);
        neighbors = find(dists <= rad);
        for jj = 1:length(neighbors)
            j = neighbors(jj);
            if j == ii, continue; end
            if checkEdgeFreeFast(nodes(:,ii)', nodes(:,j)', N, @isFree, q_min, q_max)
                d = norm(nodes(:,ii)-nodes(:,j));
                adj{ii}(end+1) = j; costs_mtx{ii}(end+1) = d;
                adj{j}(end+1) = ii; costs_mtx{j}(end+1) = d;
            end
        end
    end
    fprintf('[PRM*] 起点目标已连接, start=%d goal=%d\n', s_idx, g_idx);

    % --- A* search ---
    fprintf('[PRM*] A* 搜索中 (图规模 %d nodes, %d edges)...\n', g_idx, n_edges);
    open = [s_idx];
    g_val = inf(1, g_idx); g_val(s_idx) = 0;
    f_val = inf(1, g_idx); f_val(s_idx) = norm(nodes(:,s_idx)-nodes(:,g_idx));
    came_from = zeros(1, g_idx);
    visited = false(1, g_idx);
    t_astar = tic;
    expanded = 0;
    report_a = max(1, floor(g_idx / 5));
    while ~isempty(open)
        [~, mi] = min(f_val(open));
        cur = open(mi);
        open(mi) = [];
        expanded = expanded + 1;
        if mod(expanded, report_a) == 0
            fprintf('[PRM*] A* 进度: %d/%d 节点已展开 | %.1fs\n', expanded, g_idx, toc(t_astar));
        end
        if cur == g_idx, break; end
        visited(cur) = true;
        for ki = 1:length(adj{cur})
            nb = adj{cur}(ki);
            if visited(nb), continue; end
            tentative = g_val(cur) + costs_mtx{cur}(ki);
            if tentative < g_val(nb)
                g_val(nb) = tentative;
                f_val(nb) = tentative + norm(nodes(:,nb)-nodes(:,g_idx));
                came_from(nb) = cur;
                if ~ismember(nb, open), open(end+1) = nb; end
            end
        end
    end
    if g_val(g_idx) == inf
        fprintf('[PRM*] A* failed (%.2fs, expanded %d nodes): no path. fallback RRT.\n', toc(t_astar), expanded);
        q_best = q_init_use; success = false; return;
    end
    % Backtrack full path
    path_q = q_goal;
    c = g_idx;
    while c ~= s_idx
        c = came_from(c);
        path_q = [nodes(:,c)'; path_q];
    end
    q_best = path_q(end, :);
    fprintf('[PRM*] Path found (%.2fs): %d waypoints | A* expanded %d nodes | total graph %d nodes\n', ...
        toc(t_astar), size(path_q,1), expanded, g_idx);
    success = true;
end

function ok = checkEdgeFreeFast(qa, qb, Nvar, isFree, qmin, qmax)
    d = norm(qb - qa);
    if d < 1e-8, ok = true; return; end
    n_seg = ceil(d / 0.6);
    for s = 1:n_seg
        tvar = s/n_seg;
        qi = (1-tvar)*qa + tvar*qb;
        qi = max(qmin, min(qmax, qi));
        if ~isFree(qi), ok = false; return; end
    end
    ok = true;
end

function v = iff(c,a,b)
    if c, v=a; else, v=b; end
end

%% ---- 线段障碍物网格索引 ----
function grid = buildLineGrid(obs_lines, rho0)
    res = 2 * rho0;
    bb_min = [inf inf]; bb_max = [-inf -inf];
    for li = 1:length(obs_lines)
        ln = obs_lines{li};
        bb_min = min(bb_min, min(ln,[],1));
        bb_max = max(bb_max, max(ln,[],1));
    end
    bb_min = bb_min - rho0;
    bb_max = bb_max + rho0;
    nx = max(1, ceil((bb_max(1)-bb_min(1))/res));
    ny = max(1, ceil((bb_max(2)-bb_min(2))/res));
    cells = cell(nx, ny);
    for li = 1:length(obs_lines)
        ln = obs_lines{li};
        xr = floor((ln(:,1) - bb_min(1))/res) + 1;
        yr = floor((ln(:,2) - bb_min(2))/res) + 1;
        for ix = min(xr):max(xr)
            for iy = min(yr):max(yr)
                if ix>=1 && ix<=nx && iy>=1 && iy<=ny
                    cells{ix,iy}(end+1) = li;
                end
            end
        end
    end
    grid.cells = cells;
    grid.res = res;
    grid.min_xy = bb_min;
    grid.nx = nx;
    grid.ny = ny;
end

function collides = lineCellCheck(p_all, grid, rho0, obs_lines)
    res = grid.res;
    x0 = grid.min_xy(1);
    y0 = grid.min_xy(2);
    nx = grid.nx;
    ny = grid.ny;
    n_pts = size(p_all, 1);
    for i = 1:n_pts-1
        p0 = p_all(i,:);
        p1 = p_all(i+1,:);
        xr = floor(([p0(1) p1(1)] - x0)/res) + 1;
        yr = floor(([p0(2) p1(2)] - y0)/res) + 1;
        if any(isnan(xr)) || any(isnan(yr)), continue; end
        xr = [min(xr) max(xr)];
        yr = [min(yr) max(yr)];
        for ix = max(1,xr(1)):min(nx,xr(2))
            for iy = max(1,yr(1)):min(ny,yr(2))
                if isempty(grid.cells{ix,iy}), continue; end
                for li = grid.cells{ix,iy}
                    ln = obs_lines{li};
                    l0 = ln(1,:); l1 = ln(2,:);
                    d = pointSegDist(p0, p1, l0, l1);
                    if d < rho0
                        collides = true; return;
                    end
                end
            end
        end
    end
    collides = false;
end



%% ==================== 简化接口：solveIK ====================
function q_final = solveIK(X_target, theta_target, search_mode, print_step)
    % solveIK: 简化逆运动学求解接口
    %
    % 输入：
    %   X_target     - 目标末端位置 [x, y]  (m)
    %   theta_target - 目标末端朝向角 (rad)，可选，默认 0
    %   search_mode  - 全局搜索模式：0=关闭 1=RRT 2=RRT* 3=PRM*
    %                  可选，默认使用 GlobalParams 中的设置
    %   print_step   - 控制台打印间隔，可选，默认 50
    %
    % 输出：
    %   q_final - 最终关节角 [1×N]
    %
    % 示例：
    %   q = solveIK([2.5, 1.0]);                     % 仅目标位置
    %   q = solveIK([2.5, 1.0], 0);                   % 含目标角度
    %   q = solveIK([2.5, 1.0], 0, 2);                 % RRT* 模式
    %   q = solveIK([2.5, 1.0], 0, 3, 20);             % PRM* + 详细输出

    % 设置默认值
    if nargin < 2 || isempty(theta_target), theta_target = 0; end
    if nargin < 3 || isempty(search_mode), search_mode = []; end
    if nargin < 4 || isempty(print_step), print_step = 50; end

    % 读取全局参数
    gp = GlobalParams();
    N = gp.N;
    L = gp.L_seg;
    off = 0.06;

    % 构建 params 结构体
    params = struct();
    params.N = N;
    params.L_seg = L;
    params.X_target = X_target(:)';
    params.theta_end_target = theta_target;
    params.q_init = zeros(1, N);

    % 偏移量
    params.rod_offset_arr = zeros(1, N);
    for k = 1:N
        params.rod_offset_arr(k) = off * (-1)^(k+1);
    end

    % 关节限制
    params.q_min = gp.q_min * ones(1, N);
    params.q_max = gp.q_max * ones(1, N);

    % QP 参数
    params.dq_step_max = gp.dq_step_max;
    params.dq_step_min = gp.dq_step_min;
    params.kappa = gp.kappa;
    params.lambda_m = gp.lambda_m;
    params.lambda_damp = gp.lambda_damp;
    params.gamma_soft = gp.gamma_soft;
    params.gamma_ang_base = gp.gamma_ang_base;
    params.gamma_ang_peak = gp.gamma_ang_peak;
    params.sigma_weight = gp.sigma_weight;

    % 方差参数
    params.m_arr = maybeFill_s(gp.m_arr, N, 0);
    params.sig0_arr = maybeFill_s(gp.sig0_arr, N, [1, 0.06]);
    params.tau_arr = maybeFill_s(gp.tau_arr, N, [1, 0.7]);
    params.mu_e_arr = maybeFill_s(gp.mu_e_arr, N, 0);
    params.sig_min2 = gp.sig_min2;

    % 运动经济性
    params.lambda_part = gp.lambda_part;
    params.w_part = maybeFill_s(gp.w_part, N, 1);
    params.lambda_activate = gp.lambda_activate;
    params.lambda_motor = gp.lambda_motor;

    % 障碍物
    params.obs = gp.obs;
    params.rho0 = gp.rho0;
    params.safe_margin = gp.safe_margin;
    params.obs_lines = gp.obs_lines;
    % obs_sigma/gamma_obs 已移除
    % gamma_obs 已移除
    % 动量动力学
    params.use_momentum = gp.use_momentum;
    params.momentum_beta = gp.momentum_beta;
    params.barrier_C = gp.barrier_C;
    params.barrier_eps = gp.barrier_eps;
    params.dt_base = gp.dt_base;
    params.rho_critical = gp.rho_critical;


    % RRT / RRT* / PRM*
    params.rrt_max_samples = gp.rrt_max_samples;
    params.rrt_num_trees = gp.rrt_num_trees;
    params.rrt_max_step = gp.rrt_max_step;
    params.rrt_goal_bias = gp.rrt_goal_bias;
    params.rrt_goal_eps = gp.rrt_goal_eps;
    params.rrt_star_max_iter = gp.rrt_star_max_iter;
    params.rrt_star_radius = gp.rrt_star_radius;

    % 绘图
    params.plot_pad = gp.plot_pad;
    params.bottom_pad = gp.bottom_pad;

    % 全局搜索模式
    if ~isempty(search_mode)
        params.use_global_search = search_mode;
    else
        params.use_global_search = gp.use_global_search;
    end

    % 调用主求解器
    q_final = runLArmIK_2D(params, print_step);
end

%% 辅助函数：maybeFill（内部版本）
function v = maybeFill_s(arr, N, def)
    if isempty(arr)
        v = def(1:min(end, N));
        if length(v) < N
            v(end+1:N) = def(end);
        end
    else
        v = arr(:)';
        if length(v) < N
            v(end+1:N) = v(end);
        end
    end
    v = v(1:N);
end

function d = pointSegDist(a0, a1, b0, b1)
    mida = (a0 + a1)/2;
    midb = (b0 + b1)/2;
    d1 = pointToSegDist2(mida, b0, b1);
    d2 = pointToSegDist2(midb, a0, a1);
    d = min(sqrt(d1), sqrt(d2));
end

function dsq = pointToSegDist2(p, seg0, seg1)
    dx = seg1(1)-seg0(1); dy = seg1(2)-seg0(2);
    len2 = dx*dx + dy*dy;
    if len2 < 1e-12
        dsq = (p(1)-seg0(1))^2 + (p(2)-seg0(2))^2;
        return;
    end
    t = ((p(1)-seg0(1))*dx + (p(2)-seg0(2))*dy) / len2;
    t = max(0, min(1, t));
    px = seg0(1) + t*dx;
    py = seg0(2) + t*dy;
    dsq = (p(1)-px)^2 + (p(2)-py)^2;
end

%% ==================== FMM+DAG (mode 4) ====================
function [q_solution, success] = runLArmIK_2D_FMMDAG(params, print_step)
    N = params.N; L_seg = params.L_seg;
    rod_offset_arr = params.rod_offset_arr;
    if isempty(rod_offset_arr), rod_offset_arr = zeros(1,N); end
    DH = zeros(N,4); DH(:,3) = L_seg; DH(:,2) = 0; DH(:,4) = 0;
    X_target = params.X_target(:)'; theta_target = params.theta_end_target;
    q_init = params.q_init; safe_m = max(params.safe_margin, 0.03);
    grid_dx = 0.02; if isfield(params,'fmm_grid_dx'), grid_dx = params.fmm_grid_dx; end
    M_stream = 12; if isfield(params,'fmm_M_stream'), M_stream = params.fmm_M_stream; end
    base_layers = 20; if isfield(params,'fmm_base_layers'), base_layers = params.fmm_base_layers; end

    %% Phase 0: direct QP
    fprintf('[FMM-DAG Phase0] direct QP...\n');
    p_start = planarFKEndpoint(q_init, DH, rod_offset_arr);
    if norm(p_start - X_target) < 10*params.lambda_m
        q_solution = q_init; success = true; return;
    end
    try
        q_dir = solveShortQP(q_init, X_target, theta_target, DH, rod_offset_arr, params, 80);
        if norm(planarFKEndpoint(q_dir,DH,rod_offset_arr)-X_target) < 20*params.lambda_m
            q_solution = q_dir; success = true; return;
        end
    catch, end

    %% Phase 1: grid + FMM
    fprintf('[FMM-DAG Phase1] grid+FMM...\n');
    [occ_grid, xs, ys, nx, ny, start_ix, start_iy, goal_ix, goal_iy, x_min, y_min] = ...
        buildOccGrid2D(p_start, X_target, params, safe_m, grid_dx);
    [D, ok_fmm] = fastMarching2D(occ_grid, goal_ix, goal_iy, grid_dx);
    if ~ok_fmm || isinf(D(start_iy, start_ix))
        fprintf('[FMM-DAG] unreachable\n'); q_solution = q_init; success = false; return;
    end
    fprintf('  start dist=%.3f, grid %dx%d\n', D(start_iy,start_ix), nx, ny);
    [Gx, Gy] = computeGradient2D(D, grid_dx);
    max_dist = D(start_iy, start_ix);

    %% Phase 2: streamlines + layering
    fprintf('[FMM-DAG Phase2] %d streamlines...\n', M_stream);
    streamlines = generateStreamlines2D([start_ix,start_iy], D, Gx, Gy, occ_grid, M_stream, max_dist, grid_dx);
    layer_dists = adaptiveLayering2D(streamlines, D, max_dist, base_layers, xs, ys, grid_dx);
    K = length(layer_dists);
    layers = collectLayerNodes(layer_dists, streamlines, D, occ_grid, params, safe_m, xs, ys, x_min, y_min, grid_dx, nx, ny, p_start, X_target);
    fprintf('  %d layers, nodes: ', K);
    for k=1:K, fprintf('%d ', size(layers{k},1)); end; fprintf('\n');

    %% Phase 3: DAG search
    fprintf('[FMM-DAG Phase3] DAG search...\n');
    [waypoints, ok_dag] = dagSearch2D(layers, params, safe_m);
    if ~ok_dag || size(waypoints,1) < 2
        fprintf('[FMM-DAG] DAG no path\n'); q_solution = q_init; success = false; return;
    end
    waypoints(1,:) = p_start; waypoints(end,:) = X_target;
    fprintf('  path: %d waypoints\n', size(waypoints,1));

    %% Phase 4: segmented IK
    fprintf('[FMM-DAG Phase4] segmented IK...\n');
    [q_solution, success] = executeSegmentedIK(waypoints, q_init, theta_target, DH, rod_offset_arr, params, safe_m);
    if success, fprintf('[FMM-DAG] success!\n');
    else, fprintf('[FMM-DAG] IK failed\n'); end
end

function p_end = planarFKEndpoint(q, DH, rod_offset_arr)
    [~, p_end] = planarFK_L(q, DH, rod_offset_arr);
end


function [occ_grid, xs, ys, nx, ny, start_ix, start_iy, goal_ix, goal_iy, x_min, y_min] = ...
        buildOccGrid2D(p_start, X_target, params, safe_m, grid_dx)
    pad = 0.2;
    x_min = min([p_start(1),X_target(1)]) - pad;
    x_max = max([p_start(1),X_target(1)]) + pad;
    y_min = min([p_start(2),X_target(2)]) - pad;
    y_max = max([p_start(2),X_target(2)]) + pad;
    if isfield(params,'obs') && ~isempty(params.obs)
        o = params.obs;
        x_min=min([x_min,o(:,1)'-o(:,3)'-safe_m]); x_max=max([x_max,o(:,1)'+o(:,3)'+safe_m]);
        y_min=min([y_min,o(:,2)'-o(:,3)'-safe_m]); y_max=max([y_max,o(:,2)'+o(:,3)'+safe_m]);
    end
    if isfield(params,'obs_lines') && ~isempty(params.obs_lines)
        for li=1:length(params.obs_lines)
            ln=params.obs_lines{li};
            x_min=min([x_min,ln(:,1)'-safe_m]); x_max=max([x_max,ln(:,1)'+safe_m]);
            y_min=min([y_min,ln(:,2)'-safe_m]); y_max=max([y_max,ln(:,2)'+safe_m]);
        end
    end
    nx=ceil((x_max-x_min)/grid_dx)+1; ny=ceil((y_max-y_min)/grid_dx)+1;
    xs=linspace(x_min,x_max,nx); ys=linspace(y_min,y_max,ny);
    occ_grid=false(ny,nx);
    if isfield(params,'obs') && ~isempty(params.obs)
        for o=1:size(params.obs,1)
            cx=params.obs(o,1); cy=params.obs(o,2); cr=params.obs(o,3)+safe_m;
            [Xq,Yq]=meshgrid(xs,ys);
            occ_grid=occ_grid|((Xq-cx).^2+(Yq-cy).^2<cr^2);
        end
    end
    if isfield(params,'obs_lines') && ~isempty(params.obs_lines)
        for li=1:length(params.obs_lines)
            ln=params.obs_lines{li}; lw=safe_m;
            if size(ln,2)>=3, lw=lw+ln(1,3); end
            for ix=1:nx, for iy=1:ny
                if occ_grid(iy,ix), continue; end
                if pointToSegDist([xs(ix),ys(iy)],ln(1,1:2),ln(end,1:2))<lw
                    occ_grid(iy,ix)=true;
                end
            end, end
        end
    end
    start_ix=round((p_start(1)-x_min)/grid_dx)+1; start_iy=round((p_start(2)-y_min)/grid_dx)+1;
    goal_ix=round((X_target(1)-x_min)/grid_dx)+1; goal_iy=round((X_target(2)-y_min)/grid_dx)+1;
    start_ix=max(1,min(nx,start_ix)); start_iy=max(1,min(ny,start_iy));
    goal_ix=max(1,min(nx,goal_ix)); goal_iy=max(1,min(ny,goal_iy));
    if occ_grid(start_iy,start_ix)
        [si,sj]=findFreeNear(occ_grid,start_iy,start_ix,5);
        if ~isempty(si), start_iy=si; start_ix=sj; end
    end
    if occ_grid(goal_iy,goal_ix)
        [gi,gj]=findFreeNear(occ_grid,goal_iy,goal_ix,5);
        if ~isempty(gi), goal_iy=gi; goal_ix=gj; end
    end
end

function [si,sj]=findFreeNear(occ_grid,i0,j0,mr)
    [ny,nx]=size(occ_grid);
    for r=0:mr, for di=-r:r, for dj=-r:r
        if abs(di)~=r&&abs(dj)~=r, continue; end
        si=i0+di; sj=j0+dj;
        if si>=1&&si<=ny&&sj>=1&&sj<=nx&&~occ_grid(si,sj), return; end
    end, end, end
    si=[]; sj=[];
end

function d=pointToSegDist(p,a,b)
    ab=b-a; ap=p-a; t=dot(ap,ab)/max(dot(ab,ab),1e-12);
    t=max(0,min(1,t)); near=a+t*ab; d=norm(p-near);
end


function [D, success] = fastMarching2D(occ_grid, goal_x, goal_y, dx)
    [ny, nx] = size(occ_grid);
    D = inf(ny, nx); D(goal_y, goal_x) = 0;
    accepted = false(ny, nx);
    hi=[]; hj=[]; hd=[];
    function push(i,j,d)
        hi(end+1)=i; hj(end+1)=j; hd(end+1)=d; c=length(hi);
        while c>1, p=floor(c/2);
            if hd(c)>=hd(p), break; end
            [hi(c),hi(p)]=deal(hi(p),hi(c)); [hj(c),hj(p)]=deal(hj(p),hj(c));
            [hd(c),hd(p)]=deal(hd(p),hd(c)); c=p;
        end
    end
    function [i,j,d]=pop()
        if isempty(hi), i=0;j=0;d=inf; return; end
        i=hi(1);j=hj(1);d=hd(1); hi(1)=hi(end);hj(1)=hj(end);hd(1)=hd(end);
        hi(end)=[];hj(end)=[];hd(end)=[]; c=1; n=length(hi);
        while true, left=2*c;right=2*c+1;smallest=c;
            if left<=n&&hd(left)<hd(smallest),smallest=left;end
            if right<=n&&hd(right)<hd(smallest),smallest=right;end
            if smallest==c, break; end
            [hi(c),hi(smallest)]=deal(hi(smallest),hi(c));
            [hj(c),hj(smallest)]=deal(hj(smallest),hj(c));
            [hd(c),hd(smallest)]=deal(hd(smallest),hd(c)); c=smallest;
        end
    end
    push(goal_y, goal_x, 0); maxp = ny*nx*dx*3;
    while ~isempty(hi)
        [ci,cj,cd]=pop(); if accepted(ci,cj), continue; end
        accepted(ci,cj)=true; D(ci,cj)=cd; if cd>maxp, break; end
        for di=-1:1, for dj=-1:1
            if di==0&&dj==0, continue; end
            ni=ci+di; nj=cj+dj;
            if ni<1||ni>ny||nj<1||nj>nx, continue; end
            if accepted(ni,nj)||occ_grid(ni,nj), continue; end
            dv=[];
            if ni>1&&accepted(ni-1,nj),dv=[dv,D(ni-1,nj)];end
            if ni<ny&&accepted(ni+1,nj),dv=[dv,D(ni+1,nj)];end
            if nj>1&&accepted(ni,nj-1),dv=[dv,D(ni,nj-1)];end
            if nj<nx&&accepted(ni,nj+1),dv=[dv,D(ni,nj+1)];end
            if isempty(dv), dn=cd+dx*sqrt(di^2+dj^2);
            else
                dm=min(dv);
                if length(dv)>=2
                    ds=sort(dv); disc=(ds(1)+ds(2))^2-2*(ds(1)^2+ds(2)^2-dx^2);
                    if disc>0, dn=(ds(1)+ds(2)+sqrt(disc))/2; else dn=dm+dx; end
                else, dn=dm+dx;
                end
            end
            if dn<D(ni,nj), D(ni,nj)=dn; push(ni,nj,dn); end
        end, end
    end
    success=true;
end

function [Gx, Gy] = computeGradient2D(D, dx)
    [ny,nx]=size(D); Gx=zeros(ny,nx); Gy=zeros(ny,nx);
    for i=2:ny-1, for j=2:nx-1
        if isinf(D(i,j)), continue; end
        Gx(i,j)=(D(i,j+1)-D(i,j-1))/(2*dx);
        Gy(i,j)=(D(i+1,j)-D(i-1,j))/(2*dx);
    end, end
end


function streamlines = generateStreamlines2D(start_ij, D, Gx, Gy, occ_grid, M, max_dist, dx)
    streamline=cell(M,1); max_steps=2000; ss=dx*0.5;
    [ny,nx]=size(D);
    for m=1:M
        ci=start_ij(2); cj=start_ij(1);
        if m==1, pi=ci; pj=cj;
        else
            gx0=Gx(min(ny,max(1,ci)),min(nx,max(1,cj)));
            gy0=Gy(min(ny,max(1,ci)),min(nx,max(1,cj)));
            gn=norm([gx0,gy0]); if gn<1e-6, gx0=1;gy0=0;gn=1; end
            angle=2*pi*(m-1)/(M-1); tx=-gy0/gn; ty=gx0/gn; sp=3;
            pi=round(ci+sp*(ty*cos(angle)+gy0/gn*sin(angle)));
            pj=round(cj+sp*(tx*cos(angle)+gx0/gn*sin(angle)));
            pi=max(1,min(ny,pi)); pj=max(1,min(nx,pj));
            if occ_grid(pi,pj)
                [fi,fj]=findFreeNear(occ_grid,pi,pj,4);
                if ~isempty(fi), pi=fi;pj=fj; else continue; end
            end
        end
        path=[pj,pi,D(pi,pj)]; stuck=0;
        for step=1:max_steps
            if pi<1||pi>ny||pj<1||pj>nx, break; end
            if D(pi,pj)<dx*2, break; end
            gx=Gx(min(ny,max(1,pi)),min(nx,max(1,pj)));
            gy=Gy(min(ny,max(1,pi)),min(nx,max(1,pj)));
            gn=norm([gx,gy]);
            if gn<1e-6||stuck>15
                tx=randn();ty=randn();tn=norm([tx,ty]);tx=tx/tn;ty=ty/tn;
                ni=round(pi+ty*ss/dx); nj=round(pj+tx*ss/dx); stuck=stuck+1;
            else
                ni=round(pi-gy/gn*ss/dx); nj=round(pj-gx/gn*ss/dx); stuck=0;
            end
            ni=max(1,min(ny,ni)); nj=max(1,min(nx,nj));
            if occ_grid(ni,nj), ni=pi;nj=pj;stuck=stuck+1; end
            if ni==pi&&nj==pj&&stuck<15, stuck=stuck+1; end
            pi=ni;pj=nj; path=[path;pj,pi,D(pi,pj)];
        end
        streamline{m}=path;
    end
    streamlines=streamline;
end

function layer_dists = adaptiveLayering2D(streamlines, D, max_dist, base_layers, ~, ~, ~)
    ref=streamlines{1};
    if isempty(ref)||size(ref,1)<3
        layer_dists=linspace(max_dist,0,base_layers+1); return;
    end
    ns=min(size(ref,1),200); idx=round(linspace(1,size(ref,1),ns));
    dv=ref(idx,3)';
    if length(dv)>=3, curv=abs([0,diff(dv,2),0]); else curv=zeros(size(dv)); end
    curv=curv/max(max(curv),1e-6);
    bs=max_dist/base_layers; ls=bs./(1+5*curv); ls=max(bs*0.2,min(bs*2,ls));
    layer_dists=max_dist; cur=max_dist; si=1;
    while cur>0&&length(layer_dists)<100
        cur=cur-ls(min(si,length(ls)));
        if cur>0, layer_dists(end+1)=cur; end; si=si+1;
    end
    layer_dists(end+1)=0; layer_dists=sort(layer_dists,'descend');
end

function layers = collectLayerNodes(layer_dists, streamlines, D, occ_grid, params, safe_m, xs, ys, x_min, y_min, grid_dx, nx, ny, p_start, X_target)
    K=length(layer_dists); layers=cell(K,1);
    for k=1:K
        dt=layer_dists(k); nk=[];
        for m=1:length(streamlines)
            sl=streamlines{m};
            for i=2:size(sl,1)
                dp=sl(i-1,3); dc=sl(i,3);
                if (dp-dt)*(dc-dt)<=0&&abs(dp-dc)>1e-8
                    t=(dt-dp)/(dc-dp);
                    nk=[nk;sl(i-1,1)+t*(sl(i,1)-sl(i-1,1)),sl(i-1,2)+t*(sl(i,2)-sl(i-1,2))]; break;
                end
            end
        end
        if k<K && isfield(params,'obs')&&~isempty(params.obs)
            dn=layer_dists(k+1);
            for o=1:size(params.obs,1)
                cx=params.obs(o,1);cy=params.obs(o,2);cr=params.obs(o,3)+safe_m;
                for ang=0:pi/4:2*pi-pi/8
                    tx=cx+cr*cos(ang); ty=cy+cr*sin(ang);
                    tix=round((tx-x_min)/grid_dx)+1; tiy=round((ty-y_min)/grid_dx)+1;
                    if tix>=1&&tix<=nx&&tiy>=1&&tiy<=ny
                        dv=D(tiy,tix);
                        if dv>=dn&&dv<=dt, nk=[nk;tx,ty]; end
                    end
                end
            end
        end
        if ~isempty(nk)
            nk=unique(round(nk/grid_dx)*grid_dx,'rows');
            keep=true(size(nk,1),1);
            for ni=1:size(nk,1)
                nix=round((nk(ni,1)-x_min)/grid_dx)+1; niy=round((nk(ni,2)-y_min)/grid_dx)+1;
                if nix>=1&&nix<=nx&&niy>=1&&niy<=ny&&occ_grid(niy,nix), keep(ni)=false; end
            end
            nk=nk(keep,:);
        end
        layers{k}=nk;
    end
    if isempty(layers{1}), layers{1}=p_start; end
    if isempty(layers{K}), layers{K}=X_target; end
end


function [waypoints, ok] = dagSearch2D(layers, params, safe_m)
    K=length(layers); np=[]; ls=zeros(K+1,1);
    for k=1:K
        ls(k)=size(np,1)+1; np=[np;layers{k}];
    end
    ls(K+1)=size(np,1)+1; nt=size(np,1);
    s=[]; t=[]; w=[];
    for k=1:K-1
        nk=layers{k}; nkp=layers{k+1};
        if isempty(nk)||isempty(nkp), continue; end
        for i=1:size(nk,1), for j=1:size(nkp,1)
            u=ls(k)+i-1; v=ls(k+1)+j-1;
            [fe,cst]=checkEdge2D(nk(i,:),nkp(j,:),params,safe_m);
            if fe, s=[s;u]; t=[t;v]; w=[w;cst]; end
        end, end
    end
    dt=inf(nt,1); pr=zeros(nt,1);
    for il=1:size(layers{1},1), u=ls(1)+il-1; dt(u)=0; end
    for k=1:K-1
        for il=1:size(layers{k},1)
            u=ls(k)+il-1; if isinf(dt(u)), continue; end
            ei=find(s==u);
            for ei2=1:length(ei)
                v=t(ei(ei2)); nd=dt(u)+w(ei(ei2));
                if nd<dt(v), dt(v)=nd; pr(v)=u; end
            end
        end
    end
    gb=0; gbd=inf;
    for il=1:size(layers{K},1)
        v=ls(K)+il-1;
        if dt(v)<gbd, gbd=dt(v); gb=v; end
    end
    if gb==0||isinf(gbd), waypoints=[]; ok=false; return; end
    ns=gb;
    while ns(end)~=0&&pr(ns(end))~=0
        ns(end+1)=pr(ns(end));
        if length(ns)>2000, break; end
    end
    ns=flip(ns); if ns(1)==0, ns=ns(2:end); end
    waypoints=np(ns,:); ok=true;
end

function [feas, cost] = checkEdge2D(p1, p2, params, safe_m)
    seg=p2-p1; sl=norm(seg);
    if sl<1e-8, feas=false; cost=inf; return; end
    nc=max(3,ceil(sl/0.01)); mc=inf;
    for t=linspace(0,1,nc)
        pt=p1+t*seg;
        if isfield(params,'obs')&&~isempty(params.obs)
            for o=1:size(params.obs,1)
                cx=params.obs(o,1); cy=params.obs(o,2); cr=params.obs(o,3);
                d=norm(pt-[cx,cy])-cr; mc=min(mc,d);
                if d<safe_m, feas=false; cost=inf; return; end
            end
        end
        if isfield(params,'obs_lines')&&~isempty(params.obs_lines)
            for li=1:length(params.obs_lines)
                ln=params.obs_lines{li}; lw=safe_m;
                if size(ln,2)>=3, lw=lw+ln(1,3); end
                d=pointToSegDist(pt,ln(1,1:2),ln(end,1:2));
                mc=min(mc,d-lw+safe_m);
                if d<lw, feas=false; cost=inf; return; end
            end
        end
    end
    feas=true; cost=sl*(1+0.5/max(mc,1e-6));
end


function [q_solution, success] = executeSegmentedIK(waypoints, q_init, theta_target, DH, rod_offset_arr, params, safe_m)
    qc=q_init; nw=size(waypoints,1);
    for i=2:nw
        wp=waypoints(i,:); tp=(i-1)/(nw-1);
        wt=tp*theta_target+(1-tp)*getEndEffectorAngle_L(qc,DH,rod_offset_arr);
        try
            qs=solveShortQP(qc,wp,wt,DH,rod_offset_arr,params,60);
        catch
            qs=solveShortQP(qc,wp,wt,DH,rod_offset_arr,params,10);
        end
        pe=planarFKEndpoint(qs,DH,rod_offset_arr);
        if norm(pe-wp)>20*params.lambda_m
            q_solution=qc; success=false; return;
        end
        [pa,~]=planarFK_L(qs,DH,rod_offset_arr);
        if checkArmCollision2D(pa,params,safe_m)
            qs=solveShortQP(qc,wp,wt,DH,rod_offset_arr,params,10);
            [pa,~]=planarFK_L(qs,DH,rod_offset_arr);
            if checkArmCollision2D(pa,params,safe_m)
                q_solution=qc; success=false; return;
            end
        end
        qc=qs;
    end
    q_solution=qc; success=true;
end

function dq_round = motorStepRounding2D(dq, kappa)
    n=length(dq); dq_round=zeros(n,1);
    for i=1:n
        av=abs(dq(i));
        if av<kappa, dq_round(i)=sign(dq(i))*kappa;
        elseif av<2*kappa, dq_round(i)=sign(dq(i))*kappa;
        else, dq_round(i)=dq(i);
        end
    end
end

function collides = checkArmCollision2D(p_all, params, safe_m)
    collides=false;
    ob=[]; if isfield(params,'obs'), ob=params.obs; end
    ol=[]; if isfield(params,'obs_lines'), ol=params.obs_lines; end
    for seg=1:size(p_all,1)-1
        p0=p_all(seg,:); p1=p_all(seg+1,:);
        if ~isempty(ob)
            for o=1:size(ob,1)
                cx=ob(o,1); cy=ob(o,2); cr=ob(o,3);
                if pointToSegDist([cx,cy],p0,p1)<cr+safe_m, collides=true; return; end
            end
        end
        if ~isempty(ol)
            for li=1:length(ol)
                ln=ol{li}; lw=safe_m;
                if size(ln,2)>=3, lw=lw+ln(1,3); end
                if min(pointToSegDist(p0,ln(1,1:2),ln(end,1:2)),...
                       pointToSegDist(p1,ln(1,1:2),ln(end,1:2)))<lw
                    collides=true; return;
                end
            end
        end
    end
end


function q_out = solveShortQP(q_init, X_target, theta_target, DH, rod_offset_arr, params, max_iter)
    N=params.N; q=q_init;
    qm=params.q_min; qx=params.q_max; dqx=params.dq_step_max; dqn=params.dq_step_min;
    kp=params.kappa; r0=params.rho0; sm=params.safe_margin;
    ob=[]; if isfield(params,'obs'), ob=params.obs; end
    ol=[]; if isfield(params,'obs_lines'), ol=params.obs_lines; end
    for iter=1:max_iter
        [pa,pe]=planarFK_L(q,DH,rod_offset_arr);
        eX=X_target-pe; de=norm(eX); tc=getEndEffectorAngle_L(q,DH,rod_offset_arr); ea=theta_target-tc;
        if de<params.lambda_m&&abs(ea)<params.lambda_m, break; end
        sb=min(params.lambdaM,de); sc=1.0;
        if de>0.2, sc=1.8; elseif de>0.05, sc=1.2; end
        s=sb*sc;
        if de>1e-12, up=eX/de; else up=[0,0]; end
        J=planarJac_L(q,DH,rod_offset_arr); Ja=jacEndAngle_L(q,DH,rod_offset_arr);
        Sg=eye(N)*params.sig_min2;
        for ii=1:N
            s2=errVar(q(ii),params.m_arr(ii),params.tau_arr(ii),params.sig0_arr(ii));
            Sg(ii,ii)=max(s2,params.sig_min2);
        end
        [ga,dga]=obsSegGradient(q,DH,ob,r0+sm,pa,rod_offset_arr,ol);
        fb=zeros(N,1);
        dq=solveQP_Damped(J,Sg,s,up(:),ga,dga,r0+sm,dqn,dqx,q,qm,qx,...
            params.lambda_damp,params.gamma_soft,Ja,ea,0.01,0,ones(N,1),0,false(1,N),0,0.15,500,fb);
        if isempty(dq), dq=zeros(N,1); end
        dr=motorStepRounding2D(dq,kp);
        if norm(dr)<1e-12, break; end
        q=q+dr';
    end
    q_out=q;
end
