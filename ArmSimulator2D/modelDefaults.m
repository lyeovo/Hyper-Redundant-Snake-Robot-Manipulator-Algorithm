function def = modelDefaults()
%modelDefaults 默认参数表（唯一事实来源）
%   createArmModel 对缺省字段逐项取此默认值；所有默认值集中在此，便于审阅调整
    def = struct();
    % --- 机械臂几何 ---
    def.N = 4;                          % 电机/关节数
    def.L_seg = 1.04393;                % 单段杆长 m（真实机械臂 6×1043.93mm；标量或逐段向量）
    def.q_min = -pi;                    % 关节下限（标量或逐段向量）
    def.q_max =  pi;                    % 关节上限
    def.rod_offset_arr = [];            % 逐段垂直偏移（空 → zeros(1,N)，纯平面模型）
    def.q_init = [];                    % 初始关节角（空 → zeros(1,N)）
    % --- 任务目标 ---
    def.X_target = [2.0, 1.0];          % 末端目标位置 [x,y] m
    def.theta_target = 0.0;             % 末端目标角度 rad
    % --- 价值函数权重 ---
    def.w_pos = 1.0;                    % 末端距离权重
    def.w_ang = 0.3;                    % 末端角度权重
    def.w_obs = 0.5;                    % 障碍屏障权重
    def.w_var = 0;                       % 关节先验权重（默认关闭：避免与位置/角度项形成伪平衡；需要时用户显式开启）
    def.w_acc = 0;                      % 加速度平滑权重（默认关闭：实际从未被 solver 传 v/v_prev，处于休眠态；
                                        %   保留为将来 trajectory smoothing/feedbackCorrect 预留。见 strategy/docs）
    % --- 屏障 ---
    def.rho0 = 0.05;                    % 硬安全距离（机械臂到障碍边缘的最小许可距离）
    def.safe_margin = 0.01;             % 额外安全裕度（兼标定容差）
    def.barrier_C = 200;                % 屏障梯度幅值截断
    def.g_min = 1e-4;                   % 屏障余量下限（防除零，侵入后饱和大排斥）
    def.barrier_range = 0.5;            % 屏障激活距离（g_eff ≥ 此值势为 0；
                                        %   防止 -log(g) 在远处变负势干扰位置收敛）
    % --- 障碍 ---
    def.obstacles.rects = [-1.06, 0.0, 0.0, 2.0, 20.0];  % 后方物理防护墙体障碍物：X <= -0.06m (厚度 2m, 跨度 20m)           % [x,y,θ,w,h; ...] 可旋转矩形
    def.obstacles.circles = [];         % [x,y,r; ...] 圆形
    def.obs_mode = 'seg';               % 'seg'（默认，段级）| 'point'（点级快速）
    % --- 迭代 ---
    def.max_iter = 800;                 % 最大迭代步数
    def.dq_max = 0.15;                  % 单步最大关节增量 rad
    def.tol_pos = 1e-4;                 % 位置收敛阈值 m
    def.tol_ang = 1e-3;                 % 角度收敛阈值 rad（与 tol_pos 分离）
    % --- 动量法 ---
    def.momentum_beta = 0.85;           % 动量系数
    def.dt_base = 0.08;                 % 基础步长
    def.rho_critical = 0.15;            % 障碍邻近临界距离（自适应降速/重置动量）
    % --- RRT ---
    def.rrt_max_samples = 6000;         % 最大采样数（6 关节中等场景 ~8-10s，可覆盖）
    def.rrt_max_step = 0.3;             % 单步最大延伸（关节空间 rad）
    def.rrt_goal_eps = 0.02;            % 目标区域位置容差 m（0.05 太粗：3m 臂下肉眼可见差距仍判"已收敛"）
    def.rrt_goal_ang = 0.1;             % 目标区域角度容差 rad
    % --- 模拟退火 ---
    def.sa_max_iter = 2000;             % 最大迭代
    def.sa_alpha = 0.995;               % 几何冷却系数
    def.sa_sigma0 = 0.3;                % 初始扰动幅度 rad
    % --- PRM ---
    def.prm_n_nodes = 3000;             % 路线图节点数（single source of truth，method_prm 据此读默认）
    def.prm_gamma = 5.0;                % 连接半径系数 γ
    def.prm_rad = 1.2;                  % 固定连接半径（method_prm 现行连接方式，非 PRM* 收缩半径）
    % --- 图引导（method_graph 默认，single source of truth） ---
    def.graph_max_samples = 800;        % 段内 RRT* 预算
    def.graph_seg_retry = 2;            % 同段失败重试次数（失败预算翻倍）
    def.graph_gap_max = 1.0;            % 缝隙宽度阈值 m（超过则末端可直连无需节点）
    def.graph_margin = 0.1;             % 膨胀额外边距 m
    def.graph_max_iters = 8;            % 回溯迭代上限
    def.graph_total_budget = 12000;     % 全任务累计样本预算（防空转）
    % --- 局部最优检测 ---
    def.localmin_tol_grad = 1e-3;       % 梯度范数阈值（低于此且末端未达 → 判定局部最优）
    % --- 关节先验（w_var 项） ---
    def.m_arr = [];                     % 先验均值（空 → zeros(1,N)）
    def.sigma2_arr = [];                % 先验方差 σ_j²（空 → ones(1,N)）
    % --- 输出 ---
    def.snapshot_m = 10;                % 每 m 步快照一行（电机指令节拍）
    % --- 误差模型（默认关闭；开启后 simulateMotion/assessRobustness 注入电机执行误差） ---
    def.error.on = false;               % 总开关
    def.error.sigma_motor = 0.01;       % 电机转角高斯噪声 σ（rad）
    def.error.backlash = 0.005;         % 齿轮回差 δ_b（rad，方向随 Δq）
    def.error.kappa = 0.001;            % 最小精转量 κ（rad 量化）
    % --- 调试 ---
    def.debug.grad_check = false;       % 梯度校验开关（armGradient 自检）
end
