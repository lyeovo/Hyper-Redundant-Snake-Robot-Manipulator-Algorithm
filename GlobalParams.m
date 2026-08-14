%% GlobalParams.m — 全局参数定义文件
%  为 PlanarDrawApp 与 runLArmIK_2D 提供统一的参数预设。
%
%  调用方式：
%    p = GlobalParams();                 % 返回默认参数结构体
%    p = GlobalParams('field', value);   % 覆盖指定字段
%    p = GlobalParams('reset');          % 重置为默认值
%
function p = GlobalParams(varargin)
    % 持久化参数存储
    persistent PARAMS;
    if isempty(PARAMS)
        PARAMS = initDefaults();
    end
    
    % 处理输入
    if nargin == 0
        p = PARAMS;
        return;
    end
    
    if nargin == 1 && strcmpi(varargin{1}, 'reset')
        PARAMS = initDefaults();
        p = PARAMS;
        return;
    end
    
    % 逐字段覆盖
    i = 1;
    while i <= nargin
        if ischar(varargin{i}) && i < nargin
            fname = varargin{i};
            fval  = varargin{i+1};
            if isfield(PARAMS, fname)
                PARAMS.(fname) = fval;
            else
                warning('GlobalParams: 字段 ''%s'' 不存在，已新增。', fname);
                PARAMS.(fname) = fval;
            end
            i = i + 2;
        else
            i = i + 1;
        end
    end
    p = PARAMS;
end

function p = initDefaults()
    %% ===================== 机械臂几何参数 =====================
    p.N       = 4;        % 机械臂段数
    p.L_seg   = 1.0;      % 每段臂长 [m]
    
    %% ===================== 关节限位 =====================
    p.q_min   = -pi;      % 关节角下限 [rad]
    p.q_max   = +pi;      % 关节角上限 [rad]
    
    %% ===================== IK 求解参数 =====================
    p.max_iter   = 800;       % 最大迭代次数（动量系统需更多迭代）
    p.lambda_m   = 1e-5;      % 收敛阈值：末端位置误差范数 [m]
    p.lambda_damp = 0.01;     % 阻尼最小二乘阻尼系数
    p.dq_step_min = -0.15;    % 关节增量下限（放宽以适应动量动力学）
    p.dq_step_max = 0.15;     % 关节增量上限（放宽以适应动量动力学）
    p.dq_stall_thresh = 1e-5; % 停滞判断阈值
    p.stall_count_max = 8;    % 停滞最大容忍次数
    
    %% ===================== 自适应步长参数 =====================
    p.lambdaM       = 0.1;    % 最大步长上限 [m]
    p.kappa         = 3.2e-5; % 电机最小转动精度
    p.gamma_soft    = 1000;   % 位置误差追踪权重（model2 调优值）
    p.gamma_ang_base = 0.01;  % 角度误差基础权重
    p.gamma_ang_peak = 10;    % 角度误差峰值权重
    p.sigma_weight   = 0.1;   % 角度权重衰减速率
    
    %% ===================== 关节方差参数（用于不确定性建模） =====================
    p.m_arr    = [];     % 由 N 动态计算
    p.sig0_arr = [];     % 由 N 动态计算
    p.tau_arr  = [];     % 由 N 动态计算
    p.mu_e_arr = [];     % 由 N 动态计算
    p.sig_min2 = 0.5;     % 方差下限 (model2 调优值)
    
    %% ===================== 运动经济性代价参数（新增三项） =====================
    % ① 关节参与权重：每个关节各有"启动代价"，抑制高代价关节的动作
    p.lambda_part = 1e-3;           % 关节参与权重系数 λ₁（越大越抑制高 w_part 关节运动）
    p.w_part      = [];             % 各关节单位转动代价向量 [1×N]，由 N 动态初始化（如均设为 1）
    
    % ② 全局激活关节数权重：激活此前未运动的关节时引入额外代价，鼓励复用已有运动链
    p.lambda_activate = 5e-3;       % 激活惩罚系数 λ₂（越大越倾向于少激活新关节）
    
    % ③ 单电机转动量权重：单次迭代中单个关节转动角度越大，代价越高（L2 正则）
    p.lambda_motor = 1e-4;          % 转动量惩罚系数 λ₃（越大越抑制大幅转动）
    
    %% ===================== 障碍物参数 =====================
    p.obs         = [];   % 圆形障碍物列表 [x, y, r; ...]
    p.rho0        = 0.05; % 硬安全边界半径 [m]（绝对不允许越过的最小距离）
    p.safe_margin = 0.01; % 安全裕度 [m]
    p.obs_lines   = {};   % 线段障碍物列表（由红色绘画转换）{i} = [x1,y1; x2,y2]
    
    %% ===================== 对数屏障 + 动量动力学参数 =====================
    p.use_momentum  = true;   % 启用动量累积梯度
    p.momentum_beta = 0.9;    % 动量衰减系数（0=纯梯度，1=纯惯性）
    p.barrier_C     = 200;    % 对数屏障梯度上限 clamp
    p.barrier_eps   = 0.001;  % 对数软化 ε：ln(g-ρ₀+ε)
    p.dt_base       = 0.08;   % 基础时间步长
    p.rho_critical  = 0.15;   % 自适应步长临界距离（≈3ρ₀）
    p.use_elliptic_lines = true;  % 线段障碍椭圆山峰
    p.ellipse_kappa = 3.0;    % 椭圆拉伸系数

    
    %% ===================== 绘图参数 =====================
    p.plot_pad   = 0.15;   % 绘图上下额外边距
    p.bottom_pad = 0.15;   % 绘图下方额外边距
    p.framerate  = 30;     % 刷新帧率 [Hz]
    
    %% ===================== 接口参数 =====================
    p.udp_port   = 12345;  % UDP 监听端口
    
    %% ===================== 末端姿态参数 =====================
    p.theta_end_target = 0;   % 目标末端朝向角 [rad]
    p.rod_offset_arr   = [];  % 垂直偏移量（一般为 0）
    
    %% ===================== 全局搜索参数（RRT / RRT*） =====================
    p.use_global_search = false;  % 是否启用全局搜索（0=关闭, 1=多启动RRT, 2=RRT*最优）
    p.rrt_max_samples   = 5000;  % RRT 最大采样次数（每棵树）
    p.rrt_num_trees     = 3;     % 多启动 RRT 并行树数量
    p.rrt_max_step      = 0.3;   % RRT 单步最大关节增量 [rad]
    p.rrt_goal_bias     = 0.1;   % RRT 目标偏向概率
    p.rrt_goal_eps      = 0.05;  % RRT 末端误差收敛阈值 [m]
    p.rrt_star_max_iter = 2000;  % RRT* 椭圆采样最大额外迭代
    p.rrt_star_radius   = 0.5;   % RRT* rewire 邻居半径 [rad]
    
    %% ===================== 姿态约束运动参数（moveWithPoseConstraint） =====================
    % 在保持末端位姿尽量不变的前提下，朝目标关节构型运动
    p.lambda_joint_pose = 0.1;          % 关节追踪权重 λ_joint（越大越倾向于追逐 q_target）
    p.sigma_pos_pose    = 0.02;         % 末端位置偏差高斯 σ [m]（越小越惩罚位置偏移）
    p.sigma_ang_pose    = 0.1;          % 末端角度偏差高斯 σ [rad]（越小越惩罚角度偏移）
    p.pose_move_max_iter = 200;         % 多步迭代最大步数
    p.pose_move_eps     = 1e-4;         % 关节追踪收敛阈值 [rad]
    p.pose_dq_step_max  = 0.05;         % 单步关节增量上限 [rad]
    p.use_rrt_pose_move = false;        % 是否启用 RRT 全局搜索最优路径后再运动
   
end
