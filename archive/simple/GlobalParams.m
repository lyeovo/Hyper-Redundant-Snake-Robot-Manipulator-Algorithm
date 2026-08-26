%% GlobalParams.m -- Simplified parameters for runLArmIK_2D_Simple
%% ========================================================================
%  调用方式：
%    p = GlobalParams();                 % 返回默认参数结构体
%    p = GlobalParams('field', value);   % 覆盖指定字段
%    p = GlobalParams('reset');          % 重置为默认值
% ========================================================================
function p = GlobalParams(varargin)
    persistent PARAMS;
    if isempty(PARAMS), PARAMS = initDefaults(); end
    if nargin == 0, p = PARAMS; return; end
    if nargin == 1 && strcmpi(varargin{1}, 'reset')
        PARAMS = initDefaults(); p = PARAMS; return;
    end
    i = 1;
    while i <= nargin
        if ischar(varargin{i}) && i < nargin
            PARAMS.(varargin{i}) = varargin{i+1};
            i = i + 2;
        else, i = i + 1; end
    end
    p = PARAMS;
end

function p = initDefaults()
    %% -- 机械臂几何 --
    p.N       = 4;        % 段数
    p.L_seg   = 1.0;      % 每段臂长 [m]
    
    %% -- 关节限位 --
    p.q_min   = -pi;      % 下限 [rad]
    p.q_max   = +pi;      % 上限 [rad]
    
    %% -- IK 求解 --
    p.max_iter    = 800;     % 最大迭代
    p.lambda_m    = 1e-5;    % 收敛阈值 [m]
    p.dq_step_max = 0.15;    % 最大步长 [rad]
    
    %% -- 统一势函数权重 --
    p.w_pos = 1.0;     % 末端位置权重
    p.w_ang = 0.3;     % 末端角度权重
    p.w_obs = 0.5;     % 障碍对数势权重
    p.w_var = 0.01;    % 电机误差正态权重
    p.w_acc = 0.1;     % 加速度平滑权重
    
    %% -- 动量 --
    p.momentum_beta = 0.85;   % 动量衰减 (0=纯梯度, 1=纯惯性)
    p.barrier_eps   = 0.001;  % 对数屏障软化 epsilon
    
    %% -- 关节方差 --
    p.m_arr    = [];
    p.sig0_arr = [];
    p.tau_arr  = [];
    
    %% -- 障碍物 --
    p.obs         = [];    % 圆形障碍 [x,y,r; ...]
    p.obs_lines   = {};    % 线段障碍 {{[x1,y1;x2,y2]}, ...}
    p.rho0        = 0.05;  % 硬安全边界 [m]
    p.safe_margin = 0.01;  % 安全裕度 [m]
    
    %% -- 绘图 --
    p.plot_pad   = 0.15;
    p.bottom_pad = 0.15;
    p.framerate  = 30;
    
    %% -- 接口 --
    p.udp_port   = 12345;
    
    %% -- 末端姿态 --
    p.theta_end_target = 0;
    p.rod_offset_arr   = [];
    
    %% -- 全局搜索 RRT/RRT*/PRM* --
    p.use_global_search = 0;   % 0=关 1=多启动RRT 2=RRT* 3=PRM*
    p.rrt_max_samples   = 5000;
    p.rrt_num_trees     = 3;
    p.rrt_max_step      = 0.3;
    p.rrt_goal_bias     = 0.1;
    p.rrt_goal_eps      = 0.05;
    p.rrt_star_max_iter = 2000;
    p.rrt_star_radius   = 0.5;
end