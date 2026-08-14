clear;clc;close all;

%% 2D 蛇形/L型臂加权QP逆解迭代实例
% 基于平面连杆模型 + 相对角度串联 + 局部垂直偏移
% 与 model1.m (3D) 对应：model2 使用 runLArmIK_2D

params = struct();

%% 机械臂本体几何参数
params.N = 6;

% 每段水平连杆长度
params.L_seg = 0.5;

% 末端目标角度（世界坐标系下，rad）
params.theta_end_target = 0;

% 局部垂直偏移（一维数组，对应每段连杆的垂直偏移量）
off = 0.06;
params.rod_offset_arr = [off, -off, off, -off, off, -off];

%% 目标位置（2D）
params.X_target = [1.2, 1.4];

%% 绘图边界参数
params.plot_pad = 0.5;
params.bottom_pad = 0.3;

%% 关节限位、电机最小步长
params.q_min  = [-pi, -pi/2, -pi, -pi, -pi/2, -pi];
params.q_max  = [ pi,  pi/2,  pi,  pi,  pi/2,  pi];
params.dq_step_max = 0.04;
params.dq_step_min = -0.04;
params.kappa = 3.2e-5;

%% 关节误差方差模型参数
params.m_arr     = zeros(1,6);
params.sig0_arr  = [1, 0.06, 0.06, 0.06, 0.06, 0.06];
params.tau_arr   = [1, 0.7, 0.7, 0.7, 0.7, 0.7];
params.mu_e_arr  = [0, pi/2, -pi/2, pi/2, -pi/2, pi/2];
params.sig_min2  = 0.5;

%% QP代价权重
params.lambda_damp    = 0.01;
params.gamma_soft     = 1000;   % 位置追踪权重
params.gamma_ang_base = 0.01;   % 末端角度追踪基础权重
params.gamma_ang_peak = 10;     % 末端角度追踪峰值权重
params.sigma_weight   = 0.1;    % 方差正则权重

%% 迭代停滞判定
params.dq_stall_thresh = 1e-5;
params.stall_count_max = 8;

%% 避障参数
% 2D圆形障碍物：[x, y, r]
params.obs  = [];
params.rho0       = 0.1;
params.safe_margin = 0.05;

%% 迭代控制参数
params.lambdaM  = 0.1;
params.lambda_m = 1e-5;
params.max_iter = 200;

% 初始关节角（接近伸直姿态）
params.q_init = [0, 0.3, -0.3, 0, 0.2, 0];

%% 迭代打印间隔：每5轮输出一次完整状态
print_interval = 5;

%% 启动2D逆解迭代
fprintf('========== 2D L型臂IK求解 ==========\n');
fprintf('目标位置: [%.2f, %.2f]\n', params.X_target);
fprintf('目标末端角度: %.2f rad\n', params.theta_end_target);
fprintf('初始关节角: [%s]\n', num2str(params.q_init, '%.2f '));
fprintf('====================================\n');

%% 启动2D IK迭代求解
runLArmIK_2D(params, print_interval);