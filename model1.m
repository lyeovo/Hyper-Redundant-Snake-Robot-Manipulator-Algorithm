clear;clc;close all;

%% 3D 机械臂加权QP逆解迭代实例
% 基于标准DH参数的三维通用版本
% 姿态采用四元数存储 + 轴角误差IK求解

params = struct();

%% 机械臂本体几何参数
params.N = 6;

% 标准DH参数表：[a, alpha, d, theta_offset]
% 6自由度旋转关节机械臂（类似UR构型）
params.DH = [0,    -pi/2, 0.3, 0;     % 关节1: 绕Z1旋转，X1垂直于Z0
             0.4,   0,    0,   0;     % 关节2: 绕Z2旋转，与Z1平行
             0.3,   0,    0,   0;     % 关节3
             0,    -pi/2, 0.2, 0;     % 关节4
             0,     pi/2, 0.2, 0;     % 关节5
             0,     0,    0.1, 0];    % 关节6

% 3D垂直偏移（各连杆局部坐标系中的偏移量）
% 对于蛇形/L型臂：在局部xy平面内交替偏移
off = 0.06;
params.rod_offset_arr = [0,  off, 0;
                          0, -off, 0;
                          0,  off, 0;
                          0, -off, 0;
                          0,  off, 0;
                          0, -off, 0];

%% 目标位姿
% 目标位置（3D）
params.X_target = [0.2; 0.6; 0.8];

% 目标姿态四元数 [w, x, y, z] —— 绕Z轴180度
params.q_target = [0, 0, 0, 1];  % [cos(pi/2), 0, 0, sin(pi/2)] → 绕Z轴180°

%% 绘图边界参数
params.plot_pad = 0.5;

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
params.gamma_ang_base = 0.01;   % 姿态追踪基础权重
params.gamma_ang_peak = 10;     % 姿态追踪峰值权重
params.sigma_weight   = 0.1;    % 方差正则权重

%% 迭代停滞判定
params.dq_stall_thresh = 1e-5;
params.stall_count_max = 8;

%% 避障参数
% 3D球形障碍物：[x, y, z, r]
params.obs  = [0.6, 0.3, 0.5, 0.12;
               0.9, 0.5, 0.7, 0.10];
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

%% 启动3D逆解迭代
fprintf('========== 3D机械臂IK求解 ==========\n');
fprintf('目标位置: [%.2f, %.2f, %.2f]\n', params.X_target);
fprintf('目标姿态: 四元数 [%.2f, %.2f, %.2f, %.2f]\n', params.q_target);
fprintf('初始关节角: [%s]\n', num2str(params.q_init, '%.2f '));
fprintf('====================================\n');

runLArmIK(params, print_interval);