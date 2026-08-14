function kf = kalmanTarget2D(opts)
%kalmanTarget2D 构造目标 2D 位姿 Kalman 滤波器
%   kf = kalmanTarget2D(opts)
%   opts: .R（观测噪声 3×3） .Q（过程噪声 5×5） .x0（初始状态 [px,py,yaw,vx,vy]）
%   使用（独立函数，避免句柄作用域问题）：
%     kf = kalmanTargetUpdate(kf, z)   观测更新（z=[x,y,yaw]，首帧直接赋值）
%     kf = kalmanTargetPredict(kf, dt) 时间推进
%     kf.x = [px, py, yaw, vx, vy]
    if nargin < 1 || isempty(opts), opts = struct(); end
    kf = struct();
    kf.dt = 0.1;
    if isfield(opts, 'x0'), kf.x = opts.x0(:); else, kf.x = zeros(5, 1); end
    % P 初值大（首帧观测含手眼未收敛误差，可达 0.5m 级）
    kf.P = diag([0.5^2, 0.5^2, 0.3^2, 0.1, 0.1]);
    % R 默认含视觉噪声 + 手眼早期不确定性（3cm 位置 / 0.05 rad 角度）
    if isfield(opts, 'R'), kf.R = opts.R; else, kf.R = diag([0.03^2, 0.03^2, 0.05^2]); end
    if isfield(opts, 'Q'), kf.Q = opts.Q; else, kf.Q = diag([1e-4, 1e-4, 1e-4, 1e-5, 1e-5]); end
    kf.init = false;
end
