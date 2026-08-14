function q = solveIK(X_target, theta_target, method, N)
%solveIK 兼容快捷接口（独立文件，修复旧版"子函数对外不可用"问题）
%   q = solveIK([x, y])                         % 仅位置，N=4，auto 调度
%   q = solveIK([x, y], theta)                  % 含末端角度
%   q = solveIK([x, y], theta, method)          % 方法：momentum/sa/rrt/prm/rl/auto
%   q = solveIK([x, y], theta, method, N)       % 自定义关节数
%
%   依赖 ArmSimulator2D/（createArmModel + simulateMotion）。障碍需完整接口：
%   model = createArmModel(struct('obstacles', ...)); info = simulateMotion(model, ...)
    if nargin < 2 || isempty(theta_target), theta_target = 0; end
    if nargin < 3 || isempty(method), method = 'auto'; end
    if nargin < 4 || isempty(N), N = 4; end
    if numel(X_target) ~= 2
        error('solveIK:arg', 'X_target 应为 [x, y]');
    end
    model = createArmModel(struct('N', N, 'q_init', zeros(1, N)));
    info = simulateMotion(model, method, zeros(1, N), [X_target(1), X_target(2), theta_target]);
    q = info.q_final;
    if ~info.success
        warning('solveIK:fail', '未收敛 error_code=%d（pos=%.3f ang=%.3f）', ...
            info.error_code, info.dist_end, info.err_ang);
    end
end
