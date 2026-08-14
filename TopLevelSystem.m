%% TopLevelSystem.m — 顶层系统接口
% 为外部提供统一的方法入口，管理视觉数据接口、任务调度、任务执行、硬件仿真、参数修改。
%
% 外部调用方式：
%   sys = TopLevelSystem();               % 获取系统句柄
%   sys.visionInterface(pos_start, pos_end, theta_start, theta_end);
%   sys.setParam(fieldName, value);
%   ... 等
%
% ======================== 模块概览 ========================
% 1. 视觉数据接口  (visionInterface)
% 2. 任务调度模块  (taskScheduler)
% 3. 任务执行模块  (taskExecutor)
% 4. 硬件仿真接口  (hardwareInterface)
% 5. 参数修改模块  (paramManager)
% =========================================================

function sys = TopLevelSystem()
    % 返回系统句柄结构体，所有方法通过字段访问
    sys.visionInterface  = @visionInterface;
    sys.taskScheduler    = @taskScheduler;
    sys.taskExecutor     = @taskExecutor;
    sys.hardwareInterface = @hardwareInterface;
    sys.paramManager      = @paramManager;
    sys.getTaskQueue      = @getTaskQueue;
    sys.getMotorState     = @getMotorState;
    sys.getParams         = @getParams;
end

%% ================= 内部持久化状态 =================
function s = getState()
    % 返回持久化全局状态结构体
    persistent STATE;
    if isempty(STATE)
        STATE = initState();
    end
    s = STATE;
end

function s = initState()
    % 系统初始化状态
    s.taskQueue        = {};            % 任务队列 cell array, 每个元素为 task struct
    s.currentMotorParams = zeros(1,0);  % 当前电机参数矢量 (目标关节角)
    s.params           = initDefaultParams(); % 全局参数预设
    s.taskRequested    = false;         % 视觉模块任务请求信号
    s.visionStartPos   = [0, 0];        % 任务起始点坐标 [x, y]
    s.visionEndPos     = [0, 0];        % 任务终点坐标 [x, y]
    s.visionStartTheta = 0;             % 任务起始姿态角
    s.visionEndTheta   = 0;             % 任务终点姿态角
    s.isExecuting      = false;         % 执行锁
end

function p = initDefaultParams()
    % 默认参数预设（与 runLArmIK_2D 兼容）
    p.N               = 6;
    p.L_seg           = 0.15;
    p.theta_end_target = 0;
    p.X_target        = [0.5, 0.3];
    p.rod_offset_arr  = zeros(1, 6);
    p.plot_pad        = 0.15;
    p.bottom_pad      = 0.15;
    p.q_min           = -pi * ones(1, 6);
    p.q_max           =  pi * ones(1, 6);
    p.dq_step_max     = 0.1;
    p.dq_step_min     = 1e-4;
    p.kappa           = 0.001;
    p.m_arr           = zeros(1, 6);
    p.sig0_arr        = 0.02 * ones(1, 6);
    p.tau_arr         = 0.3 * ones(1, 6);
    p.mu_e_arr        = zeros(1, 6);
    p.sig_min2        = 1e-6;
    p.lambda_damp     = 0.5;
    p.gamma_soft      = 1.0;
    p.gamma_ang_base  = 0.5;
    p.gamma_ang_peak  = 2.0;
    p.sigma_weight    = 0.1;
    p.dq_stall_thresh = 1e-4;
    p.stall_count_max = 10;
    p.obs             = [];
    p.rho0            = 0.05;
    p.safe_margin     = 0.01;
    p.lambdaM         = 0.15;
    p.lambda_m        = 0.001;
    p.max_iter        = 200;
    p.q_init          = zeros(1, 6);
    % 任务额外参数预设
    p.moveStepLength  = 0.05;       % move 任务默认步长
    p.moveDirection   = [1, 0];     % move 任务默认方向 (单位矢量)
    p.catchPresetQ    = zeros(1, 6);% catch 任务预设目标电机参数
    p.putPresetQ      = zeros(1, 6);% put 任务预设目标电机参数
    p.ikStepCount     = 10;         % move_to 迭代步长数(每 N 次迭代输出一次)
end

%% ================= 1. 视觉数据接口 =================
function visionInterface(pos_start, pos_end, theta_start, theta_end)
    % visionInterface
    %   服务于外部视觉模块的数据接口。
    %   传入两个同类型矢量：任务起点与终点。
    %   每个矢量包含：二维相对坐标 [x, y] 与姿态角 theta。
    %
    %   调用示例：
    %     visionInterface([0, 0], [0.5, 0.3], 0, pi/4);
    %
    %   内部行为：
    %     将数据存入全局状态，并将任务请求信号呈递给任务调度模块。
    STATE = getState();
    STATE.visionStartPos   = pos_start(:)';
    STATE.visionEndPos     = pos_end(:)';
    STATE.visionStartTheta = theta_start;
    STATE.visionEndTheta   = theta_end;
    STATE.taskRequested    = true;

    fprintf('[视觉接口] 收到任务数据：起点[%.3f, %.3f, θ=%.3f] → 终点[%.3f, %.3f, θ=%.3f]\n', ...
        pos_start(1), pos_start(2), theta_start, ...
        pos_end(1), pos_end(2), theta_end);

    % 呈递至任务调度模块
    taskScheduler();
end

%% ================= 2. 任务调度模块 =================
function taskScheduler()
    % taskScheduler
    %   对视觉模块传入参数进行分析，然后向任务队列创建新任务。
    %   分析部分标记为 TODO，当前行为：直接依据传入参数构造 move_to 任务。
    STATE = getState();
    if ~STATE.taskRequested
        fprintf('[任务调度] 无待处理任务请求。\n');
        return;
    end

    % TODO: 视觉参数分析 —— 识别任务类型、规划路径、决策行为
    % 当前为占位逻辑：一律生成 move_to 类型任务
    fprintf('[任务调度] TODO — 视觉参数分析中（当前默认生成 move_to 任务）...\n');

    task = struct();
    task.type        = 'move_to';                       % 任务类型
    task.startPos    = STATE.visionStartPos;            % 起点坐标
    task.endPos      = STATE.visionEndPos;              % 终点坐标
    task.startTheta  = STATE.visionStartTheta;          % 起点姿态角
    task.endTheta    = STATE.visionEndTheta;            % 终点姿态角
    task.createdAt   = now;                             % 创建时间戳
    task.status      = 'pending';                       % 状态: pending / running / done / failed
    task.params      = [];                              % 任务专属参数覆盖 (struct)
    task.result      = [];                              % 执行结果

    % 入队
    STATE.taskQueue{end+1} = task;
    STATE.taskRequested = false;

    fprintf('[任务调度] 已创建新任务 [%s] 并加入队列（队列长度：%d）\n', ...
        task.type, length(STATE.taskQueue));

    % 若当前未在运行，自动触发执行
    if ~STATE.isExecuting
        taskExecutor();
    end
end

%% ================= 3. 任务执行模块 =================
function taskExecutor()
    % taskExecutor
    %   按照任务队列，执行最新任务。
    %   支持任务类型：move_to, move, catch, put
    STATE = getState();
    if isempty(STATE.taskQueue)
        fprintf('[任务执行] 任务队列为空，无需执行。\n');
        return;
    end
    if STATE.isExecuting
        fprintf('[任务执行] 当前已有任务在执行中，等待完成...\n');
        return;
    end

    STATE.isExecuting = true;

    % 取出队首任务
    task = STATE.taskQueue{1};
    task.status = 'running';
    STATE.taskQueue{1} = task;

    fprintf('[任务执行] 开始执行任务 [%s]...\n', task.type);

    switch task.type
        case 'move_to'
            executeMoveTo(task);
        case 'move'
            executeMove(task);
        case 'catch'
            executeCatch(task);
        case 'put'
            executePut(task);
        otherwise
            fprintf('[任务执行] 未知任务类型: %s\n', task.type);
            task.status = 'failed';
            task.result = 'Unknown task type';
            STATE.taskQueue{1} = task;
    end

    % 执行完成后出队
    STATE.taskQueue(1) = [];
    STATE.isExecuting = false;

    fprintf('[任务执行] 任务 [%s] 已完成，队列剩余：%d\n', ...
        task.type, length(STATE.taskQueue));

    % 继续执行队列中后续任务
    if ~isempty(STATE.taskQueue)
        taskExecutor();
    end
end

%% --- move_to 任务实现 ---
function executeMoveTo(task)
    % executeMoveTo
    %   调用 runLArmIK_2D 的新增模式（传入步长数 ikStepCount），
    %   在迭代过程中每 ikStepCount 次迭代输出一次目标电机参数矢量，
    %   并反复调用硬件仿真数据传出接口进行电机参数传出。
    STATE = getState();
    params = STATE.params;

    % 配置 IK 参数：设定目标为任务终点
    params.X_target        = task.endPos;
    params.theta_end_target = task.endTheta;
    params.q_init          = STATE.currentMotorParams;
    if isempty(params.q_init) || all(params.q_init == 0)
        params.q_init = zeros(1, params.N);
    end

    ikStepCount = params.ikStepCount;
    fprintf('[move_to] 调用 runLArmIK_2D，步长数 = %d，目标 = [%.3f, %.3f, θ=%.3f]\n', ...
        ikStepCount, task.endPos(1), task.endPos(2), task.endTheta);

    % 调用 runLArmIK_2D 新增外部调用模式
    resultQ = runLArmIK_2D_External(params, ikStepCount, @onMotorStep);

    % 更新当前电机参数状态
    STATE.currentMotorParams = resultQ;
    fprintf('[move_to] 任务完成，最终电机参数 q = [%s]\n', num2str(resultQ, '%.4f '));

    % 嵌套回调：每 IK 步长输出时传至硬件仿真接口
    function onMotorStep(q_current, iter)
        % 传出当前电机参数到硬件仿真接口
        hardwareInterface('output', q_current);
        fprintf('[move_to] 步 %d — 电机参数已传出至硬件仿真\n', iter);
    end
end

%% --- move 任务实现 ---
function executeMove(task)
    % executeMove
    %   传出当前电机参数、运动方向与步长调用 runMove，然后传出结果至硬件仿真。
    %   runMove 尚未实现 — 此处预留接口。
    STATE = getState();
    params = STATE.params;

    currentQ = STATE.currentMotorParams;
    direction = task.direction;
    if isempty(direction)
        direction = params.moveDirection;
    end
    stepLen = task.stepLength;
    if isempty(stepLen)
        stepLen = params.moveStepLength;
    end

    fprintf('[move] 调用 runMove: 当前电机 q=[%s], 方向=[%.2f, %.2f], 步长=%.3f\n', ...
        num2str(currentQ, '%.4f '), direction(1), direction(2), stepLen);

    % TODO: 调用 runMove — 根据当前电机参数、运动方向与步长反解运动学参数所需变化
    % 当前为占位实现，返回原电机参数
    fprintf('[move] TODO: runMove 尚未实现，返回当前电机参数\n');
    targetQ = currentQ;  % 占位

    hardwareInterface('output', targetQ);
    STATE.currentMotorParams = targetQ;
end

%% --- catch 任务实现 ---
function executeCatch(task)
    % executeCatch
    %   按照预设参数传出目标电机参数。
    STATE = getState();
    params = STATE.params;
    targetQ = params.catchPresetQ;
    fprintf('[catch] 按预设参数传出电机参数: q = [%s]\n', num2str(targetQ, '%.4f '));
    hardwareInterface('output', targetQ);
    STATE.currentMotorParams = targetQ;
end

%% --- put 任务实现 ---
function executePut(task)
    % executePut
    %   按照预设参数传出目标电机参数。
    STATE = getState();
    params = STATE.params;
    targetQ = params.putPresetQ;
    fprintf('[put] 按预设参数传出电机参数: q = [%s]\n', num2str(targetQ, '%.4f '));
    hardwareInterface('output', targetQ);
    STATE.currentMotorParams = targetQ;
end

%% ================= 4. 硬件仿真接口 =================
function varargout = hardwareInterface(direction, data)
    % hardwareInterface
    %   外部硬件仿真的电机参数传入与传出接口。
    %
    %   传入模式：hardwareInterface('input')  — 返回当前电机参数矢量
    %   传出模式：hardwareInterface('output', q_vector) — 将目标电机参数传出
    %
    %   当前为仿真占位实现，实际需与硬件驱动对接。
    STATE = getState();

    switch direction
        case 'input'
            % 从硬件读入当前电机参数（仿真：返回持久化值）
            varargout{1} = STATE.currentMotorParams;
            fprintf('[硬件接口] 读入当前电机参数: q = [%s]\n', ...
                num2str(STATE.currentMotorParams, '%.4f '));

        case 'output'
            % 将目标电机参数传出到硬件（仿真：存入持久化状态）
            STATE.currentMotorParams = data(:)';
            fprintf('[硬件接口] 传出目标电机参数: q = [%s]\n', ...
                num2str(data(:)', '%.4f '));

        otherwise
            fprintf('[硬件接口] 未知操作: %s\n', direction);
    end
end

%% ================= 5. 参数修改模块 =================
function paramManager(varargin)
    % paramManager
    %   用于修改所有参数预设与任务需要的额外参数预设。
    %
    %   调用方式：
    %     paramManager('set', fieldName, value)          — 设置指定参数
    %     paramManager('setMultiple', structOfFields)    — 批量设置参数
    %     paramManager('get', fieldName)                 — 获取指定参数值
    %     paramManager('list')                           — 列出所有参数
    %     paramManager('reset')                          — 重置为默认值
    STATE = getState();

    if nargin == 0
        fprintf('[参数管理] 请指定操作: set / get / list / reset / setMultiple\n');
        fprintf('  示例: paramManager(''set'', ''N'', 8);\n');
        fprintf('  示例: paramManager(''get'', ''N'');\n');
        fprintf('  示例: paramManager(''list'');\n');
        fprintf('  示例: paramManager(''reset'');\n');
        fprintf('  示例: s.kappa=0.002; s.N=7; paramManager(''setMultiple'', s);\n');
        return;
    end

    op = varargin{1};

    switch op
        case 'set'
            if nargin < 3
                fprintf('[参数管理] 用法: paramManager(''set'', fieldName, value)\n');
                return;
            end
            fieldName = varargin{2};
            value = varargin{3};
            if isfield(STATE.params, fieldName)
                STATE.params.(fieldName) = value;
                fprintf('[参数管理] 参数 %s 已更新为: %s\n', fieldName, mat2str(value));
            else
                fprintf('[参数管理] 参数 %s 不存在，已新增并赋值\n', fieldName);
            end

        case 'setMultiple'
            if nargin < 2
                fprintf('[参数管理] 用法: paramManager(''setMultiple'', paramStruct)\n');
                return;
            end
            paramStruct = varargin{2};
            fields = fieldnames(paramStruct);
            for i = 1:length(fields)
                fn = fields{i};
                STATE.params.(fn) = paramStruct.(fn);
            end
            fprintf('[参数管理] 已批量更新 %d 个参数\n', length(fields));

        case 'get'
            if nargin < 2
                fprintf('[参数管理] 用法: paramManager(''get'', fieldName)\n');
                return;
            end
            fieldName = varargin{2};
            if isfield(STATE.params, fieldName)
                val = STATE.params.(fieldName);
                fprintf('[参数管理] %s = %s\n', fieldName, mat2str(val));
            else
                fprintf('[参数管理] 参数 %s 不存在\n', fieldName);
            end

        case 'list'
            fprintf('\n====== 当前参数预设列表 ======\n');
            fields = fieldnames(STATE.params);
            for i = 1:length(fields)
                fn = fields{i};
                val = STATE.params.(fn);
                if isnumeric(val) && numel(val) <= 20
                    fprintf('  %-20s = %s\n', fn, mat2str(val));
                elseif isnumeric(val)
                    fprintf('  %-20s = [%d elements]\n', fn, numel(val));
                else
                    fprintf('  %-20s = %s\n', fn, class(val));
                end
            end
            fprintf('==============================\n\n');

        case 'reset'
            STATE.params = initDefaultParams();
            fprintf('[参数管理] 参数已重置为默认值\n');

        otherwise
            fprintf('[参数管理] 未知操作: %s\n', op);
            fprintf('  支持: set, get, list, reset, setMultiple\n');
    end
end

%% ================= 辅助查询接口 =================
function q = getTaskQueue()
    % 返回当前任务队列（只读）
    STATE = getState();
    q = STATE.taskQueue;
end

function m = getMotorState()
    % 返回当前电机参数状态
    STATE = getState();
    m = STATE.currentMotorParams;
end

function p = getParams()
    % 返回当前全部参数预设
    STATE = getState();
    p = STATE.params;
end

%% ====================================================================
%% runLArmIK_2D 外部调用模式（新增）
%  在原 runLArmIK_2D 基础上包装，支持：
%   1. 每 ikStepCount 次迭代触发一次回调 (onStepCallback)
%   2. 完成后返回最终关节角度矢量
%  该函数作为顶层系统内部组件，对 runLArmIK_2D 的核心逻辑进行复用。
%% ====================================================================
function q_final = runLArmIK_2D_External(params, ikStepCount, onStepCallback)
    % runLArmIK_2D_External
    %   与 runLArmIK_2D 功能一致，但新增外部调用模式特性:
    %     - ikStepCount: 每 ikStepCount 次迭代输出一次目标电机参数
    %     - onStepCallback: 函数句柄 @(q_current, iter)，每次输出时调用
    %   返回: 最终关节角度 q_final (1×N)
    %
    %   说明：此模式关闭交互式选点和图形绘制，专用于自动化任务执行。

    %% 从结构体读取参数
    N               = params.N;
    L_seg           = params.L_seg;
    theta_end_target= params.theta_end_target;
    X_target        = params.X_target;
    rod_offset_arr  = params.rod_offset_arr;

    DH = zeros(N,4);
    DH(:,3) = L_seg;
    DH(:,2) = 0;
    DH(:,4) = 0;

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
    stall_counter   = 0;

    obs             = params.obs;
    rho0            = params.rho0;
    safe_margin     = params.safe_margin;

    lambdaM         = params.lambdaM;
    lambda_m        = params.lambda_m;
    max_iter        = params.max_iter;
    q               = params.q_init;

    % 若初始电机参数非空，覆盖 q_init
    STATE = getState();
    if ~isempty(STATE.currentMotorParams)
        q = STATE.currentMotorParams;
    end

    %% 初始化
    dq_norm_round = 0;
    avg_joint_err = 0;
    theta_curr = 0;
    err_ang = 0;

    %% 主迭代循环（无图形，自动运行）
    for iter = 1:max_iter
        [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr);
        X_curr = p_end;
        err_X = X_target - X_curr;
        dist_end = norm(err_X);
        if dist_end < lambda_m
            fprintf('  [IK_外部] 迭代 %d 收敛，到达目标！\n', iter);
            break;
        end

        theta_curr = getEndEffectorAngle_L(q, DH, rod_offset_arr);
        err_ang = theta_end_target - theta_curr;
        gamma_ang = gamma_ang_base + (gamma_ang_peak - gamma_ang_base) * exp(-dist_end^2 / (2*sigma_weight^2));
        J_ang = jacEndAngle_L(q, DH, rod_offset_arr);

        s_base = min(lambdaM, dist_end);
        if dist_end > 0.2
            speed_coeff = 1.8;
        elseif dist_end > 0.05
            speed_coeff = 1.2;
        else
            speed_coeff = 1.0;
        end
        s = s_base * speed_coeff;
        u = err_X / dist_end;
        u = u(:);

        [g_all, dg_all] = obsSegGradient(q, DH, obs, rho0+safe_margin, p_all, rod_offset_arr);
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

        J = planarJac_L(q, DH, rod_offset_arr);

        dq = solveQP_Damped(J, Sigma, s, u, g_all, dg_all, ...
            rho0+safe_margin, dq_step_min, dq_step_max, ...
            q, q_min, q_max, lambda_damp, gamma_soft, ...
            J_ang, err_ang, gamma_ang);

        q_test = q + dq';
        [p_test_all,~] = planarFK_L(q_test, DH, rod_offset_arr);
        [g_check,~] = obsSegGradient(q_test, DH, obs, rho0, p_test_all, rod_offset_arr);
        if ~isempty(g_check) && min(g_check) < rho0
            dq = dq * 0.5;
        end

        dq_norm = norm(dq);
        if dq_norm < dq_stall_thresh
            stall_counter = stall_counter + 1;
            if stall_counter >= stall_count_max
                fprintf('  [IK_外部] 迭代 %d 停滞，提前终止\n', iter);
                break;
            end
        else
            stall_counter = 0;
        end

        dq_round = motorStepRounding(dq, kappa);
        dq_norm_round = norm(dq_round);
        if dq_norm_round < 1e-12
            fprintf('  [IK_外部] 迭代 %d 电机增量小于最小步长，终止\n', iter);
            break;
        end
        q = q + dq_round';

        %% ★ 新增外部调用模式核心功能：每 ikStepCount 次迭代回调一次
        if ~isempty(onStepCallback) && mod(iter, ikStepCount) == 0
            onStepCallback(q, iter);
        end
    end

    q_final = q;
    fprintf('  [IK_外部] 结束，最终 q = [%s]\n', num2str(q_final, '%.4f '));
end

%% ================= runLArmIK_2D 底层函数副本（供外部模式复用） =================
%% 这些函数是 runLArmIK_2D.m 中对应子函数的副本，确保外部模式独立可用。
%% 若 runLArmIK_2D.m 在相同路径下，可删除此部分并直接调用其子函数。

function [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr)
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

function p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr)
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

function J = planarJac_L(q, DH, rod_offset_arr)
    n = length(q);
    [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr);
    p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr);
    J = zeros(2,n);
    xn = p_end(1); yn = p_end(2);
    for i=1:n
        xi = p_nodes(i,1); yi = p_nodes(i,2);
        J(1,i) = -(yn - yi);
        J(2,i) = xn - xi;
    end
end

function Jp = planarJacPoint_L(q, DH, idx, rod_offset_arr)
    n = length(q);
    p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr);
    [p_all,~] = planarFK_L(q, DH, rod_offset_arr);
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
    [p_all,~] = planarFK_L(q, DH, rod_offset_arr);
    x0 = p_all(end-1,1); y0 = p_all(end-1,2);
    x1 = p_all(end-2,1); y1 = p_all(end-2,2);
    dx = x1 - x0;
    dy = y1 - y0;
    theta_end = atan2(dy, dx);
end

function J_ang = jacEndAngle_L(q, DH, rod_offset_arr)
    n = length(q);
    [p_all,~] = planarFK_L(q, DH, rod_offset_arr);
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

function sigma2 = errVar(x, m, tau, sig0)
    sigma2 = sig0^2 * exp( -(x - m).^2 / (2*tau^2) );
end

function [g_total, dg_total] = obsSegGradient(q, DH, obs, rho0, p_all, rod_offset_arr)
    g_total = [];
    dg_total = [];
    if isempty(obs)
        return;
    end
    n_seg = size(p_all,1)-1;
    n_obs = size(obs,1);
    filter_dist = 2 * rho0;
    for o = 1:n_obs
        xo = obs(o,1); yo = obs(o,2);
        ro = obs(o,3);
        for seg = 1:n_seg
            p0 = p_all(seg,:); p1 = p_all(seg+1,:);
            [dist, grad_dist] = segCircleDistGrad(p0,p1,xo,yo,ro,q,DH,seg,rod_offset_arr);
            if dist > filter_dist
                continue;
            end
            g_total = [g_total; dist];
            dg_total = [dg_total; grad_dist];
        end
    end
end

function [dist, dg] = segCircleDistGrad(p0, p1, xo, yo, ro, q, DH, seg_idx, rod_offset_arr)
    dx_seg = p1(1)-p0(1);
    dy_seg = p1(2)-p0(2);
    t = clamp(((xo-p0(1))*dx_seg + (yo-p0(2))*dy_seg)/(dx_seg^2+dy_seg^2),0,1);
    p_near = p0 + t*[dx_seg, dy_seg];
    dx = p_near(1)-xo; dy = p_near(2)-yo;
    dist = sqrt(dx^2 + dy^2);
    n = length(q);
    dg = zeros(1,n);
    J0 = planarJacPoint_L(q, DH, seg_idx, rod_offset_arr);
    J1 = planarJacPoint_L(q, DH, seg_idx+1, rod_offset_arr);
    grad_pnear = (1-t)*[-dx/dist, -dy/dist]*J0 + t*[-dx/dist, -dy/dist]*J1;
    dg = grad_pnear;
end

function val = clamp(x, low, high)
    val = min(max(x, low), high);
end

function dq = solveQP_Damped(J, Sigma, s, u, g, dg, rho0, dq_min, dq_max, ...
    q, q_min, q_max, lambda_damp, gamma_soft, J_ang, err_ang, gamma_ang)
    n = size(J,2);
    invSigma = inv(Sigma);
    H = 2 * invSigma ...
        + 2 * lambda_damp^2 * eye(n) ...
        + 2 * gamma_soft * (J'*J) ...
        + 2 * gamma_ang * (J_ang' * J_ang);
    f = -2 * gamma_soft * s * J' * u ...
        - 2 * gamma_ang * err_ang * J_ang';

    A_obs = -dg;
    b_obs = -(rho0 - g);

    A_qmin = -eye(n);
    b_qmin = -(q_min' - q');
    A_qmax = eye(n);
    b_qmax = q_max' - q';
    A_joint = [A_qmin; A_qmax];
    b_joint = [b_qmin; b_qmax];

    Aineq = [A_obs; A_joint];
    bineq = [b_obs; b_joint];

    lb = dq_min * ones(n,1);
    ub = dq_max * ones(n,1);

    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    dq = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],opts);
    if isempty(dq)
        H = 2 * invSigma ...
            + 2 * (lambda_damp*2)^2 * eye(n) ...
            + 2 * gamma_soft * (J'*J) ...
            + 2 * gamma_ang * (J_ang' * J_ang);
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
            dq_round(i) = 0;
        elseif abs_val < 2*kappa
            dq_round(i) = sign(val) * kappa;
        else
            dq_round(i) = val;
        end
    end
end