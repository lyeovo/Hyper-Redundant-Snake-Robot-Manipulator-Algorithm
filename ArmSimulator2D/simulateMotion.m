function info = simulateMotion(model, method, q0, target, varargin)
%simulateMotion 接口2：统一运动模拟入口（任务执行与仿真环节）
%   info = simulateMotion(model, method, q0, target, varargin)
%
%   输入：
%     model  : createArmModel 返回的模型对象
%     method : 'momentum' | 'sa' | 'rrt' | 'prm' | 'rl' | 'auto'
%              （M1 提供 'momentum'；其余方法 M2/M3/M5 接入）
%     q0     : 初始关节角 [1×N]，或初始末端位姿 [x,y,θ]（内部反解，失败 error_code=5）
%     target : 目标位姿 [x,y,θ]，覆盖 model.cfg 默认目标（同一模型多次运动）
%     varargin: 'Snapshot', m   快照间隔（每 m 步一行，电机指令节拍）
%               'OnStep', @cb  每 m 步回调 cb(q, iter, dist_end)（电控实时通道）
%               'Cancel', @cb  取消检查 @() bool（每步触发，true 中断）
%               'MaxIter', n   覆盖最大迭代
%               'Verbose', b   打印开关
%
%   输出 info（下游契约 motorCmd 的核心，见《电控对接说明》）：
%     .q_snapshot [K×N]  .t_seq [1×K]  .V_hist  .q_final
%     .success .error_code .vel_ok .safety_ok .method_used .stats .meta
    cfg = model.cfg;
    if nargin < 4, error('simulateMotion:args', '需要 model, method, q0, target'); end
    if nargin >= 5, opts = parseOpts(varargin); else, opts = struct(); end
    if ~isfield(opts, 'snapshot_m'), opts.snapshot_m = cfg.snapshot_m; end

    % ---- 目标覆盖 ----
    m2 = model;
    m2.cfg.X_target = target(1:2);
    m2.cfg.theta_target = target(3);

    % ---- 初始构型：关节角 或 末端位姿（反解） ----
    if numel(q0) == cfg.N
        q_start = q0(:)';
    elseif numel(q0) == 3
        q_start = inverseKinPose(m2, q0, struct('verbose', false));
        if isempty(q_start)
            info = emptyInfo();
            info.success = false;  info.error_code = 5;
            info.method_used = method;  info.meta = metaOf(model, method);
            return;
        end
    else
        error('simulateMotion:q0', 'q0 应为 1×%d 关节角或 1×3 末端位姿', cfg.N);
    end

    % ---- 方法分派 ----
    if isfield(opts, 'Cancel'),  opts.isCancel = opts.Cancel; end   % 统一键名
    mopts = opts;                                                   % 全量透传（方法用 of() 自取）
    if ~isfield(mopts, 'snapshot_m'), mopts.snapshot_m = opts.snapshot_m; end

    switch method
        case 'momentum'
            info = method_momentum(m2, q_start, target, mopts);
        case 'sa'
            info = method_sa(m2, q_start, target, mopts);
        case 'rrt'
            info = method_rrt(m2, q_start, target, mopts);
        case 'rrtstar'
            info = method_rrtstar(m2, q_start, target, mopts);
        case 'graph'
            info = method_graph(m2, q_start, target, mopts);
        case 'prm'
            info = method_prm(m2, q_start, target, mopts);
        case 'auto'
            info = method_auto(m2, q_start, target, mopts);
        case 'multilayer'
            info = method_multilayer(m2, q_start, target, mopts);
        case 'rl'
            % 默认加载离线训练策略（RLPolicy.mat 存在且 N 匹配），否则回退任务内现训
            info = [];
            if ~isfield(mopts, 'theta0') && ~isfield(mopts, 'train')
                pol = 'RLPolicy.mat';
                if isfield(opts, 'rl_policy') && ~isempty(opts.rl_policy), pol = opts.rl_policy; end
                if exist(pol, 'file') == 2
                    S = load(pol);
                    if isfield(S, 'theta') && size(S.theta, 1) == cfg.N
                        mopts.train = false;
                        mopts.theta0 = S.theta;
                        if isfield(S, 'max_roll') && ~isempty(S.max_roll)
                            mopts.max_rollout = S.max_roll;
                        end
                        info = method_rl(m2, q_start, target, mopts);
                        info.stats.policy_file = pol;
                    end
                end
            end
            if isempty(info)
                info = method_rl(m2, q_start, target, mopts);
            end
        otherwise
            error('simulateMotion:method', '未知方法 %s', method);
    end

    % ---- 后处理：vel_ok / safety_ok / meta ----
    info.vel_ok = checkVel(info.q_snapshot, opts.snapshot_m, cfg.dq_max);
    info.safety_ok = checkSafety(model, info.q_snapshot, cfg.rho0);
    % 终态安全校验：q_final 侵入障碍安全区 → 标记碰撞失败（保护下游导出/电控）
    g_fin = obsDistAll(model, info.q_final);
    if ~isempty(g_fin) && min(g_fin) < cfg.rho0
        if ~isfield(info, 'stats') || ~isstruct(info.stats), info.stats = struct(); end
        info.success = false;
        info.error_code = 9;                  % COLLISION
        info.stats.collision = true;
    end
    if ~strcmp(method, 'auto')
        info.method_used = method;      % auto 保留调度链内部记录
    end
    info.meta = metaOf(model, method);
end

%% ---------- 内部工具 ----------
function opts = parseOpts(v)
% 'Key', value 对 → struct
    opts = struct();
    for i = 1:2:numel(v)
        opts.(v{i}) = v{i+1};
    end
end

function ok = checkVel(q_snap, m, dq_max)
    if size(q_snap, 1) < 2, ok = true; return; end
    d = max(max(abs(diff(q_snap, 1, 1))));
    ok = d <= m * dq_max + 1e-9;
end

function ok = checkSafety(model, q_snap, rho0)
    if isempty(q_snap), ok = true; return; end
    cfg = model.cfg;
    if isempty(cfg.obstacles.rects) && isempty(cfg.obstacles.circles)
        ok = true; return;
    end
    ok = true;
    for i = 1:size(q_snap, 1)
        [g_all, ~] = obsDistGradAll(model, q_snap(i,:));
        if ~isempty(g_all) && min(g_all) < rho0 - 1e-9
            ok = false; return;
        end
        % 快照行间线性插值 3 点再查（轨迹级安全）
        if i < size(q_snap, 1)
            for t = 1:2
                qi = q_snap(i,:) + (t/3) * (q_snap(i+1,:) - q_snap(i,:));
                [g2, ~] = obsDistGradAll(model, qi);
                if ~isempty(g2) && min(g2) < rho0 - 1e-9
                    ok = false; return;
                end
            end
        end
    end
end

function m = metaOf(model, method)
    m = struct('command_id', '', 'task_type', '', 'method', method, ...
        'N', model.cfg.N, 'L_seg', model.cfg.L_seg(1), ...
        'timestamp', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
end

function info = emptyInfo()
    info = struct('q_snapshot', [], 't_seq', [], 'V_hist', [], 'q_final', [], ...
        'success', false, 'error_code', -1, 'vel_ok', false, 'safety_ok', false, ...
        'method_used', '', 'stats', struct(), 'meta', struct());
end
