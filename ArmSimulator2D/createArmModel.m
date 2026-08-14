function model = createArmModel(params)
%createArmModel 接口1：构造机械臂运动模型（纯函数，无副作用）
%   model = createArmModel(params)
%   params 字段缺省时用 modelDefaults 默认值（缺失不崩溃）；
%   兼容旧字段：obs=[x,y,r;...] 与 obs_lines={[x1,y1;x2,y2],...} 自动迁移到 obstacles
%   （obs→circles；obs_lines→薄矩形，宽度=2·(rho0+safe_margin)）
%
%   返回 model：.cfg（填充完毕的全部参数）、.DH、.meta
    def = modelDefaults();
    if nargin < 1 || isempty(params), params = struct(); end
    cfg = struct();

    % --- 机械臂几何 ---
    cfg.N = gv(params, 'N', def.N);
    cfg.L_seg = gv(params, 'L_seg', def.L_seg);
    if isscalar(cfg.L_seg)
        cfg.L_seg = cfg.L_seg * ones(1, cfg.N);
    elseif numel(cfg.L_seg) ~= cfg.N
        error('createArmModel:len', 'L_seg 长度应为标量或 %d 维向量，实际 %d', cfg.N, numel(cfg.L_seg));
    end
    cfg.q_min = repParam(gv(params, 'q_min', def.q_min), cfg.N);
    cfg.q_max = repParam(gv(params, 'q_max', def.q_max), cfg.N);
    cfg.rod_offset_arr = gv(params, 'rod_offset_arr', []);
    if isempty(cfg.rod_offset_arr), cfg.rod_offset_arr = zeros(1, cfg.N); end
    cfg.q_init = gv(params, 'q_init', []);
    if isempty(cfg.q_init), cfg.q_init = zeros(1, cfg.N); end
    if numel(cfg.q_init) ~= cfg.N, cfg.q_init = zeros(1, cfg.N); end

    % --- 任务目标 ---
    cfg.X_target = gv(params, 'X_target', def.X_target);
    cfg.theta_target = gv(params, 'theta_target', def.theta_target);

    % --- 权重 / 屏障 ---
    cfg.w_pos = gv(params, 'w_pos', def.w_pos);
    cfg.w_ang = gv(params, 'w_ang', def.w_ang);
    cfg.w_obs = gv(params, 'w_obs', def.w_obs);
    cfg.w_var = gv(params, 'w_var', def.w_var);
    cfg.w_acc = gv(params, 'w_acc', def.w_acc);
    cfg.rho0 = gv(params, 'rho0', def.rho0);
    cfg.safe_margin = gv(params, 'safe_margin', def.safe_margin);
    cfg.barrier_C = gv(params, 'barrier_C', def.barrier_C);
    cfg.g_min = gv(params, 'g_min', def.g_min);
    cfg.barrier_range = gv(params, 'barrier_range', def.barrier_range);
    cfg.d_safe = cfg.rho0 + cfg.safe_margin;

    % --- 障碍（新结构 + 旧字段迁移） ---
    obs = struct('rects', [], 'circles', []);
    if isfield(params, 'obstacles') && ~isempty(params.obstacles)
        if isfield(params.obstacles, 'rects') && ~isempty(params.obstacles.rects)
            obs.rects = params.obstacles.rects;
        end
        if isfield(params.obstacles, 'circles') && ~isempty(params.obstacles.circles)
            obs.circles = params.obstacles.circles;
        end
    end
    if isfield(params, 'obs') && ~isempty(params.obs)      % 旧圆形障碍
        obs.circles = [obs.circles; params.obs];
    end
    if isfield(params, 'obs_lines') && ~isempty(params.obs_lines)  % 旧线段障碍 → 薄矩形
        hw = cfg.rho0 + cfg.safe_margin;                   % 半宽
        lines = params.obs_lines;
        if ~iscell(lines), lines = {lines}; end
        for li = 1:numel(lines)
            ln = lines{li};
            dxy = ln(2,:) - ln(1,:);
            len = norm(dxy);
            th = atan2(dxy(2), dxy(1));
            if len < 1e-12, len = 1e-6; end
            obs.rects(end+1, :) = [(ln(1,1)+ln(2,1))/2, (ln(1,2)+ln(2,2))/2, th, len, 2*hw]; %#ok<AGROW>
        end
    end
    cfg.obstacles = obs;
    cfg.obs_mode = gv(params, 'obs_mode', def.obs_mode);

    % --- 迭代 / 动量 ---
    cfg.max_iter = gv(params, 'max_iter', def.max_iter);
    cfg.dq_max = gv(params, 'dq_max', def.dq_max);
    cfg.tol_pos = gv(params, 'tol_pos', def.tol_pos);
    cfg.tol_ang = gv(params, 'tol_ang', def.tol_ang);
    cfg.momentum_beta = gv(params, 'momentum_beta', def.momentum_beta);
    cfg.dt_base = gv(params, 'dt_base', def.dt_base);
    cfg.rho_critical = gv(params, 'rho_critical', def.rho_critical);

    % --- RRT / SA / PRM / 局部最优检测 ---
    cfg.rrt_max_samples = gv(params, 'rrt_max_samples', def.rrt_max_samples);
    cfg.rrt_max_step = gv(params, 'rrt_max_step', def.rrt_max_step);
    cfg.rrt_goal_eps = gv(params, 'rrt_goal_eps', def.rrt_goal_eps);
    cfg.rrt_goal_ang = gv(params, 'rrt_goal_ang', def.rrt_goal_ang);
    cfg.sa_max_iter = gv(params, 'sa_max_iter', def.sa_max_iter);
    cfg.sa_alpha = gv(params, 'sa_alpha', def.sa_alpha);
    cfg.sa_sigma0 = gv(params, 'sa_sigma0', def.sa_sigma0);
    cfg.prm_n_nodes = gv(params, 'prm_n_nodes', def.prm_n_nodes);
    cfg.prm_gamma = gv(params, 'prm_gamma', def.prm_gamma);
    cfg.localmin_tol_grad = gv(params, 'localmin_tol_grad', def.localmin_tol_grad);

    % --- 关节先验 ---
    cfg.m_arr = gv(params, 'm_arr', []);
    if isempty(cfg.m_arr), cfg.m_arr = zeros(1, cfg.N); end
    cfg.sigma2_arr = gv(params, 'sigma2_arr', []);
    if isempty(cfg.sigma2_arr), cfg.sigma2_arr = ones(1, cfg.N); end

    % --- 输出 / 调试 ---
    cfg.snapshot_m = gv(params, 'snapshot_m', def.snapshot_m);
    if isfield(params, 'debug') && isstruct(params.debug)
        cfg.debug.grad_check = gv(params.debug, 'grad_check', def.debug.grad_check);
    else
        cfg.debug = def.debug;
    end
    % --- 误差模型 ---
    e_def = def.error;
    if isfield(params, 'error') && isstruct(params.error)
        cfg.error.on = gv(params.error, 'on', e_def.on);
        cfg.error.sigma_motor = gv(params.error, 'sigma_motor', e_def.sigma_motor);
        cfg.error.backlash = gv(params.error, 'backlash', e_def.backlash);
        cfg.error.kappa = gv(params.error, 'kappa', e_def.kappa);
    else
        cfg.error = e_def;
    end

    model.cfg = cfg;
    model.DH = zeros(cfg.N, 4);
    model.DH(:,3) = cfg.L_seg;
    model.meta = struct('source', 'ArmSimulator2D', ...
        'created', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
end

function v = gv(s, field, default)
% gv 字段读取：存在且非空则用之，否则默认值
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end

function v = repParam(x, N)
% repParam 标量 → 1×N 向量；向量校验长度
    if isscalar(x)
        v = x * ones(1, N);
    else
        v = x(:)';
        if numel(v) ~= N
            error('createArmModel:len', '参数长度应为标量或 %d 维向量，实际 %d', N, numel(v));
        end
    end
end
