function info = taskExecute(model, cmd, opts)
%taskExecute 执行一个 TaskCommand（任务列表的原子单元，方案 §1.1/§1.2）
%   info = taskExecute(model, cmd, opts)
%   model: createArmModel 输出
%   cmd  : TaskCommand struct（jsondecode 自 data/outbox/CMD-*.json）
%   opts : .inbox（TaskStatus 回写目录，空则不回写）
%          .method（默认 'auto'） .snapshot_m（默认 10）
%          .approach_dist（默认 0.15） .estop（@() bool 急停检查）
%
%   流程：安全校验 → ACCEPTED → PLANNING → 逐段 simulateMotion（EXECUTING 更新
%   progress/joint_positions）→ COMPLETED / FAILED / REJECTED / ESTOP_TRIGGERED
%   返回 info：.success .error_code .seg_infos .motor_cmd .status
    cfg = model.cfg;
    if nargin < 3 || isempty(opts), opts = struct(); end
    inbox = optget(opts, 'inbox', '');
    method = optget(opts, 'method', 'auto');
    snapshot_m = optget(opts, 'snapshot_m', cfg.snapshot_m);
    approach_dist = optget(opts, 'approach_dist', 0.15);
    estop = optget(opts, 'estop', []);

    % ---- 安全校验（消费 TaskCommand.safety 字段） ----
    if isfield(cmd, 'safety') && ~isempty(cmd.safety)
        if isfield(cmd.safety, 'estop_active') && cmd.safety.estop_active
            taskWriteStatus(inbox, cmd, 'REJECTED', 'Msg', '急停激活');
            info = mkInfo(false, 6, 'REJECTED'); return;
        end
        if isfield(cmd.safety, 'allow_execute') && ~isempty(cmd.safety.allow_execute) ...
                && ~cmd.safety.allow_execute
            taskWriteStatus(inbox, cmd, 'REJECTED', 'Msg', '禁止执行（allow_execute=false）');
            info = mkInfo(false, 6, 'REJECTED'); return;
        end
    end
    % 置信度门槛
    if isfield(cmd, 'selected_target') && isfield(cmd.selected_target, 'confidence')
        if cmd.selected_target.confidence < 0.5
            taskWriteStatus(inbox, cmd, 'REJECTED', 'Msg', '目标置信度不足');
            info = mkInfo(false, 6, 'REJECTED'); return;
        end
    end

    % ---- 控制类指令（不做运动展开，直接回写控制状态） ----
    if strcmp(cmd.command_type, 'emergency_stop')
        taskWriteStatus(inbox, cmd, 'ESTOP_TRIGGERED', 'Msg', '急停');
        info = mkInfo(false, 6, 'ESTOP_TRIGGERED'); return;
    end
    if strcmp(cmd.command_type, 'cancel_task')
        taskWriteStatus(inbox, cmd, 'CANCELED', 'Msg', '任务取消');
        info = mkInfo(false, 0, 'CANCELED'); return;
    end

    taskWriteStatus(inbox, cmd, 'ACCEPTED');
    taskWriteStatus(inbox, cmd, 'PLANNING');

    % ---- 任务展开 ----
    try
        segs = taskToSegments(cmd, model, approach_dist);
    catch e
        taskWriteStatus(inbox, cmd, 'FAILED', 'Msg', e.message);
        info = mkInfo(false, 7, 'FAILED', e.message); return;
    end

    % ---- 逐段执行 ----
    seg_infos = repmat(struct('target', [], 'gripper', [], 'name', '', 'info', struct()), 1, numel(segs));
    q = model.cfg.q_init;
    n_seg = numel(segs);
    % 预检：段目标是否被障碍吞没（距障碍 < rho0，安全约束下物理不可达）→ 明确报错
    for s = 1:n_seg
        if isempty(segs(s).target), continue; end   % 关节/相对段目标在执行时解析，此处跳过
        d_tgt = inf;
        cfgm = model.cfg;
        for ci = 1:size(cfgm.obstacles.circles, 1)
            d_tgt = min(d_tgt, norm(segs(s).target(1:2) - cfgm.obstacles.circles(ci,1:2)) - cfgm.obstacles.circles(ci,3));
        end
        for ri = 1:size(cfgm.obstacles.rects, 1)
            r = cfgm.obstacles.rects(ri,:);
            ct = cos(r(3)); st = sin(r(3));
            lx = (segs(s).target(1)-r(1))*ct + (segs(s).target(2)-r(2))*st;
            ly = -(segs(s).target(1)-r(1))*st + (segs(s).target(2)-r(2))*ct;
            d_tgt = min(d_tgt, max(abs(lx)-r(4)/2, abs(ly)-r(5)/2));
        end
        if d_tgt < cfgm.rho0
            msg = sprintf('段 %d %s 目标被障碍吞没（距障碍 %.3f m < 安全距 %.3f m），任务不可达', ...
                s, segs(s).name, d_tgt, cfgm.rho0);
            taskWriteStatus(inbox, cmd, 'FAILED', 'Msg', msg);
            info = mkInfo(false, 10, 'FAILED', msg); info.seg_infos = seg_infos;
            return;
        end
    end
    for s = 1:n_seg
        if ~isempty(estop) && estop()
            taskWriteStatus(inbox, cmd, 'ESTOP_TRIGGERED', 'Progress', (s-1)/n_seg);
            info = mkInfo(false, 6, 'ESTOP_TRIGGERED'); info.seg_infos = seg_infos;
            return;
        end
        taskWriteStatus(inbox, cmd, 'EXECUTING', 'Progress', (s-1)/n_seg, ...
            'Step', segs(s).name, 'Msg', sprintf('段 %d/%d: %s', s, n_seg, segs(s).name));
        try
            % 段级重试（采样类方法带随机性，最多 3 次尝试）
            si = [];
            if strcmp(segs(s).kind, 'joint_delta') || strcmp(segs(s).kind, 'joint_abs')
                si = execJointSeg(model, q, segs(s), snapshot_m);   % 关节空间段（直接驱动）
            elseif strcmp(segs(s).kind, 'reset')
                si = execResetSeg(model, q, snapshot_m);            % 复位段：全关节平滑回零（朝前直伸，不缩圈）
            else
                tgt = resolveSegTarget(model, q, segs(s));          % 末端/相对/旋转段 → [x,y,θ]
                for attempt = 1:3
                    si = simulateMotion(model, method, q, tgt, 'Snapshot', snapshot_m);
                    if si.success, break; end
                end
            end
        catch e
            % 段求解异常：记录失败信息并终止任务（避免半状态任务）
            seg_infos(s).target = segs(s).target;
            seg_infos(s).gripper = segs(s).gripper;
            seg_infos(s).name = segs(s).name;
            seg_infos(s).info = struct('success', false, 'error_code', 8, ...
                'q_snapshot', [], 't_seq', [], 'q_final', q, 'dist_end', Inf, ...
                'err_ang', Inf, 'message', e.message);
            taskWriteStatus(inbox, cmd, 'FAILED', 'Progress', s/n_seg, ...
                'Step', segs(s).name, 'Msg', sprintf('段 %d 求解异常: %s', s, e.message));
            info = mkInfo(false, 8, 'FAILED', e.message); info.seg_infos = seg_infos;
            return;
        end
        seg_infos(s).target = segs(s).target;
        seg_infos(s).gripper = segs(s).gripper;
        seg_infos(s).name = segs(s).name;
        seg_infos(s).info = si;
        q = si.q_final;
        if ~si.success
            taskWriteStatus(inbox, cmd, 'FAILED', 'Progress', s/n_seg, ...
                'Step', segs(s).name, 'Msg', sprintf('段 %d 失败 error_code=%d', s, si.error_code));
            info = mkInfo(false, si.error_code, 'FAILED'); info.seg_infos = seg_infos;
            return;
        end
    end

    % ---- 完成：组装 motorCmd（逐段拼接轨迹 + 夹爪事件序列） ----
    q_seq = zeros(0, cfg.N);  t_seq = zeros(1, 0);  gripper_seq = zeros(1, 0);
    t_off = 0;
    for s = 1:n_seg
        qs = seg_infos(s).info.q_snapshot;
        if isempty(qs), continue; end
        Ks = size(qs, 1);
        ts = seg_infos(s).info.t_seq;
        if isempty(ts), ts = 1:Ks; end
        ts = ts(:)' - ts(1);                 % 段内相对时间归零
        q_seq = [q_seq; qs]; %#ok<AGROW>
        t_seq = [t_seq, t_off + ts]; %#ok<AGROW>
        gs = zeros(1, Ks);  gs(end) = seg_infos(s).gripper;   % 夹爪动作在段末快照执行（0=保持 1=开 2=合）
        gripper_seq = [gripper_seq, gs]; %#ok<AGROW>
        t_off = t_off + ts(end) + 1;        % 下一段时间偏移（+1 保证严格递增）
    end
    motor_cmd = struct('q_seq', q_seq, 't_seq', t_seq, 'gripper_seq', gripper_seq, ...
        'success', true, 'error_code', 0, 'q_final', q);

    % ---- 慢放与逐帧运动回放（供 UI 观察连续轨迹动效） ----
    playback_delay = optget(opts, 'playback_delay', 0.04);
    if playback_delay > 0 && ~isempty(q_seq) && size(q_seq, 1) > 1
        K = size(q_seq, 1);
        step_stride = max(1, round(K / 30));
        for k = 1:step_stride:K
            taskWriteStatus(inbox, cmd, 'EXECUTING', 'Progress', k/K, ...
                'Step', 'TRAJECTORY_PLAYBACK', 'Msg', sprintf('机械臂轨迹慢放中 (%d/%d)', k, K), ...
                'JointPos', q_seq(k,:));
            pause(playback_delay);
        end
    end

    taskWriteStatus(inbox, cmd, 'COMPLETED', 'Progress', 1, 'Msg', '任务完成', 'JointPos', q);

    info = mkInfo(true, 0, 'COMPLETED');
    info.seg_infos = seg_infos;
    info.motor_cmd = motor_cmd;
end

function info = mkInfo(success, code, status, msg)
    if nargin < 4, msg = ''; end
    info = struct('success', success, 'error_code', code, 'status', status, ...
        'message', msg, 'seg_infos', [], 'motor_cmd', []);
end

function tgt = resolveSegTarget(model, q, seg)
%resolveSegTarget 把段解析为末端目标 [x,y,θ]（ee / ee_relative / ee_rotate）
    cfg = model.cfg;
    switch seg.kind
        case 'ee'
            tgt = seg.target;
        case 'ee_relative'          % 当前末端沿 seg.dir 方向 seg.dist 米
            [~, pe] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
            th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
            tgt = [pe(1) + seg.dist*cos(seg.dir), pe(2) + seg.dist*sin(seg.dir), th];
        case 'ee_rotate'            % 末端位置不动，θ 转 seg.alpha
            [~, pe] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
            th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
            tgt = [pe(1), pe(2), wrapAngle(th + seg.alpha)];
        otherwise
            tgt = seg.target;
    end
end

function si = execJointSeg(model, q, seg, snapshot_m)
%execJointSeg 关节空间段：第 n 关节增量(joint_delta)或连杆朝向(joint_abs)
%   生成 q→q' 的线性短轨迹作为该段快照（成功即达，无需 IK）
    cfg = model.cfg;  N = cfg.N;  n = max(1, min(N, seg.joint));
    if strcmp(seg.kind, 'joint_abs')
        cur = sum(q(1:n));                        % 平面链：连杆 n 绝对角 = 前 n 关节角求和
        dq = wrapAngle(seg.angle - cur);
    else
        dq = seg.delta;
    end
    qn = q;  qn(n) = max(cfg.q_min(n), min(cfg.q_max(n), q(n) + dq));
    m = max(2, min(20, ceil(snapshot_m*3)));
    xs = linspace(0, 1, m);
    Q = repmat(q, m, 1) + (qn - q).*xs(:);
    si = struct('q_snapshot', Q, 't_seq', (0:m-1), 'q_final', qn, ...
        'success', true, 'error_code', 0, 'dist_end', 0, 'err_ang', 0, ...
        'vel_ok', true, 'safety_ok', true, 'converged', true, 'method_used', 'joint');
end


function si = execResetSeg(model, q, snapshot_m)
%execResetSeg 复位段：所有关节平滑回零（伸直朝前，不缩成圈）
    cfg = model.cfg;
    qn = zeros(1, cfg.N);
    m = max(2, min(20, ceil(snapshot_m*3)));
    xs = linspace(0, 1, m);
    Q = repmat(q, m, 1) + (qn - q).*xs(:);
    si = struct('q_snapshot', Q, 't_seq', (0:m-1), 'q_final', qn, ...
        'success', true, 'error_code', 0, 'dist_end', 0, 'err_ang', 0, ...
        'vel_ok', true, 'safety_ok', true, 'converged', true, 'method_used', 'reset');
end
