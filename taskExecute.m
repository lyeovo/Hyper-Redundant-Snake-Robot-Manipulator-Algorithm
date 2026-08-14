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
    inbox = getopt(opts, 'inbox', '');
    method = getopt(opts, 'method', 'auto');
    snapshot_m = getopt(opts, 'snapshot_m', cfg.snapshot_m);
    approach_dist = getopt(opts, 'approach_dist', 0.15);
    estop = getopt(opts, 'estop', []);

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
            for attempt = 1:3
                si = simulateMotion(model, method, q, segs(s).target, 'Snapshot', snapshot_m);
                if si.success, break; end
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
    taskWriteStatus(inbox, cmd, 'COMPLETED', 'Progress', 1, 'Msg', '任务完成', 'JointPos', q);

    info = mkInfo(true, 0, 'COMPLETED');
    info.seg_infos = seg_infos;
    info.motor_cmd = motor_cmd;
end

function v = getopt(opts, field, default)
    if isfield(opts, field) && ~isempty(opts.(field))
        v = opts.(field);
    else
        v = default;
    end
end

function info = mkInfo(success, code, status, msg)
    if nargin < 4, msg = ''; end
    info = struct('success', success, 'error_code', code, 'status', status, ...
        'message', msg, 'seg_infos', [], 'motor_cmd', []);
end
