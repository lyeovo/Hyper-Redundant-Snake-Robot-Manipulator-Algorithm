function P = motorCmdPayload(motor_cmd)
%motorCmdPayload 规整 motor_cmd → 可 jsonencode 的载荷
%   P = motorCmdPayload(motor_cmd)
%   输入可为 taskExecute 的 .motor_cmd（含 q_seq）或 simulateMotion 的 info（q_snapshot）
%   输出 P：q_seq / t_seq / gripper_seq / dq_max / vel_ok / safety_ok / success / error_code / q_final / meta
    if isfield(motor_cmd,'q_seq') && ~isempty(motor_cmd.q_seq)
        q = motor_cmd.q_seq;  tseq = motor_cmd.t_seq;  g = motor_cmd.gripper_seq;
    elseif isfield(motor_cmd,'q_snapshot') && ~isempty(motor_cmd.q_snapshot)
        q = motor_cmd.q_snapshot;
        if isfield(motor_cmd,'t_seq'), tseq = motor_cmd.t_seq; else, tseq = 0:size(q,1)-1; end
        g = optget(motor_cmd,'gripper_seq', zeros(1,size(q,1)));
    else
        error('motorCmdPayload:noq', '缺少 q_seq / q_snapshot');
    end
    if isempty(g) || numel(g) < size(q,1), g = [g, zeros(1, size(q,1)-numel(g))]; end
    P = struct('q_seq', q, 't_seq', tseq, 'gripper_seq', g(1:size(q,1)), ...
        'gripper_ori', optget(motor_cmd,'gripper_ori', 0), ...      % 7 通道：夹爪朝向(rad)
        'gripper_gap', optget(motor_cmd,'gripper_gap', gap0(g)), ... % 8 通道：夹爪开合 0..1
        'dq_max',    optget(motor_cmd,'dq_max', 0.15), ...
        'vel_ok',    optget(motor_cmd,'vel_ok', true), ...
        'safety_ok', optget(motor_cmd,'safety_ok', true), ...
        'success',   optget(motor_cmd,'success', true), ...
        'error_code',optget(motor_cmd,'error_code', 0), ...
        'q_final',   optget(motor_cmd,'q_final', q(end,:)), ...
        'meta',      optget(motor_cmd,'meta', struct()));
end

function v = gap0(g)
    % 由夹爪指令推导开合基准：0=保持→0.5，1=张开→1，2=闭合→0（末值优先）
    if isempty(g) || all(g==0), v = 0.5; else, v = (g(end)==1)*1 + (g(end)==2)*0; v = max(0,min(1,v)); end
end
