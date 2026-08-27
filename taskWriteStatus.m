function taskWriteStatus(inbox, cmd, status, varargin)
%taskWriteStatus 回写 TaskStatus JSON 到视觉 UI inbox（方案 §1.2 状态机）
%   taskWriteStatus(inbox, cmd, status, 'Progress', 0.5, 'Step', 'MOVING_TO_PREGRASP', 'Msg', '', 'JointPos', q)
%   status 枚举：RECEIVED/ACCEPTED/REJECTED/PLANNING/EXECUTING/PAUSED/COMPLETED/FAILED/CANCELED/ESTOP_TRIGGERED
    opts = struct();
    for i = 1:2:numel(varargin), opts.(varargin{i}) = varargin{i+1}; end
    if ~isfield(opts, 'Progress'), opts.Progress = 0; end
    if ~isfield(opts, 'Step'), opts.Step = ''; end
    if ~isfield(opts, 'Msg'), opts.Msg = ''; end
    if ~isfield(opts, 'JointPos'), opts.JointPos = []; end
    if isempty(inbox), return; end

    s = struct();
    s.schema_version = '1.0';
    s.command_id = cmd.command_id;
    s.timestamp = posixtime(datetime('now'));
    s.status = status;
    s.current_step = opts.Step;
    s.progress = opts.Progress;
    s.message = opts.Msg;
    s.robot_state = struct('state', status, ...
        'end_effector_pose_base', [], ...
        'joint_positions', opts.JointPos, ...
        'message', opts.Msg);

    fn = fullfile(inbox, [cmd.command_id '_status.json']);
    tmp = fullfile(inbox, [cmd.command_id '_status.tmp.json']);   % 原子写：先写临时文件再替换，防视觉侧读到半截 JSON
    fid = fopen(tmp, 'w');
    if fid < 0
        warning('taskWriteStatus:io', '无法写入 %s', tmp);
        return;
    end
    fprintf(fid, '%s', jsonencode(s));
    fclose(fid);
    movefile(tmp, fn, 'f');
end
