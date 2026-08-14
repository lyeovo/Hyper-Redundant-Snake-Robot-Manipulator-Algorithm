function exportMotorCmd(motor_cmd, filename, varargin)
%exportMotorCmd 导出 motorCmd 为 dSPACE 可加载文件（方案 §1.2 / 电控对接说明 §2）
%   exportMotorCmd(motor_cmd, filename)
%   exportMotorCmd(motor_cmd, filename, 'GripperSeq', gs, 'Meta', meta)
%   motor_cmd: taskExecute 返回的 .motor_cmd，或 simulateMotion 返回的 info
%              （兼容：字段 q_snapshot/t_seq/vel_ok/safety_ok/success/error_code/q_final）
%   filename : 'xxx.mat'（save -struct，推荐）或 'xxx.csv'（文本可读）
%   'GripperSeq': 夹爪指令 [1×K]（缺省全 0 = 保持）
%   'Meta'      : 溯源 struct（command_id/task_type 等）
%
%   输出文件字段（与《电控对接说明.md》§2.1 一致）：
%     q_seq / t_seq / gripper_seq / dq_max / vel_ok / safety_ok / success / error_code / q_final / meta
    p = inputParser;
    addParameter(p, 'GripperSeq', [], @(x) isnumeric(x) || isempty(x));
    addParameter(p, 'Meta', struct(), @isstruct);
    parse(p, varargin{:});
    gs = p.Results.GripperSeq;
    meta = p.Results.Meta;

    % ---- 字段统一（兼容两种输入） ----
    if isfield(motor_cmd, 'q_snapshot') && ~isfield(motor_cmd, 'q_seq')
        mc = struct();
        mc.q_seq = motor_cmd.q_snapshot;
        mc.t_seq = motor_cmd.t_seq;
        mc.vel_ok = motor_cmd.vel_ok;
        mc.safety_ok = motor_cmd.safety_ok;
        mc.success = motor_cmd.success;
        mc.error_code = motor_cmd.error_code;
        mc.q_final = motor_cmd.q_final;
        if isfield(motor_cmd, 'meta'), meta = motor_cmd.meta; end
    else
        mc = motor_cmd;
    end
    if ~isfield(mc, 'dq_max'), mc.dq_max = 0.15; end
    if ~isfield(mc, 'q_seq'),  error('exportMotorCmd:args', '缺少 q_seq/q_snapshot'); end
    K = size(mc.q_seq, 1);
    if isempty(gs)
        if isfield(mc, 'gripper_seq') && ~isempty(mc.gripper_seq)
            gs = mc.gripper_seq;         % 输入已带夹爪序列（taskExecute 的 motor_cmd），优先保留
        else
            gs = zeros(1, K);            % 缺省全 0 = 保持
        end
    elseif numel(gs) < K
        gs(end+1:K) = gs(end);
    end
    mc.gripper_seq = gs(1:K);
    mc.meta = meta;

    [~, ~, ext] = fileparts(filename);
    switch lower(ext)
        case '.mat'
            save(filename, '-struct', 'mc');
        case '.csv'
            fid = fopen(filename, 'w');
            if fid < 0, error('exportMotorCmd:io', '无法写入 %s', filename); end
            N = size(mc.q_seq, 2);
            hdr = 't';
            for j = 1:N, hdr = [hdr sprintf(',q%d', j)]; end %#ok<AGROW>
            hdr = [hdr ',gripper'];
            fprintf(fid, '%s\n', hdr);
            for k = 1:K
                row = sprintf('%.6f', mc.t_seq(k));
                for j = 1:N
                    row = [row sprintf(',%.6f', mc.q_seq(k,j))]; %#ok<AGROW>
                end
                fprintf(fid, '%s,%d\n', row, mc.gripper_seq(k));
            end
            fclose(fid);
        otherwise
            error('exportMotorCmd:ext', '仅支持 .mat / .csv，got %s', ext);
    end
end
