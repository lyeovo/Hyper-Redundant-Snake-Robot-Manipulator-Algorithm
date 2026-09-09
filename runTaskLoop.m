function stats = runTaskLoop(model, opts)
%runTaskLoop 任务列表循环执行器（方案 §4.5 task_executor 主循环）
%   stats = runTaskLoop(model, opts)

    runDir = fileparts(mfilename('fullpath'));
    if isempty(runDir), runDir = pwd; end
    addpath(fullfile(runDir, 'ArmSimulator2D'));

    if nargin < 2, opts = struct(); end
    outbox = optget(opts, 'outbox', '');
    inbox  = optget(opts, 'inbox', '');
    method = optget(opts, 'method', 'auto');
    snapshot_m = optget(opts, 'snapshot_m', 10);
    approach_dist = optget(opts, 'approach_dist', 0.15);
    playback_delay = optget(opts, 'playback_delay', 0.04);
    max_tasks = optget(opts, 'max_tasks', 0);
    poll = optget(opts, 'poll_interval', 1.0);
    estop = optget(opts, 'estop', []);
    verbose = optget(opts, 'verbose', true);
    % 可选电控下发：elec_host 非空才把 motor_cmd 26参帧发给电控；空则纯仿真（不连电控）
    elec_host = optget(opts, 'elec_host', '');
    elec_port = optget(opts, 'elec_port', '');
    elec_run  = optget(opts, 'elec_run', 1);       % 服务端运行参数：0=暂停电控驱动(仿真可容忍)
    % elec_decimate: 抽稀下发。1=逐快照下发；n>=段长(如20)=每段只下发一次总目标角
    elec_decimate = optget(opts, 'elec_decimate', 1);
    elec_speed_dps = optget(opts, 'elec_speed_dps', 1.8);   % 电机角速度(度/秒)，用于估算等待时间
    elec_extra_s   = optget(opts, 'elec_extra_s', 5.0);     % 每次驱动后额外等待(秒)
    % elec_mode: 'push'=本地起服务端推帧(dSPACE Interpreter 连入收帧，真实链)；'connect'=作为客户端连电控发帧(mock/旧)
    elec_mode = lower(optget(opts, 'elec_mode', 'push'));
    if ischar(elec_port) || isstring(elec_port)
        elec_port = str2double(elec_port);   % 字符串端口 → 数值
    end
    if isempty(outbox) || isempty(inbox)
        error('runTaskLoop:args', 'outbox and inbox dirs required');
    end
    if ~exist(outbox, 'dir'), mkdir(outbox); end
    if ~exist(inbox, 'dir'), mkdir(inbox); end

    stats = struct('processed', 0, 'completed', 0, 'failed', 0, ...
        'rejected', 0, 'esstopped', 0, 'tasks', struct('id', cell(1,0), 'status', cell(1,0)));
    n_done = 0;

    if verbose
        N_deg = 0;
        if isfield(model, 'cfg') && isfield(model.cfg, 'N')
            N_deg = model.cfg.N;
        end
        fprintf('=============================================================\n');
        fprintf('[runTaskLoop] task loop ready (DOF N=%d)\n', N_deg);
        fprintf('outbox dir: %s\n', outbox);
        fprintf('inbox dir : %s\n', inbox);
        fprintf('note: MATLAB resident listening (Busy is normal).\n');
        fprintf('      Send tasks from the Python UI; press Ctrl+C to stop.\n');
        if isempty(elec_host)
            fprintf('elec: not configured -> pure simulation (no elec)\n');
        elseif strcmp(elec_mode, 'push')
            fprintf('elec: push server (long-lived) %s:%g (ServerSocket; waits Interpreter client; master control)\n', elec_host, elec_port);
        else
            fprintf('elec: client connect %s:%g (send 26-param frames + read STATE per task)\n', elec_host, elec_port);
        end
        fprintf('=============================================================\n\n');
    end

    % ---- push 模式：先在循环前起长连接服务器(仅一次)，接受一个客户端并保持 ----
    pushh = [];
    if ~isempty(elec_host) && strcmp(elec_mode, 'push')
        try
            pushh = tcpPushMotorCmd('open', elec_port, 'speed_dps', elec_speed_dps, 'extra_s', elec_extra_s);
            fprintf('[runTaskLoop] waiting Interpreter client (once; long-lived master control)...\n');
            pushh = tcpPushMotorCmd('accept', pushh);
        catch e
            if verbose, fprintf('[runTaskLoop] push server start/accept failed (tolerated): %s\n', e.message); end
            pushh = [];
        end
    end

    while true
        if ~isempty(estop) && estop()
            if verbose, fprintf('[runTaskLoop] estop, terminating loop\n'); end
            break;
        end
        d = dir(fullfile(outbox, '*.json'));
        if isempty(d)
            if max_tasks > 0 && n_done >= max_tasks, break; end
            pause(poll);
            continue;
        end
        [~, order] = sort([d.datenum]);
        fn = fullfile(outbox, d(order(1)).name);
        try
            cmd = jsondecode(fileread(fn));
        catch e
            if verbose, fprintf('[runTaskLoop] parse failed %s: %s\n', d(order(1)).name, e.message); end
            movefile(fn, [fn '.bad']);
            continue;
        end
        if verbose
            fprintf('[runTaskLoop] executing task %s (type=%s)\n', cmd.command_id, cmd.command_type);
        end
        info = taskExecute(model, cmd, struct('inbox', inbox, 'method', method, ...
            'snapshot_m', snapshot_m, 'approach_dist', approach_dist, 'estop', estop, ...
            'playback_delay', playback_delay));
        if isfield(info, 'motor_cmd') && isfield(info.motor_cmd, 'q_final') && ~isempty(info.motor_cmd.q_final)
            model.q = info.motor_cmd.q_final;
        end
        % 可选：把 motor_cmd 26参帧下发给电控（elec_host 空则纯仿真、不连电控；连了但失败也容忍）
        if ~isempty(elec_host) && isfield(info, 'motor_cmd') && ~isempty(info.motor_cmd) && info.success
            try
                cp = motorCmdToControlParams(info.motor_cmd, 'decimate', elec_decimate);
                if strcmp(elec_mode, 'push')
                    % 长连接总控：向已连接的 Interpreter 客户端推一任务的帧(不关闭连接)
                    if isempty(pushh)
                        if verbose, fprintf('[runTaskLoop] push not connected, skip elec send\n'); end
                    else
                        % 若客户端曾断开，先有限等待其重连(最多 10s)，避免整链停摆
                        if ~pushh.connected
                            if verbose, fprintf('[runTaskLoop] waiting Interpreter client reconnect (<=10s)...\n'); end
                            pushh = tcpPushMotorCmd('accept', pushh, 'timeout_ms', 10000);
                        end
                        if pushh.connected
                            pushh = tcpPushMotorCmd('push', pushh, cp, 'run', elec_run);
                            if verbose, fprintf('[runTaskLoop] elec push done (%d frames, long connection kept)\n', numel(cp)); end
                        else
                            if verbose, fprintf('[runTaskLoop] client not connected, skip elec send\n'); end
                        end
                    end
                else
                    % 旧方向：作为客户端连电控发帧 + 回读 STATE
                    sout = tcpSendControl(elec_host, elec_port, cp, 'run', elec_run);
                    if verbose
                        fprintf('[runTaskLoop] elec feedback joints=%s (err=%.4g)\n', ...
                            mat2str(sout.q_actual_final,4), max(abs(sout.q_actual_final - info.motor_cmd.q_final)));
                    end
                end
            catch e
                if verbose, fprintf('[runTaskLoop] elec send failed (tolerated): %s\n', e.message); end
            end
        else
            if verbose && ~isempty(elec_host)
                fprintf('[runTaskLoop] task %s has no motor_cmd/not success, skip elec send\n', cmd.command_id);
            end
        end
        n_done = n_done + 1;
        stats.processed = stats.processed + 1;
        switch info.status
            case 'COMPLETED', stats.completed = stats.completed + 1;
            case 'FAILED',    stats.failed = stats.failed + 1;
            case 'REJECTED',  stats.rejected = stats.rejected + 1;
            case 'ESTOP_TRIGGERED', stats.esstopped = stats.esstopped + 1;
        end
        stats.tasks(end+1) = struct('id', cmd.command_id, 'status', info.status); %#ok<AGROW>
        movefile(fn, [fn '.done']);
        if max_tasks > 0 && n_done >= max_tasks, break; end
    end

    % ---- 循环结束：关闭 push 长连接服务器 ----
    if ~isempty(pushh) && isstruct(pushh)
        try
            tcpPushMotorCmd('close', pushh);
        catch e %#ok<NASGU>
        end
    end
end