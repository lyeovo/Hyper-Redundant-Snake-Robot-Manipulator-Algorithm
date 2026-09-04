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
    if isempty(outbox) || isempty(inbox)
        error('runTaskLoop:args', '需要 outbox 与 inbox 目录');
    end
    if ~exist(outbox, 'dir'), mkdir(outbox); end
    if ~exist(inbox, 'dir'), mkdir(inbox); end

    stats = struct('processed', 0, 'completed', 0, 'failed', 0, ...
        'rejected', 0, 'esstopped', 0, 'tasks', struct('id', cell(1,0), 'status', cell(1,0)));
    n_done = 0;
    while true
        if ~isempty(estop) && estop()
            if verbose, fprintf('[runTaskLoop] 急停，终止循环\n'); end
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
            if verbose, fprintf('[runTaskLoop] 解析失败 %s: %s\n', d(order(1)).name, e.message); end
            movefile(fn, [fn '.bad']);
            continue;
        end
        if verbose
            fprintf('[runTaskLoop] 执行任务 %s (type=%s)\n', cmd.command_id, cmd.command_type);
        end
        info = taskExecute(model, cmd, struct('inbox', inbox, 'method', method, ...
            'snapshot_m', snapshot_m, 'approach_dist', approach_dist, 'estop', estop, ...
            'playback_delay', playback_delay));
        if isfield(info, 'motor_cmd') && isfield(info.motor_cmd, 'q_final') && ~isempty(info.motor_cmd.q_final)
            model.q = info.motor_cmd.q_final;
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
end