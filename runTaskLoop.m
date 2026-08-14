function stats = runTaskLoop(model, opts)
%runTaskLoop 任务列表循环执行器（方案 §4.5 task_executor 主循环）
%   stats = runTaskLoop(model, opts)
%   model: createArmModel 输出
%   opts : .outbox（任务发布目录，必填） .inbox（状态回写目录，必填）
%          .method（默认 'auto'） .snapshot_m（默认 10） .approach_dist（默认 0.15）
%          .max_tasks（0=无限，默认 0） .poll_interval（秒，默认 1.0）
%          .estop（@() bool 急停） .verbose（打印，默认 true）
%
%   流程：轮询 outbox 中的 TaskCommand JSON（按 timestamp 升序）→ 逐个
%   taskExecute 执行 → 回写 TaskStatus → 处理完的 outbox 文件重命名为 .done
%   （防重复处理）。返回执行统计；Ctrl+C / max_tasks 可终止。
%
%   真实运行：MATLAB 常驻进程（或 GUI 按钮启动后台轮询）；虚拟仿真联调时
%   由测试脚本/演示向 outbox 灌入 mock 任务即可验证任务下发闭环。
%
%   独立启动：自动加入本文件所在目录下的 ArmSimulator2D/（幂等）。

    % 自动加入依赖目录（独立运行无需手动 addpath）
    runDir = fileparts(mfilename('fullpath'));
    if isempty(runDir), runDir = pwd; end
    addpath(fullfile(runDir, 'ArmSimulator2D'));

    if nargin < 2, opts = struct(); end
    outbox = getopt(opts, 'outbox', '');
    inbox  = getopt(opts, 'inbox', '');
    method = getopt(opts, 'method', 'auto');
    snapshot_m = getopt(opts, 'snapshot_m', 10);
    approach_dist = getopt(opts, 'approach_dist', 0.15);
    max_tasks = getopt(opts, 'max_tasks', 0);
    poll = getopt(opts, 'poll_interval', 1.0);
    estop = getopt(opts, 'estop', []);
    verbose = getopt(opts, 'verbose', true);
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
        % 取下一个未处理任务（*.json，按 timestamp 升序）
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
            movefile(fn, [fn '.bad']);      % 坏文件隔离
            continue;
        end
        if verbose
            fprintf('[runTaskLoop] 执行任务 %s (type=%s)\n', cmd.command_id, cmd.command_type);
        end
        info = taskExecute(model, cmd, struct('inbox', inbox, 'method', method, ...
            'snapshot_m', snapshot_m, 'approach_dist', approach_dist, 'estop', estop));
        n_done = n_done + 1;
        stats.processed = stats.processed + 1;
        switch info.status
            case 'COMPLETED', stats.completed = stats.completed + 1;
            case 'FAILED',    stats.failed = stats.failed + 1;
            case 'REJECTED',  stats.rejected = stats.rejected + 1;
            case 'ESTOP_TRIGGERED', stats.esstopped = stats.esstopped + 1;
        end
        stats.tasks(end+1) = struct('id', cmd.command_id, 'status', info.status); %#ok<AGROW>
        % 处理完成 → 重命名防重复
        movefile(fn, [fn '.done']);
        if max_tasks > 0 && n_done >= max_tasks, break; end
    end
end

function v = getopt(opts, field, default)
    if isfield(opts, field) && ~isempty(opts.(field))
        v = opts.(field);
    else
        v = default;
    end
end
