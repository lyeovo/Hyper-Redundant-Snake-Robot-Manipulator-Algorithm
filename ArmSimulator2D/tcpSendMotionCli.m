function tcpSendMotionCli(jobDir)
%tcpSendMotionCli 后台下发 CLI（供 GUI 以 MATLAB 子进程异步调用）
%   tcpSendMotionCli(jobDir)
%   jobDir 内含 job.mat（-struct 保存：traj/q0/host/port/step/n/cid）
%   运行 tcpSendMotion（锁步），把每帧进度追加到 jobDir/progress.txt；
%   完成后写 jobDir/out.mat 并创建 jobDir/DONE 标志。
%   失败也写 jobDir/out.mat（含 .error 字段）再写 DONE。
    proj = fileparts(mfilename('fullpath'));
    addpath(proj);
    j = load(fullfile(jobDir, 'job.mat'));
    prog = fullfile(jobDir, 'progress.txt');
    out = struct('error', '');
    try
        out = tcpSendMotion(j.host, j.port, j.traj, j.q0, ...
            'step_per_rev', j.step, 'decimate', j.n, 'command_id', j.cid, ...
            'onMove', @(m) appendLine(prog, m));
    catch e
        out = struct('log', {{sprintf('ERROR: %s', e.message)}}, ...
            'q_actual', [], 'q_actual_final', [], 'error', e.message);
        appendLine(prog, sprintf('ERROR: %s', e.message));
    end
    save(fullfile(jobDir, 'out.mat'), 'out');
    fid = fopen(fullfile(jobDir, 'DONE'), 'w'); if fid > 0, fclose(fid); end
end

function appendLine(fp, m)
    fid = fopen(fp, 'a');
    if fid > 0, fprintf(fid, '%s\n', m); fclose(fid); end
end
