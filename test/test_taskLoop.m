function test_taskLoop()
%test_taskLoop 任务列表循环执行器 + motorCmd 导出端到端
%   模拟视觉 UI 连续下发 3 个任务 → runTaskLoop 顺序执行 → 状态回写
%   → 导出 motorCmd .mat/.csv（dSPACE 可加载格式）→ 文件验证
    fprintf('== test_taskLoop ==\n');
    fails = 0;
    base = fullfile(tempdir, 'taskloop_test');
    outbox = fullfile(base, 'outbox');
    inbox  = fullfile(base, 'inbox');
    if exist(base, 'dir'), rmdir(base, 's'); end
    mkdir(outbox); mkdir(inbox);

    m = testModel('N', 4, 'X_target', [2.5, 1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects', [], 'circles', []));

    % ---- 1. 灌入 3 个任务（模拟视觉 UI 发布） ----
    types = {'pick_and_place', 'move_near_target', 'pick_target'};
    for k = 1:numel(types)
        cmd = mockCmd(types{k}, k);
        fid = fopen(fullfile(outbox, [cmd.command_id '.json']), 'w');
        fprintf(fid, '%s', jsonencode(cmd));
        fclose(fid);
    end

    % ---- 2. 任务列表顺序执行 ----
    stats = runTaskLoop(m, struct('outbox', outbox, 'inbox', inbox, ...
        'max_tasks', 3, 'poll_interval', 0.05, 'verbose', false));
    fails = fails + assertEq(stats.processed, 3, '处理 3 个任务');
    fails = fails + assertEq(stats.completed, 3, '全部 COMPLETED');
    fails = fails + assertEq(stats.failed, 0, '无失败');

    % ---- 3. 文件状态验证 ----
    d_done = dir(fullfile(outbox, '*.done'));
    fails = fails + assertEq(numel(d_done), 3, 'outbox 任务已 .done');
    d_stat = dir(fullfile(inbox, '*_status.json'));
    fails = fails + assertEq(numel(d_stat), 3, 'inbox 3 个状态文件');
    s_end = jsondecode(fileread(fullfile(inbox, d_stat(1).name)));
    fails = fails + assertEq(s_end.status, 'COMPLETED', '状态文件 COMPLETED');

    % ---- 4. motorCmd 导出（.mat + .csv） ----
    % 重放一个任务拿 motor_cmd（.done 文件内容即原 TaskCommand JSON）
    cmd0 = jsondecode(fileread(fullfile(outbox, d_done(1).name)));
    info = taskExecute(m, cmd0, struct('inbox', inbox, 'snapshot_m', 5));
    fails = fails + assertTrue(info.success, '重放任务成功');
    fn_mat = fullfile(base, 'motorCmd.mat');
    fn_csv = fullfile(base, 'motorCmd.csv');
    exportMotorCmd(info.motor_cmd, fn_mat);
    exportMotorCmd(info.motor_cmd, fn_csv);
    mc = load(fn_mat);
    fails = fails + assertTrue(isfield(mc, 'q_seq') && isfield(mc, 't_seq') ...
        && isfield(mc, 'gripper_seq'), '.mat 字段齐全');
    K = size(mc.q_seq, 1);
    fails = fails + assertTrue(K > 0, 'q_seq 非空（轨迹已导出）');
    fails = fails + assertEq(numel(mc.gripper_seq), K, 'gripper_seq 与 q_seq 对齐');
    % CSV 行数 = 表头 + K
    fid = fopen(fn_csv, 'r');
    lines = 0;
    while ~feof(fid), fgetl(fid); lines = lines + 1; end
    fclose(fid);
    fails = fails + assertEq(lines, K + 1, 'CSV 行数 = 快照数 + 表头');
    % CSV 内容抽查：首行表头
    fid = fopen(fn_csv, 'r');
    hdr = fgetl(fid);
    fclose(fid);
    fails = fails + assertTrue(~isempty(strfind(hdr, 'gripper')), 'CSV 含 gripper 列');

    rmdir(base, 's');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_taskLoop FAILED'); end
end

function cmd = mockCmd(ct, idx)
    cmd = struct();
    cmd.schema_version = '1.0';
    cmd.command_id = sprintf('CMD-LOOP-%d', idx);
    cmd.timestamp = posixtime(datetime('now')) + idx;   % 保证顺序
    cmd.source = 'SpaceSnakeVisionUI';
    cmd.command_type = ct;
    cmd.selected_target = struct('target_id', 'TGT-001', 'class_name', 'payload_module', ...
        'confidence', 0.9, ...
        'pose_camera', struct('frame_id', 'camera_left', ...
            'position', struct('x', 2.5, 'y', 1.0, 'z', 0.0), ...
            'orientation_quat', struct('x', 0, 'y', 0, 'z', 0, 'w', 1)), ...
        'pose_base', []);
    cmd.destination = struct('name', 'Assembly_Port_A', ...
        'pose_base', struct('frame_id', 'robot_base', ...
            'position', struct('x', 3.1, 'y', 0.2, 'z', 0.0), ...
            'orientation_quat', struct('x', 0, 'y', 0, 'z', 0, 'w', 1)));
    cmd.motion_params = struct('approach_distance_m', 0.15, ...
        'gripper_mode', 'demo_grip', 'speed_mode', 'normal');
    cmd.safety = struct('require_user_confirm', false, ...
        'allow_execute', true, 'estop_active', false);
end

function f = assertTrue(b, name)
    if ~b
        fprintf('  [FAIL] %s\n', name);
        f = 1;
    else
        f = 0;
    end
end

function f = assertEq(a, b, name)
    if ~isequal(a, b)
        fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b));
        f = 1;
    else
        f = 0;
    end
end
