function test_taskBridge()
%test_taskBridge 视觉任务桥 mock 端到端（方案 §1.1/§1.2）
%   构造 TaskCommand JSON → taskExecute → TaskStatus 回写验证
    fprintf('== test_taskBridge ==\n');
    fails = 0;
    inbox = fullfile(tempdir, 'taskbridge_test_inbox');
    if ~exist(inbox, 'dir'), mkdir(inbox); end
    % 清理旧状态文件
    delete(fullfile(inbox, '*.json'));

    m = testModel('N', 4, 'X_target', [2.5, 1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects', [], 'circles', []));

    % ---- 1. pick_and_place 正常执行 ----
    cmd = mockCmd('pick_and_place');
    info = taskExecute(m, cmd, struct('inbox', inbox, 'snapshot_m', 10));
    fails = fails + assertTrue(info.success, 'pick_and_place 成功');
    fails = fails + assertEq(info.status, 'COMPLETED', '状态 COMPLETED');
    gseq = info.motor_cmd.gripper_seq;
    fails = fails + assertEq(numel(gseq), size(info.motor_cmd.q_seq, 1), 'gripper_seq 与 q_seq 对齐');
    fails = fails + assertEq(gseq(gseq ~= 0), [2 1], '夹爪事件序列 [闭合(2) 张开(1)]');
    % 状态文件存在且最终状态正确
    fn = fullfile(inbox, [cmd.command_id '_status.json']);
    fails = fails + assertTrue(exist(fn, 'file') == 2, '状态文件写出');
    s = jsondecode(fileread(fn));
    fails = fails + assertEq(s.status, 'COMPLETED', '文件状态 COMPLETED');
    fails = fails + assertTrue(numel(s.robot_state.joint_positions) == 4, 'joint_positions 回填');

    % ---- 2. 状态机路径：应出现 ACCEPTED/PLANNING/EXECUTING 中间态（写文件被覆盖，检查接口不崩溃即可） ----
    taskWriteStatus(inbox, cmd, 'ACCEPTED');
    taskWriteStatus(inbox, cmd, 'PLANNING');
    taskWriteStatus(inbox, cmd, 'EXECUTING', 'Progress', 0.4, 'Step', 'GRASPING');
    s2 = jsondecode(fileread(fn));
    fails = fails + assertEq(s2.progress, 0.4, 'progress 回写');

    % ---- 3. 安全校验拒绝：allow_execute=false ----
    cmd2 = mockCmd('pick_and_place');
    cmd2.safety.allow_execute = false;
    info2 = taskExecute(m, cmd2, struct('inbox', inbox));
    fails = fails + assertTrue(~info2.success, 'allow_execute=false 拒绝');
    fails = fails + assertEq(info2.status, 'REJECTED', '状态 REJECTED');

    % ---- 4. 急停中断 ----
    cmd3 = mockCmd('pick_and_place');
    info3 = taskExecute(m, cmd3, struct('inbox', inbox, 'estop', @() true));
    fails = fails + assertTrue(~info3.success, '急停中断');
    fails = fails + assertEq(info3.status, 'ESTOP_TRIGGERED', '状态 ESTOP_TRIGGERED');

    % ---- 5. move_near_target 单段 ----
    cmd4 = mockCmd('move_near_target');
    info4 = taskExecute(m, cmd4, struct('inbox', inbox));
    fails = fails + assertTrue(info4.success, 'move_near_target 成功');
    fails = fails + assertEq(numel(info4.seg_infos), 1, '单段任务');

    % 清理
    delete(fullfile(inbox, '*.json'));
    rmdir(inbox, 's');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_taskBridge FAILED'); end
end

function cmd = mockCmd(ct)
% 构造与视觉 UI TaskCommand schema 一致的结构（jsonencode 后可被 jsondecode 还原）
    cmd = struct();
    cmd.schema_version = '1.0';
    cmd.command_id = ['CMD-TEST-', ct, '-', num2str(round(rand*1e5))];
    cmd.timestamp = posixtime(datetime('now'));
    cmd.source = 'SpaceSnakeVisionUI';
    cmd.command_type = ct;
    cmd.selected_target = struct( ...
        'target_id', 'TGT-001', 'class_name', 'payload_module', ...
        'confidence', 0.92, ...
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
