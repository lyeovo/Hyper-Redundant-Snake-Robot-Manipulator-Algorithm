function test_taskUnify()
%test_taskUnify 视觉接口 v2.0 本地统一验证（params 键 / 度转弧度 / 控制类指令）
%   验证 taskToSegments / taskExecute 与 TASK_COMMAND_INTERFACE v2.0 的字段、单位一致：
%   params 为主（motion_params 镜像）、角度用度、距离用米、关节 1 索引、位置用 x/y，
%   并验证 emergency_stop / cancel_task 被 taskExecute 提前拦截回写。
    fprintf('== test_taskUnify ==\n');
    fails = 0;
    inbox = fullfile(tempdir, 'taskunify_test_inbox');
    if ~exist(inbox, 'dir'), mkdir(inbox); end
    delete(fullfile(inbox, '*.json'));

    m = testModel('N', 4, 'X_target', [2.5, 1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects', [], 'circles', []));

    % ---- 1. move_to：params.x/y（doc 常设 selected_target=null）----
    cmd = docCmd('move_to');  cmd.selected_target = [];  cmd.params = struct('x', 0.35, 'y', -0.12);
    segs = taskToSegments(cmd, m, 0.15);
    fails = fails + assertEq(segs(1).target, [0.35, -0.12, 0], 'move_to 用 params.x/y');

    % ---- 2. move_along：theta_deg/distance_m（度→弧度）----
    cmd = docCmd('move_along');  cmd.params = struct('theta_deg', 45, 'distance_m', 0.2);
    segs = taskToSegments(cmd, m, 0.15);
    fails = fails + assertEq(segs(1).kind, 'ee_relative', 'move_along kind');
    fails = fails + assertTrue(abs(segs(1).dir - pi/4) < 1e-9, 'move_along dir 度→弧度');
    fails = fails + assertEq(segs(1).dist, 0.2, 'move_along dist 米');

    % ---- 3. rotate：alpha_deg（度→弧度）----
    cmd = docCmd('rotate');  cmd.params = struct('alpha_deg', 90);
    segs = taskToSegments(cmd, m, 0.15);
    fails = fails + assertTrue(abs(segs(1).alpha - pi/2) < 1e-9, 'rotate alpha 度→弧度');

    % ---- 4. rotate_arm：joint_index + alpha_deg，且 joint_index 按 model.N 截断 ----
    cmd = docCmd('rotate_arm');  cmd.params = struct('joint_index', 10, 'alpha_deg', -30);
    segs = taskToSegments(cmd, m, 0.15);   % N=4 → 10 截断为 4
    fails = fails + assertEq(segs(1).joint, 4, 'joint_index 截断到 N');
    fails = fails + assertTrue(abs(segs(1).delta + pi/6) < 1e-9, 'rotate_arm delta 度→弧度');

    % ---- 5. facing_arm：joint_index + theta_deg（度→弧度）----
    cmd = docCmd('facing_arm');  cmd.params = struct('joint_index', 2, 'theta_deg', 60);
    segs = taskToSegments(cmd, m, 0.15);
    fails = fails + assertTrue(abs(segs(1).angle - pi/3) < 1e-9, 'facing_arm angle 度→弧度');

    % ---- 6. 向后兼容：旧键 theta/alpha/d/n（弧度）仍可用 ----
    cmd = docCmd('move_along');  cmd.params = [];  cmd.motion_params = struct('theta', pi/2, 'd', 0.3);
    segs = taskToSegments(cmd, m, 0.15);
    fails = fails + assertTrue(abs(segs(1).dir - pi/2) < 1e-9, '旧键 theta 回退可用');
    fails = fails + assertEq(segs(1).dist, 0.3, '旧键 d 回退可用');

    % ---- 7. emergency_stop：taskExecute 直接回写 ESTOP_TRIGGERED，不走运动展开 ----
    cmd = docCmd('emergency_stop');  cmd.params = struct();
    info = taskExecute(m, cmd, struct('inbox', inbox));
    fails = fails + assertEq(info.status, 'ESTOP_TRIGGERED', '急停状态');
    fails = fails + assertTrue(~info.success, '急停非成功');
    st = jsondecode(fileread(fullfile(inbox, [cmd.command_id '_status.json'])));
    fails = fails + assertEq(st.status, 'ESTOP_TRIGGERED', '急停状态文件');

    % ---- 8. cancel_task：回写 CANCELED ----
    cmd = docCmd('cancel_task');  cmd.params = struct();
    info = taskExecute(m, cmd, struct('inbox', inbox));
    fails = fails + assertEq(info.status, 'CANCELED', '取消状态');
    fails = fails + assertTrue(~info.success, '取消非成功');
    st = jsondecode(fileread(fullfile(inbox, [cmd.command_id '_status.json'])));
    fails = fails + assertEq(st.status, 'CANCELED', '取消状态文件');

    % ---- 9. 端到端：doc 格式 move_to（无 selected_target）经 taskExecute 正常执行 ----
    cmd = docCmd('move_to');  cmd.selected_target = [];  cmd.params = struct('x', 1.5, 'y', 0.5);
    info = taskExecute(m, cmd, struct('inbox', inbox));
    fails = fails + assertTrue(info.success, 'doc 格式 move_to 成功');

    delete(fullfile(inbox, '*.json'));
    rmdir(inbox, 's');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_taskUnify FAILED'); end
end

function cmd = docCmd(ct)
% docCmd 构造与 TASK_COMMAND_INTERFACE v2.0 schema 一致的命令（params 为主，motion_params 镜像）
    cmd = struct();
    cmd.schema_version = '2.0';
    cmd.command_id = ['CMD-UNI-', ct, '-', num2str(round(rand*1e5))];
    cmd.timestamp = posixtime(datetime('now'));
    cmd.source = 'SpaceSnakeVisionUI';
    cmd.command_type = ct;
    cmd.selected_target = struct('target_id','TGT-UNI','class_name','payload_module', ...
        'confidence', 0.95, ...
        'pose_camera', struct('frame_id','camera_left', ...
            'position', struct('x',2.5,'y',1.0,'z',0.0), ...
            'orientation_euler', struct('roll',0,'pitch',0,'yaw',0.5)), ...
        'pose_base', []);
    cmd.destination = struct('name','Assembly_Port_A', ...
        'pose_base', struct('frame_id','robot_base', ...
            'position', struct('x',3.1,'y',0.2,'z',0.0), ...
            'orientation_euler', struct('roll',0,'pitch',0,'yaw',0), ...
            'orientation_quat', struct('x',0,'y',0,'z',0,'w',1)));
    cmd.params = struct();       % 测试用例按需覆盖
    cmd.motion_params = struct(); % 镜像，测试按需覆盖
    cmd.safety = struct('require_user_confirm', false, ...
        'allow_execute', true, 'estop_active', false);
end

function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end

function f = assertEq(a, b, name)
    if ~isequal(a, b)
        fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b));
        f = 1;
    else
        f = 0;
    end
end
