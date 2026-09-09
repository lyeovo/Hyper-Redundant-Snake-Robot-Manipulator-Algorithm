function test_chain_full_loop()
%test_chain_full_loop 页面级整链闭环（对应「链路控制台」页的编排）：可行则据此整修页面
%   1 电控 mock(TCP服务器,保留创建) → 2 注入 TaskCommand 到 outbox（同页面 inject_command）
%   3 算法: 读 outbox → taskExecute → motor_cmd + 状态(COMPLETED)  4 26参帧 → 电控 → STATE
%   5 校验：STATE 累计关节角 ≈ 算法 q_final；且 inbox 状态为 COMPLETED。
    fprintf('== test_chain_full_loop ==\n');
    fails = 0;  port = 9100;
    projRoot = fileparts(fileparts(mfilename('fullpath')));   % 仓库根 (test/..)
    addpath(genpath(projRoot));
    base = fullfile(tempdir, 'chainloop_test');  outbox = fullfile(base,'outbox');  inbox = fullfile(base,'inbox');
    if exist(base,'dir'), rmdir(base,'s'); end;  mkdir(outbox);  mkdir(inbox);
    readyf = fullfile(tempdir, 'elec_mock_ready.txt');
    if exist(readyf,'file'), delete(readyf); end

    % ---- 1. 启动电控 mock（保留 TCP 服务器创建）----
    pysrv = fullfile(projRoot, 'mock_elec_server.py');
    st = system(sprintf('start /b "" python "%s" %d 127.0.0.1 once', pysrv, port));
    assert(st == 0, '启动电控 mock 失败');
    ready = false;
    for w = 1:30, pause(0.5); if exist(readyf,'file')==2, ready=true; break; end; end
    assert(ready, '电控 mock 未就绪');

    try
        % ---- 2. 注入 TaskCommand 到 outbox（同页面 inject_command：move_to params x/y）----
        cid = 'CMD-CHAIN-1';
        cmd = struct('schema_version','2.0','command_id',cid,'timestamp',now(), ...
            'source','SpaceSnakeVisionUI','command_type','move_to', ...
            'params', struct('x', 1.2, 'y', 0.8), 'motion_params', struct(), ...
            'selected_target', [], ...
            'destination', struct('name','Goal_Zone','pose_base', struct('frame_id','robot_base', ...
                'position',struct('x',1.2,'y',0.8,'z',0.0),'orientation_euler',struct('roll',0,'pitch',0,'yaw',0))), ...
            'safety', struct('allow_execute', true, 'estop_active', false));
        fid = fopen(fullfile(outbox,[cid '.json']),'w'); fprintf(fid,'%s',jsonencode(cmd)); fclose(fid);
        fprintf('  注入 %s → outbox\n', cid);

        % ---- 3. 算法：读 outbox → taskExecute → motor_cmd + 状态 ----
        m = testModel('N', 6, 'X_target', [2.5, 1.0], 'theta_target', 0.0, ...
            'obstacles', struct('rects', [], 'circles', []));
        cmd2 = jsondecode(fileread(fullfile(outbox, [cid '.json'])));
        info = taskExecute(m, cmd2, struct('inbox', inbox, 'snapshot_m', 10, 'playback_delay', 0));
        fails = fails + assertTrue(info.success, '算法规划成功');
        mc = info.motor_cmd;
        fprintf('  算法: %d 帧轨迹, q_final=%s\n', size(mc.q_seq,1), mat2str(mc.q_final,4));
        stfile = fullfile(inbox, [cid '_status.json']);
        fails = fails + assertTrue(exist(stfile,'file')==2, 'inbox 状态写出');
        s = jsondecode(fileread(stfile));
        fails = fails + assertEq(s.status, 'COMPLETED', 'inbox 状态 COMPLETED');

        % ---- 4. 26参帧 → 电控 mock → STATE ----
        cp = motorCmdToControlParams(mc);
        nParams = numel(cp(1).motors)*4 + 2;
        fails = fails + assertEq(nParams, 26, '26 参');
        out = tcpSendControl('127.0.0.1', port, cp);
        fprintf('  电控回读累计关节角(rad): %s\n', mat2str(out.q_actual_final,4));

        % ---- 5. 校验闭环 ----
        err = max(abs(out.q_actual_final - mc.q_final));
        fprintf('  闭环误差: %.4g rad\n', err);
        fails = fails + assertTrue(err < 1e-6, '闭环收敛');
    catch e
        fails = fails + 1;
        fprintf('  [FAIL] 异常: %s\n    at: %s\n', e.message, e.stack(1).name);
    end
    rmdir(base, 's');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_chain_full_loop FAILED'); end
end

function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end
function f = assertEq(a, b, name)
    if ~isequal(a, b), fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b)); f = 1; else, f = 0; end
end
