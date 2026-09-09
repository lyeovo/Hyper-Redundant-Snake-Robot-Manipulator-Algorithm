function test_loop_vision_algo_elec()
%test_loop_vision_algo_elec 视觉→算法→电控 全流程回环通讯仿真
%   1. 视觉(模拟 TaskCommand) 2. 算法(taskExecute→motor_cmd) 3. 电控(mock 子进程收 26 参帧→回 STATE)
%   4. 客户端锁步发 26 参帧+读 STATE → 校验累计关节角 ≈ 算法 q_final（闭环收敛）。
%   依赖：mockElecControlServer / motorCmdToControlParams / tcpSendControl / tcpEncodeFrame。
    fprintf('== test_loop_vision_algo_elec ==\n');
    fails = 0;  port = 9100;
    projRoot = fileparts(fileparts(mfilename('fullpath')));   % 仓库根
    addpath(genpath(projRoot));

    % ---- 启动电控 mock（Python 快速启动：ControlDesk/Interpreter+模型 的替身） ----
    pysrv = fullfile(projRoot, 'mock_elec_server.py');
    st = system(sprintf('start /b "" python "%s" %d 127.0.0.1 once', pysrv, port));
    assert(st == 0, '启动 mock 电控失败');
    % 等服务器绑定端口（就绪握手标志文件）
    readyf = fullfile(tempdir, 'elec_mock_ready.txt');
    ready = false;
    for w = 1:30
        pause(0.5);
        if exist(readyf, 'file') == 2, ready = true; break; end
    end
    assert(ready, 'mock 电控服务器 30s 内未就绪');;

    try
        % ---- 1. 视觉：可下达的 TaskCommand（move_to，绝对目标在可达域） ----
        m = testModel('N', 6, 'X_target', [2.5, 1.0], 'theta_target', 0.0, ...
            'obstacles', struct('rects', [], 'circles', []));
        cmdv = struct('schema_version','2.0','command_id','CMD-LOOP-1','timestamp',1, ...
            'source','SpaceSnakeVisionUI','command_type','move_to', ...
            'params', struct('x', 1.2, 'y', 0.8), 'motion_params', struct(), ...
            'selected_target', [], ...
            'destination', struct('name','Goal_Zone','pose_base', struct('frame_id','robot_base', ...
                'position',struct('x',1.2,'y',0.8,'z',0.0),'orientation_euler',struct('roll',0,'pitch',0,'yaw',0))), ...
            'safety', struct('allow_execute', true, 'estop_active', false));

        % ---- 2. 算法规划：taskExecute → motor_cmd ----
        info = taskExecute(m, cmdv, struct('inbox','', 'snapshot_m', 10, 'playback_delay', 0));
        fails = fails + assertTrue(info.success, '算法规划成功');
        mc = info.motor_cmd;
        K = size(mc.q_seq, 1);
        fprintf('  算法: %d 帧轨迹, q_final=%s\n', K, mat2str(mc.q_final, 4));

        % ---- 3. 26 参控制帧 ----
        cp = motorCmdToControlParams(mc);
        nParams = numel(cp(1).motors)*4 + 2;   % 6*4 + 2 = 26
        fprintf('  每档参数数: %d (6电机*4 + 1舵机*2)\n', nParams);
        fails = fails + assertEq(nParams, 26, '26 参');

        % ---- 4. 闭环：发 26 参帧 + 读 STATE 回环 ----
        out = tcpSendControl('127.0.0.1', port, cp);
        fprintf('  电控回读累计关节角(rad): %s\n', mat2str(out.q_actual_final, 4));

        % ---- 5. 回环校验：电控累计 ≈ 算法目标 ----
        err = max(abs(out.q_actual_final - mc.q_final));
        fprintf('  闭环误差: %.4g rad\n', err);
        fails = fails + assertTrue(err < 1e-6, '闭环收敛（电控累计=算法目标）');
    catch e
        fails = fails + 1;
        fprintf('  [FAIL] 异常: %s\n    at: %s\n', e.message, e.stack(1).name);
    end
    % 客户端断开后 mock 以 once 模式自行 exit（否则会继续 accept 残留占端口）。

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_loop_vision_algo_elec FAILED'); end
end

function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end
function f = assertEq(a, b, name)
    if ~isequal(a, b), fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b)); f = 1; else, f = 0; end
end
