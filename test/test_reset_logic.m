function test_reset_logic()
%test_reset_logic 诊断复位（reset）逻辑：目标/起点/安全性
%   验证 3 点：
%     A) 从非零当前位姿发 reset -> 复位段目标为全 0（伸直朝前）
%     B) 关键：taskExecute 的起点是 cfg.q_init 还是实际累计位姿 model.q
%     C) 复位后 motor_cmd.q_final 是否回零
    fprintf('== test_reset_logic ==\n');
    fails = 0;
    addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

    m = testModel('N', 6, 'X_target', [2.5,1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects',[],'circles',[]));

    % 模拟"已执行过上一步，实际位姿不再是初始位"：本地记录 model.q 已是非零
    q_now = [0.3, -0.4, 0.5, -0.6, 0.2, -0.1];   % 非零当前位姿(rad)
    m.q = q_now;                                  % 本地仿真位姿记录(电控不返回绝对位姿)

    % 发 reset 指令
    cmd = struct('schema_version','2.0','command_id','CMD-RST-1','timestamp',1, ...
        'source','x','command_type','reset','params',struct(),'motion_params',struct(), ...
        'selected_target',[],'destination',struct(),'safety',struct('allow_execute',true,'estop_active',false));

    info = taskExecute(m, cmd, struct('inbox','','snapshot_m',10,'playback_delay',0));
    fails = fails + assertTrue(info.success, 'reset 规划成功');
    fprintf('  success=%d status=%s\n', info.success, info.status);

    % A) 复位段目标（第一段为 reset，其 q_final 应回零）
    si = info.seg_infos(1).info;
    fprintf('  reset q_final = %s\n', mat2str(si.q_final,4));
    fails = fails + assertTrue(max(abs(si.q_final)) < 1e-9, '复位段目标全零');

    % C) motor_cmd.q_final 回零
    fprintf('  motor_cmd.q_final = %s\n', mat2str(info.motor_cmd.q_final,4));
    fails = fails + assertTrue(max(abs(info.motor_cmd.q_final)) < 1e-9, 'motor_cmd 回零');

    % B) 起点：taskExecute 应从本地仿真位姿记录 model.q 起始（电控不返回绝对位姿，以本地记录为准）。
    %    cfg.q_init 仅作初始记录；只要本地记录已累进为非零，复位/任一段都从 model.q 规划。
    %    这里 cfg.q_init 与 model.q 故意不一致，验证 taskExecute 用的是后者。
    m2 = testModel('N', 6, 'X_target', [2.5,1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects',[],'circles',[]));
    m2.q = q_now;                    % 本地仿真记录：设备实际位姿(非零)
    info2 = taskExecute(m2, cmd, struct('inbox','','snapshot_m',10,'playback_delay',0));
    sit = info2.seg_infos(1).info;
    q_start_use = sit.q_snapshot(1,:);   % 复位首帧（起始）
    fprintf('  不一致测试: 规划首帧起点=%s (model.q=%s)\n', mat2str(q_start_use,4), mat2str(q_now,4));
    % 修复后：taskExecute 用 model.q（->≈q_now）；修复前用 cfg.q_init（->≈0）
    fails = fails + assertTrue(max(abs(q_start_use - q_now)) < 1e-6, '复位/任务起点来自本地记录 model.q');

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_reset_logic FAILED'); end
end

function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end
