function test_runTaskLoop_elec()
%test_runTaskLoop_elec 验证 runTaskLoop 带 elec_host/elec_port 的下发行为 + 仿真(不连电控)容忍
%   模式A: elec_host 空 → 纯仿真(不连电控)，任务照常规划+COMPLETED。
%   模式B: elec_host 非空 + 电控运行 → 任务完成后下发电控(26参帧)，仍 COMPLETED。
    fprintf('== test_runTaskLoop_elec ==\n');
    fails = 0;  port = 9100;
    projRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(projRoot));
    base = fullfile(tempdir, 'rtl_base');
    out = fullfile(base, 'outbox');  inn = fullfile(base, 'inbox');
    if exist(base,'dir'), rmdir(base,'s'); end;  mkdir(out);  mkdir(inn);
    readyf = fullfile(tempdir, 'elec_mock_ready.txt');
    if exist(readyf,'file'), delete(readyf); end
    pysrv = fullfile(projRoot, 'mock_elec_server.py');
    m = testModel('N', 6, 'X_target', [2.5,1.0], 'theta_target', 0.0, ...
        'obstacles', struct('rects', [], 'circles', []));

    % 模式A：纯仿真（不连电控）
    delete(fullfile(out,'*.json'));  delete(fullfile(inn,'*.json'));
    write_task(out, 'CMD-A');
    stA = runTaskLoop(m, struct('outbox',out,'inbox',inn,'max_tasks',1,'poll_interval',0.05, ...
        'elec_host','','elec_port','','verbose',false));
    fails = fails + assertEq(stA.completed, 1, '仿真模式 completed');
    fprintf('  模式A 仿真: completed=%d\n', stA.completed);

    % 模式B：连接电控（启动电控 mock 服务器 → runTaskLoop 作为客户端下发 26参帧）
    %   必须显式 elec_mode='connect'：mock_elec_server.py 自己是服务器占 9100，
    %   而 runTaskLoop 缺省 push 模式会自己 bind 同一端口 → 冲突导致推送被静默跳过(假绿)。
    delete(fullfile(out,'*.json'));  delete(fullfile(inn,'*.json'));
    if exist(readyf,'file'), delete(readyf); end
    system(sprintf('start /b "" python "%s" %d 127.0.0.1 once', pysrv, port));
    ready = false;
    for w = 1:30, pause(0.5); if exist(readyf,'file')==2, ready=true; break; end; end
    assert(ready, '电控服务器未就绪');
    write_task(out, 'CMD-B');
    stB = runTaskLoop(m, struct('outbox',out,'inbox',inn,'max_tasks',1,'poll_interval',0.05, ...
        'elec_host','127.0.0.1','elec_port',port,'elec_mode','connect','verbose',true));
    fails = fails + assertEq(stB.completed, 1, '连接电控 completed');
    stf = fullfile(inn, 'CMD-B_status.json');
    s = jsondecode(fileread(stf));
    fails = fails + assertEq(s.status, 'COMPLETED', '状态 COMPLETED');
    fprintf('  模式B 连电控: completed=%d status=%s\n', stB.completed, s.status);
    % 清理
    try, delete(readyf); catch, end
    rmdir(base, 's');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_runTaskLoop_elec FAILED'); end
end

function write_task(out, cid)
    cmd = struct('schema_version','2.0','command_id',cid,'timestamp',1,'source','x', ...
        'command_type','move_to','params',struct('x',1.2,'y',0.8),'motion_params',struct(), ...
        'selected_target',[],'destination',struct(),'safety',struct('allow_execute',true,'estop_active',false));
    fd = fopen(fullfile(out,[cid '.json']),'w'); fprintf(fd,'%s',jsonencode(cmd)); fclose(fd);
end

function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end
function f = assertEq(a, b, name)
    if ~isequal(a, b), fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b)); f = 1; else, f = 0; end
end
