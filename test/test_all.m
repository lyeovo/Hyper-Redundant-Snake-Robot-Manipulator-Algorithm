function nfail = test_all()
%test_all 全量回归汇总入口
%   运行方式：matlab -batch "addpath('ArmSimulator2D'); addpath('test'); test_all"
%           或：matlab -batch "buildtool test"（见根目录 buildfile.m）
%   返回 nfail：失败项数（>0 时抛错，使 CI 能红灯）
%
%   历史坑：本函数原为无条件 fprintf('=========== 全部通过 ===========')，
%   不聚合任何失败数，导致子测试（如 test_gui）报了失败仍输出"全部通过"。
%   现改为逐项 try/catch 汇总，任一项失败即 error 退出。
    names = {'test_obstacle_geometry', 'test_gradcheck', 'test_createModel', ...
             'test_simulateMotion', 'test_errorModel', 'test_methodRl', ...
             'test_taskBridge', 'test_taskLoop', 'test_gui', 'test_topLevel'};
    fprintf('=========== 测试开始（共 %d 项）===========\n', numel(names));
    t0 = tic;
    failed = {};
    for i = 1:numel(names)
        nm = names{i};
        ok = true;
        try
            feval(nm);
        catch e
            ok = false;
            failed{end+1} = nm; %#ok<AGROW>
            fprintf('  [FAILED] %s: %s\n', nm, e.message);
        end
        if ok
            fprintf('  [ok] %s\n', nm);
        end
    end
    nfail = numel(failed);
    fprintf('=========== 耗时 %.1fs | 通过 %d / 失败 %d ===========\n', ...
        toc(t0), numel(names) - nfail, nfail);
    if nfail > 0
        fprintf('失败项：%s\n', strjoin(failed, ', '));
        error('test_all:FAILED', '%d 项测试失败', nfail);
    end
    fprintf('全部通过\n');
end
