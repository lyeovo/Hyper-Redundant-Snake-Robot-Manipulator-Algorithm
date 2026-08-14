function test_all()
%test_all M0/M1 汇总测试入口
%   运行方式：matlab -batch "cd('...'); addpath('ArmSimulator2D'); addpath('test'); test_all"
    fprintf('=========== 测试开始 ===========\n');
    test_obstacle_geometry();
    test_gradcheck();
    test_createModel();
    test_simulateMotion();
    test_errorModel();
    test_methodRl();
    test_taskBridge();
    test_taskLoop();
    test_gui();
    test_topLevel();
    fprintf('=========== 全部通过 ===========\n');
end
