function test_methodRl()
%test_methodRl RL 方法冒烟：训练完成、策略参数有效、返回结构
%   不要求收敛（RL 收敛依赖训练预算，属超参调优；residual 集成保证不劣于纯梯度）
    fprintf('== test_methodRl ==\n');
    fails = 0;
    m = testModel('X_target', [2.5,1.0], 'theta_target', 0.5, ...
        'obstacles', struct('rects', [], 'circles', []));
    info = simulateMotion(m, 'rl', zeros(1,4), [2.5,1.0,0.5], ...
        'Train', true, 'NPop', 8, 'NGen', 5, 'MaxRollout', 30);
    fails = fails + assertTrue(isstruct(info), 'RL 返回结构');
    fails = fails + assertTrue(info.stats.trained, 'RL 训练完成');
    fails = fails + assertTrue(all(all(isfinite(info.stats.theta))), '策略参数 θ 有限');
    fails = fails + assertTrue(~isempty(info.q_final), 'RL 有 q_final');
    % 未训练模式（θ=0 residual = 纯梯度）也应可用
    info2 = simulateMotion(m, 'rl', zeros(1,4), [2.5,1.0,0.5], ...
        'Train', false, 'MaxRollout', 30);
    fails = fails + assertTrue(isstruct(info2), 'RL 推理模式返回结构');
    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_methodRl FAILED'); end
end

function f = assertTrue(b, name)
    if ~b
        fprintf('  [FAIL] %s\n', name);
        f = 1;
    else
        f = 0;
    end
end
