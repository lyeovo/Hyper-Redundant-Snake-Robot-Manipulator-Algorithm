function test_createModel()
%test_createModel 接口1 冒烟：缺省值、参数覆盖、旧字段迁移
    fprintf('== test_createModel ==\n');
    fails = 0;

    % 空参数 → 全默认
    m = createArmModel(struct());
    fails = fails + assertEq(m.cfg.N, 4, '默认 N=4');
    fails = fails + assertEq(m.cfg.w_pos, 1.0, '默认 w_pos');
    fails = fails + assertEq(m.cfg.d_safe, 0.06, '默认 d_safe=rho0+margin');
    fails = fails + assertEq(numel(m.cfg.q_min), 4, 'q_min 展开为向量');
    fails = fails + assertEq(m.cfg.q_max(1), pi, '默认 q_max');

    % 参数覆盖 + 逐段杆长
    m = createArmModel(struct('N', 6, 'L_seg', [1.0 0.8 0.8 0.6 0.6 0.5], ...
        'q_min', -0.5, 'q_max', 0.5, 'X_target', [3,1], 'theta_target', 0.2));
    fails = fails + assertEq(m.cfg.N, 6, '覆盖 N');
    fails = fails + assertEq(m.cfg.L_seg(3), 0.8, '逐段杆长');
    fails = fails + assertEq(m.cfg.q_max(6), 0.5, 'q_max 标量展开');
    fails = fails + assertEq(m.cfg.X_target(1), 3, '覆盖目标');
    fails = fails + assertEq(m.cfg.theta_target, 0.2, '覆盖角度');

    % 旧字段迁移：obs → circles；obs_lines → 薄矩形
    m = createArmModel(struct('obs', [1.5, 0.5, 0.2], ...
        'obs_lines', {{[0,0; 2,0]}}));
    fails = fails + assertEq(size(m.cfg.obstacles.circles,1), 1, 'obs→circles');
    fails = fails + assertEq(m.cfg.obstacles.circles(1,3), 0.2, '圆半径保留');
    fails = fails + assertEq(size(m.cfg.obstacles.rects,1), 1, 'obs_lines→矩形');
    fails = fails + assertNear(m.cfg.obstacles.rects(1,4), 2.0, '薄矩形长度=线段长');
    fails = fails + assertNear(m.cfg.obstacles.rects(1,5), 2*m.cfg.d_safe, '薄矩形宽度=2·d_safe');

    % 新结构 obstacles 直接传入
    m = createArmModel(struct('obstacles', struct('rects',[0,0,0,1,1], 'circles',[])));
    fails = fails + assertEq(size(m.cfg.obstacles.rects,1), 1, 'rects 直传');

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_createModel FAILED'); end
end

function f = assertEq(a, b, name)
    % 数值用容差比较（isequal 对浮点加法误差过严），其余用 isequal
    if isnumeric(a) && isnumeric(b)
        f = assertNear(a, b, name, 1e-12);
    elseif ~isequal(a, b)
        fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b));
        f = 1;
    else
        f = 0;
    end
end

function f = assertNear(a, b, name, tol)
    if nargin < 4, tol = 1e-9; end
    if abs(a - b) > tol
        fprintf('  [FAIL] %s: got %.6f, expected %.6f\n', name, a, b);
        f = 1;
    else
        f = 0;
    end
end
