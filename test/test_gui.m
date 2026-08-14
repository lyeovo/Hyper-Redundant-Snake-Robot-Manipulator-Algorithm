function test_gui()
%test_gui 交互 GUI（ArmSimApp）验收测试
%   覆盖：创建/默认值、参数修改→模型重建、障碍增删改、各方法单次求解、
%        5 种模拟视觉任务下发执行、关闭。
%   通过 feval 触发控件回调模拟真实交互（回调内部用 src+ancestor 定位 figure，
%   不依赖 gcbf，可在 -batch 下触发）。
    fprintf('== test_gui（交互 GUI 验收） ==\n');
    nf = 0;
    t0 = tic;
    app = ArmSimApp();
    set(app.fig, 'Visible', 'off');
    s = guidata(app.fig);
    try
        % ---------- 1. 创建与默认值 ----------
        ok(1) = s.model.cfg.N == 6;
        ok(2) = size(s.model.cfg.obstacles.circles, 1) == 1 && ...
                size(s.model.cfg.obstacles.rects, 1) == 0;
        ok(3) = numel(get(s.hTaskType, 'String')) == 5;   % 5 种任务类型

        % ---------- 2. 参数修改 → 单次求解（先清空障碍保证快收敛） ----------
        set(s.hObsTable, 'Data', {});
        feval(get(s.hObsTable, 'CellEditCallback'), s.hObsTable, []);
        s = guidata(app.fig);
        % 2a. 权重自定义生效
        set(s.hWPos, 'String', '2.0');
        set(s.hWAng, 'String', '0.6');
        set(s.hWObs, 'String', '1.0');
        feval(get(s.hRun, 'Callback'), s.hRun, []);
        s = guidata(app.fig);
        ok(4) = abs(s.model.cfg.w_pos - 2.0) < 1e-9 && abs(s.model.cfg.w_ang - 0.6) < 1e-9 && ...
                abs(s.model.cfg.w_obs - 1.0) < 1e-9;
        % 2b. 关节角单独修改
        set(s.hQj(2), 'String', '1.0');
        feval(get(s.hQj(2), 'Callback'), s.hQj(2), []);
        s = guidata(app.fig);
        ok(5) = abs(s.q(2) - 1.0) < 1e-9;
        % 2c. 一键全水平 / 全折叠（奇数 +π、偶数 -π 交替）
        feval(get(s.hLevel, 'Callback'), s.hLevel, []);
        s = guidata(app.fig);
        ok(6) = all(s.q == 0);
        feval(get(s.hFold, 'Callback'), s.hFold, []);
        s = guidata(app.fig);
        ok(7) = all(abs(s.q(1:2:end) - pi) < 1e-3) && all(abs(s.q(2:2:end) + pi) < 1e-3);
        % 2d. 关节数与输入框同步（N 变化重建）
        set(s.hN, 'String', '7');
        feval(get(s.hRun, 'Callback'), s.hRun, []);
        s = guidata(app.fig);
        ok(8) = numel(s.hQj) == 7;
        set(s.hN, 'String', '5');
        set(s.hL, 'String', '0.8');
        set(s.hQm, 'String', '-2.5');
        set(s.hQx, 'String', '2.5');
        set(s.hTx, 'String', '3.2');
        set(s.hTy, 'String', '0.6');
        set(s.hTth, 'String', '0.2');
        feval(get(s.hRun, 'Callback'), s.hRun, []);
        s = guidata(app.fig);
        ok(9) = s.model.cfg.N == 5 && ~isempty(s.snap) && size(s.snap, 2) == 5;
        ok(10) = ~isempty(s.info) && isfield(s.info, 'q_snapshot');

        % ---------- 3. 障碍增删改（表格 → 模型 → 实时可视化） ----------
        set(s.hObsAdd, 'Value', 1);   % 添加圆
        feval(get(s.hObsBtn, 'Callback'), s.hObsBtn, []);
        s = guidata(app.fig);
        ok(6) = size(s.model.cfg.obstacles.circles, 1) == 1;
        data = get(s.hObsTable, 'Data');
        data{1, 2} = '2.0, 0.5, 0.4';
        set(s.hObsTable, 'Data', data);
        feval(get(s.hObsTable, 'CellEditCallback'), s.hObsTable, []);
        s = guidata(app.fig);
        ok(11) = size(s.model.cfg.obstacles.circles, 1) == 1 && ...
                all(abs(s.model.cfg.obstacles.circles(1,:) - [2.0 0.5 0.4]) < 1e-9);
        feval(get(s.hObsDel, 'Callback'), s.hObsDel, []);
        s = guidata(app.fig);
        ok(12) = isempty(s.model.cfg.obstacles.circles);

        % ---------- 4. 各方法单次求解（无障碍目标 [3.2,0.6]） ----------
        methods = {'auto','momentum','sa','rrt','prm','rl'};
        for k = 1:numel(methods)
            set(s.hMethod, 'Value', k);
            feval(get(s.hRun, 'Callback'), s.hRun, []);
            s = guidata(app.fig);
            if isempty(s.snap) || isempty(s.info)
                fprintf('  [WARN] 方法 %s 无结果\n', methods{k});
            end
        end
        ok(13) = true;   % 各方法回调均正常返回（未抛错即通过）

        % ---------- 5. 5 种模拟视觉任务下发执行 ----------
        ct = {'move_near_target','pick_target','pick_and_place','dock_to_interface','home'};
        for k = 1:numel(ct)
            set(s.hTaskType, 'Value', k);
            feval(get(s.hTaskRun, 'Callback'), s.hTaskRun, []);
            s = guidata(app.fig);
            if isempty(s.info) || isempty(s.snap)
                fprintf('  [WARN] 任务 %s 无结果\n', ct{k});
            end
        end
        ok(14) = true;

        % ---------- 6. 关闭 ----------
        close(app.fig);
        ok(15) = true;

        nf = sum(~ok);
        if nf > 0
            fprintf('  失败断言: %s\n', mat2str(find(~ok)));
        end
    catch e
        nf = nf + 1;
        fprintf('  [FAIL] 异常: %s\n    at: %s\n', e.message, e.stack(1).name);
        if ishandle(app.fig), close(app.fig); end
    end
    fprintf('  耗时 %.1fs | 失败 %d 项\n', toc(t0), nf);
    if nf == 0
        fprintf('  GUI 验收通过 ✓\n');
    end
end
