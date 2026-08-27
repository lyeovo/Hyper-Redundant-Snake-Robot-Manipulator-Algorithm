function app = ArmSimApp()

%ArmSimApp — 基于 ArmSimulator2D 新接口的交互仿真 GUI（含模拟视觉任务）
%   app = ArmSimApp()
%
%   独立启动：无需手动 addpath，自动将本文件所在目录下的 ArmSimulator2D/
%   加入 MATLAB 路径（幂等，重复调用无副作用）。
%
%   功能：
%   - 模型参数：N / 杆长 / 关节限位 / 安全距离 / 迭代上限
%   - 【关节角手动微调】：每个关节一个输入框，可单独修改；一键"全水平"
%     （所有关节 0，臂沿 x 轴展开）/ "全折叠"（相邻关节交替 π，臂折叠最紧凑）
%   - 【目标函数权重】：w_pos / w_ang / w_obs / w_var / w_acc 自定义
%   - 障碍编辑：圆形 [x,y,r] 与可旋转矩形 [x,y,θ,w,h]（表格增删改，实时可视化）
%   - 目标位姿 [x,y,θ] 设置（绘图区十字可拖拽）
%   - 单次求解：auto/momentum/sa/rrt/rrtstar/prm/graph/rl/cvae/multilayer（多层·骨架图）+ 轨迹回放
%   - 【模拟视觉任务】选择任务类型（move_near_target/pick_target/pick_and_place/
%     dock_to_interface/home）→ 构造 TaskCommand → 任务级执行（展开运动序列、
%     逐段求解、夹爪动作、状态流 RECEIVED→…→COMPLETED）→ 逐段动画 → motorCmd 导出
%   - 可选文件桥：设置 outbox/inbox 目录后可真实模拟"视觉下发→回写"闭环
%
%   说明：旧 PlanarDrawApp.m 基于旧接口（runLArmIK_2D），本程序为重构版入口。

    % 自动加入依赖目录（独立运行无需手动 addpath）
    guiDir = fileparts(mfilename('fullpath'));
    if isempty(guiDir), guiDir = pwd; end
    addpath(fullfile(guiDir, 'ArmSimulator2D'));

    %% ---------- 状态 ----------
    s = struct();
    s.model = createArmModel(struct('N', 6, 'L_seg', 1.04393));   % 默认 6 关节 × 1043.93mm（真实机械臂）
    s.q = zeros(1, s.model.cfg.N);
    s.snap = [];            % 轨迹（单次求解或任务级各段拼接）
    s.dragMode = 'none';    % 拖拽模式：none/target/place/obs
    s.dragRow = 0;          % 拖拽障碍的表行号
    s.graphDisp = [];       % 走廊图叠加显示数据（method_graph 求解后）
    s.gripperSeq = [];      % 与 snap 对齐的夹爪指令（0保持 1开 2合）
    s.snapIdx = 1;
    s.info = [];
    s.dragMode = 'none';
    s.taskOutbox = '';      % 可选：模拟视觉发布的 outbox 目录
    s.taskInbox = '';       % 可选：状态回写 inbox 目录

    %% ---------- 界面 ----------
    fig = figure('Name','ArmSimApp — 平面高冗余机械臂仿真','NumberTitle','off', ...
        'Units','normalized','Position',[0.04 0.05 0.92 0.90], ...
        'Color',[0.13 0.14 0.16],'KeyPressFcn',@onKey,'CloseRequestFcn',@onClose);

    % ---- 左侧控制面板（标签页分组，避免拥挤/底部溢出） ----
    hp = uipanel('Parent',fig,'Units','normalized','Position',[0.005 0.02 0.235 0.96], ...
        'Background',[0.14 0.15 0.18],'Title','控制面板','Foreground',[0.9 0.9 1],'FontSize',9);
    tg = uitabgroup(hp,'Units','normalized','Position',[0.01 0.01 0.98 0.98]);
    tabScene = uitab(tg,'Title','场景 · 求解');
    tabParam = uitab(tg,'Title','模型 · 权重');
    rH = 0.0265;

    % ========== Tab 1：场景 · 求解 ==========
    py = 0.97;
    sectionTitle(tabScene, py, '── 目标位姿 ──'); py = py - 0.024;
    [hTx, hTy, py]  = twoCol(tabScene, py, rH, '目标 X:', '2.0', '目标 Y:', '0.5');
    s.hTx = hTx;  s.hTy = hTy;
    [hTth, ~, py]   = twoCol(tabScene, py, rH, '目标 θ:', '0.0', '', '');
    s.hTth = hTth;

    sectionTitle(tabScene, py, '── 关节角（手动微调） ──'); py = py - 0.024;
    s.hQjPanel = uipanel(tabScene,'Units','normalized','Position',[0.02 py-0.155 0.96 0.155], ...
        'BorderType','none','Background',[0.14 0.15 0.18]);
    py = py - 0.164;
    s.hLevel = uicontrol(tabScene,'Style','pushbutton','String','— 全水平', ...
        'Units','normalized','Position',[0.05 py-0.016 0.42 0.033], ...
        'Background',[0.25 0.45 0.55],'Foreground',[1 1 1],'FontSize',9,'Callback',@onSetLevel);
    s.hFold = uicontrol(tabScene,'Style','pushbutton','String','≡ 全折叠', ...
        'Units','normalized','Position',[0.53 py-0.016 0.42 0.033], ...
        'Background',[0.45 0.35 0.55],'Foreground',[1 1 1],'FontSize',9,'Callback',@onSetFold);
    py = py - 0.038;
    s.hQj = [];  s.hQjLbl = [];   % 动态创建（syncJointEditors 填充）

    sectionTitle(tabScene, py, '── 单次求解 ──'); py = py - 0.024;
    s.hMethod = uicontrol(tabScene,'Style','popupmenu', ...
        'String',{'auto（推荐）','momentum 动量','sa 模拟退火','rrt 采样','rrtstar 渐近最优','prm 路线图','graph 图引导','rl 强化学习（实验性）','cvae 策略（L2 模型）','multilayer 多层（骨架图）'}, ...
        'Value',1,'Units','normalized','Position',[0.05 py-0.012 0.90 0.026], ...
        'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',8);
    py = py - 0.031;
    s.hMulti = uicontrol(tabScene,'Style','checkbox','String','强迫多层（use_multilayer）', ...
        'Value',1,'Units','normalized','Position',[0.05 py-0.012 0.90 0.024], ...
        'Background',[0.14 0.15 0.18],'Foreground',[0.85 0.85 0.95],'FontSize',7);
    py = py - 0.028;
    uicontrol(tabScene,'Style','text','String','策略文件:','Units','normalized', ...
        'Position',[0.03 py-rH 0.24 rH],'Background',[0.14 0.15 0.18], ...
        'Foreground',[0.85 0.85 0.95],'FontSize',7,'HorizontalAlignment','left');
    s.hPolicyFile = uicontrol(tabScene,'Style','edit','String','rl_pipeline/policy_cvae_c1_ft.mat', ...
        'Units','normalized','Position',[0.27 py-rH 0.69 rH],'Background',[0.2 0.2 0.3], ...
        'Foreground',[1 1 1],'FontSize',7);
    py = py - rH - 0.005;
    s.hRun = uicontrol(tabScene,'Style','pushbutton','String','▶ 求解运动', ...
        'Units','normalized','Position',[0.05 py-0.016 0.40 0.033], ...
        'Background',[0.15 0.55 0.25],'Foreground',[1 1 1],'FontSize',9,'Callback',@onRun);
    s.hPlay = uicontrol(tabScene,'Style','pushbutton','String','⏩ 回放', ...
        'Units','normalized','Position',[0.47 py-0.016 0.25 0.033], ...
        'Background',[0.25 0.4 0.65],'Foreground',[1 1 1],'FontSize',9,'Callback',@onPlay);
    uicontrol(tabScene,'Style','text','String','回放×','Units','normalized', ...
        'Position',[0.73 py-0.012 0.11 0.022],'Background',[0.14 0.15 0.18], ...
        'Foreground',[0.85 0.85 0.95],'FontSize',7,'HorizontalAlignment','right');
    s.hSpeed = uicontrol(tabScene,'Style','edit','String','1.0','Units','normalized', ...
        'Position',[0.84 py-0.014 0.13 0.026],'Background',[0.2 0.2 0.3], ...
        'Foreground',[1 1 1],'FontSize',8);
    py = py - 0.037;
    s.hFit = uicontrol(tabScene,'Style','pushbutton','String','🔄 适应视图（显示全部物体）', ...
        'Units','normalized','Position',[0.05 py-0.014 0.90 0.028], ...
        'Background',[0.30 0.30 0.42],'Foreground',[1 1 1],'FontSize',8, ...
        'TooltipString','自动缩放/平移视图，使臂、障碍、目标、轨迹全部可见', ...
        'Callback',@onFitView);
    py = py - 0.033;

    sectionTitle(tabScene, py, '── 模拟视觉任务 ──'); py = py - 0.024;
    uicontrol(tabScene,'Style','text','String','任务类型:','Units','normalized', ...
        'Position',[0.03 py-rH 0.26 rH],'Background',[0.14 0.15 0.18], ...
        'Foreground',[0.9 0.9 1],'FontSize',7,'HorizontalAlignment','left');
    s.hTaskType = uicontrol(tabScene,'Style','popupmenu', ...
        'String',{'move_near_target','pick_target','pick_and_place','dock_to_interface','home'}, ...
        'Value',3,'Units','normalized','Position',[0.31 py-rH 0.64 0.027], ...
        'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    py = py - rH - 0.005;
    [hDx, hDy, py]  = twoCol(tabScene, py, rH, '放置 X:', '1.0', '放置 Y:', '0.5');
    s.hDx = hDx;  s.hDy = hDy;
    py = py - 0.004;
    s.hTaskRun = uicontrol(tabScene,'Style','pushbutton','String','📤 下发任务并执行', ...
        'Units','normalized','Position',[0.05 py-0.016 0.42 0.033], ...
        'Background',[0.55 0.4 0.15],'Foreground',[1 1 1],'FontSize',9,'Callback',@onTaskRun);
    s.hExport = uicontrol(tabScene,'Style','pushbutton','String','💾 导出 motorCmd', ...
        'Units','normalized','Position',[0.53 py-0.016 0.42 0.033], ...
        'Background',[0.35 0.35 0.5],'Foreground',[1 1 1],'FontSize',9,'Callback',@onExportMotor);
    py = py - 0.037;
    [hOb, hIb, py]  = twoCol(tabScene, py, rH, 'outbox(可选):', '', 'inbox(可选):', '');
    s.hOutbox = hOb;  s.hInbox = hIb;

    % ========== Tab 2：模型 · 权重 ==========
    py = 0.97;
    sectionTitle(tabParam, py, '── 模型参数 ──'); py = py - 0.024;
    [hN, hL, py]    = twoCol(tabParam, py, rH, '关节数 N:', num2str(s.model.cfg.N), '杆长 L(m):', num2str(s.model.cfg.L_seg(1)));
    s.hN = hN;  s.hL = hL;
    [hQm, hQx, py]  = twoCol(tabParam, py, rH, 'q_min:', num2str(s.model.cfg.q_min(1)), 'q_max:', num2str(s.model.cfg.q_max(1)));
    s.hQm = hQm;  s.hQx = hQx;
    [hRho, hMi, py]= twoCol(tabParam, py, rH, '安全距 rho0:', num2str(s.model.cfg.rho0), 'max_iter:', num2str(s.model.cfg.max_iter));
    s.hRho = hRho;  s.hMi = hMi;
    [hSamp, ~, py]  = twoCol(tabParam, py, rH, '采样预算:', num2str(s.model.cfg.rrt_max_samples), '', '');
    s.hSamp = hSamp;
    s.hAdaptive = uicontrol(tabParam,'Style','checkbox','String','自适应预算（按场景难度，失败自动升档）', ...
        'Value',1,'Units','normalized','Position',[0.05 py-rH-0.008 0.92 rH], ...
        'Background',[0.14 0.15 0.18],'Foreground',[0.85 0.85 0.95],'FontSize',7, ...
        'Callback',@onParamEdit);
    py = py - rH - 0.016;
    s.hStrict = uicontrol(tabParam,'Style','checkbox','String','严格收敛（位置<1e-4 且 角度<1e-3；关闭=动态逐项<0.001）', ...
        'Value',1,'Units','normalized','Position',[0.05 py-rH-0.008 0.92 rH], ...
        'Background',[0.14 0.15 0.18],'Foreground',[0.85 0.85 0.95],'FontSize',7, ...
        'Callback',@onParamEdit);
    py = py - rH - 0.016;

    sectionTitle(tabParam, py, '── RRT / RRT* 参数（采样方法专用）──'); py = py - 0.024;
    [hMst, hGep, py] = twoCol(tabParam, py, rH, '单步延伸:', num2str(s.model.cfg.rrt_max_step), ...
        '位置容差:', num2str(s.model.cfg.rrt_goal_eps));
    s.hMaxStep = hMst;  s.hGoalEps = hGep;
    [hGan, hRf, py] = twoCol(tabParam, py, rH, '角度容差:', num2str(s.model.cfg.rrt_goal_ang), ...
        'RRT* 精修步:', '150');
    s.hGoalAng = hGan;  s.hRfSteps = hRf;
    [hGam, ~, py] = twoCol(tabParam, py, rH, 'RRT* 半径γ:', '2.5', '', '');
    s.hRGamma = hGam;
    py = py - 0.004;

    sectionTitle(tabParam, py, '── 目标函数权重 ──'); py = py - 0.024;
    [hWp, hWa, py] = twoCol(tabParam, py, rH, 'w_pos:', num2str(s.model.cfg.w_pos), 'w_ang:', num2str(s.model.cfg.w_ang));
    s.hWPos = hWp;  s.hWAng = hWa;
    [hWo, hWv, py] = twoCol(tabParam, py, rH, 'w_obs:', num2str(s.model.cfg.w_obs), 'w_var:', num2str(s.model.cfg.w_var));
    s.hWObs = hWo;  s.hWVar = hWv;
    [hWc, ~, py]   = twoCol(tabParam, py, rH, 'w_acc:', num2str(s.model.cfg.w_acc), '', '');
    s.hWAcc = hWc;

    % 参数编辑回调：修改模型/目标/权重参数后立即重建并重置图像（保持一致）
    set([hN hL hQm hQx hRho hMi hSamp hTx hTy hTth hWp hWa hWo hWv hWc], 'Callback', @onParamEdit);


    % ---- 右侧：绘图区（中） + 运行信息日志列（右） ----
    ax = axes('Parent',fig,'Units','normalized','Position',[0.26 0.05 0.51 0.90], ...
        'Color',[0.10 0.11 0.13],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7], ...
        'Box','on','ButtonDownFcn',@onAxClick);
    s.ax = ax;
    hold(ax,'on'); axis(ax,'equal');
    title(ax,'平面高冗余机械臂（拖拽目标十字移动目标）','Color',[0.9 0.9 0.9]);

    uicontrol(fig,'Style','text','String', ...
        '操作：目标十字可拖拽；关节角可单独修改；“下发任务并执行”走完整任务链路（展开→逐段→夹爪→状态流）', ...
        'Units','normalized','Position',[0.26 0.015 0.51 0.025], ...
        'Background',[0.13 0.14 0.16],'Foreground',[0.7 0.7 0.8],'FontSize',8);

    % 右侧运行信息列（可滚动日志，解决日志显示不全）——上部日志 + 右下角障碍编辑
    hLogP = uipanel('Parent',fig,'Units','normalized','Position',[0.78 0.40 0.215 0.58], ...
        'Background',[0.13 0.13 0.15],'Title','运行信息（任务状态流 / 求解结果）','Foreground',[0.9 0.9 1],'FontSize',9);
    s.hLog = uicontrol(hLogP,'Style','listbox','Max',2,'Value',1, ...
        'Units','normalized','Position',[0.02 0.02 0.96 0.93], ...
        'String',{'就绪。可自定义机械臂/障碍 → 单次求解 或 下发模拟视觉任务。'}, ...
        'Background',[0.1 0.1 0.12],'Foreground',[0.8 1 0.85],'FontSize',8);
    s.hLogP = hLogP;

    % 右下角：场景 · 障碍编辑（圆 [x,y,r] / 矩形 [x,y,θ,w,h]）
    hObsP = uipanel('Parent',fig,'Units','normalized','Position',[0.78 0.02 0.215 0.36], ...
        'Background',[0.13 0.13 0.15],'Title','场景 · 障碍编辑','Foreground',[0.9 0.9 1],'FontSize',9);
    s.hRand = uicontrol(hObsP,'Style','pushbutton','String','🎲 一键随机场景', ...
        'Units','normalized','Position',[0.03 0.87 0.94 0.10], ...
        'Background',[0.30 0.45 0.30],'Foreground',[1 1 1],'FontSize',8, ...
        'TooltipString','随机生成合理障碍组 + 臂初始姿态 + 目标（不修改 N/杆长/角度区间）', ...
        'Callback',@onRandomScene);
    s.hObsTable = uitable(hObsP,'Units','normalized','Position',[0.03 0.17 0.94 0.67], ...
        'ColumnName',{'类型','参数'},'ColumnWidth',{70 155},'ColumnEditable',[true true], ...
        'Data',{'circle','1.0, 1.4, 0.30'}, ...
        'CellEditCallback',@onObsEdit,'BackgroundColor',[0.2 0.2 0.28], ...
        'ForegroundColor',[1 1 1],'FontSize',8);
    s.hObsAdd = uicontrol(hObsP,'Style','popupmenu','String',{'添加圆','添加矩形'}, ...
        'Value',1,'Units','normalized','Position',[0.03 0.04 0.40 0.09], ...
        'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    s.hObsBtn = uicontrol(hObsP,'Style','pushbutton','String','添加', ...
        'Units','normalized','Position',[0.45 0.04 0.22 0.09], ...
        'Background',[0.3 0.3 0.4],'Foreground',[1 1 1],'FontSize',7,'Callback',@onObsAdd);
    s.hObsDel = uicontrol(hObsP,'Style','pushbutton','String','删选中行', ...
        'Units','normalized','Position',[0.69 0.04 0.28 0.09], ...
        'Background',[0.55 0.25 0.2],'Foreground',[1 1 1],'FontSize',7,'Callback',@onObsDel);
    s.hObsP = hObsP;

    s = rebuildModel(s);
    s = syncJointEditors(s);
    s = drawAll(s);
    guidata(fig, s);
    app.fig = fig;
end

%% ---------- 模型重建与绘制 ----------
function s = rebuildModel(s)
    p = struct();
    p.N = max(2, round(str2double(get(s.hN,'String'))));
    p.L_seg = str2double(get(s.hL,'String'));
    p.q_min = str2double(get(s.hQm,'String'));
    p.q_max = str2double(get(s.hQx,'String'));
    p.rho0 = str2double(get(s.hRho,'String'));
    p.max_iter = max(50, round(str2double(get(s.hMi,'String'))));
    p.X_target = [str2double(get(s.hTx,'String')), str2double(get(s.hTy,'String'))];
    p.theta_target = str2double(get(s.hTth,'String'));
    % 目标函数权重（自定义）
    if isfield(s, 'hWPos') && ishandle(s.hWPos)
        p.w_pos = str2double(get(s.hWPos,'String'));
        p.w_ang = str2double(get(s.hWAng,'String'));
        p.w_obs = str2double(get(s.hWObs,'String'));
        p.w_var = str2double(get(s.hWVar,'String'));
        p.w_acc = str2double(get(s.hWAcc,'String'));
    end
    obs = struct('rects', [], 'circles', []);
    data = get(s.hObsTable,'Data');
    for k = 1:size(data,1)
        if isempty(data{k,2}), continue; end
        v = str2num(data{k,2}); %#ok<ST2NM>
        if isempty(v), continue; end
        if strcmpi(data{k,1}, 'circle')
            if numel(v) >= 3, obs.circles(end+1,:) = v(1:3); end %#ok<AGROW>
        elseif strcmpi(data{k,1}, 'rect')
            if numel(v) >= 5, obs.rects(end+1,:) = v(1:5); end %#ok<AGROW>
        end
    end
    p.obstacles = obs;
    s.model = createArmModel(p);
    if numel(s.q) ~= s.model.cfg.N
        s.q = zeros(1, s.model.cfg.N);
    end
end

function s = syncJointEditors(s)
    % 根据当前 N 重建关节角输入框（每行 2 个，最多 12 关节）
    N = s.model.cfg.N;
    if ~isfield(s, 'hQjPanel') || ~ishandle(s.hQjPanel), return; end
    if ~isempty(s.hQj)
        delete([s.hQjLbl(:); s.hQj(:)]);
    end
    s.hQj = [];  s.hQjLbl = [];
    nRows = max(1, ceil(N/2));
    for i = 1:min(N, 12)
        col = mod(i-1, 2);  row = floor((i-1)/2);
        x0 = 0.02 + col*0.50;  y0 = 1 - (row+1)*0.155;
        s.hQjLbl(i) = uicontrol(s.hQjPanel,'Style','text','String',sprintf('q%d', i), ...
            'Units','normalized','Position',[x0 y0-0.015 0.11 0.12], ...
            'Background',[0.14 0.15 0.18],'Foreground',[0.9 0.9 1],'FontSize',7, ...
            'HorizontalAlignment','right');
        s.hQj(i) = uicontrol(s.hQjPanel,'Style','edit','String',num2str(s.q(i),'%.3f'), ...
            'Units','normalized','Position',[x0+0.12 y0-0.018 0.36 0.125], ...
            'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',8, ...
            'Callback',@onQjEdit);
    end
    if N > 12
        uicontrol(s.hQjPanel,'Style','text','String',sprintf('(N=%d 超出 12，余下关节请用命令行)', N), ...
            'Units','normalized','Position',[0.02 0.01 0.96 0.1], ...
            'Background',[0.14 0.15 0.18],'Foreground',[1 0.7 0.3],'FontSize',7);
    end
end

function s = computeTraj(s)
    % 由求解快照生成碰撞安全的连贯插值轨迹（回放/轨迹线共用）
    %   s.trajQ   [M×N] 插值构型（侵入帧已替换为最近安全帧）
    %   s.trajPts [M×2] 末端点序列（用于轨迹线）
    %   s.trajGIdx [1×M] 夹爪指令对齐索引（映射回快照）
    s.trajQ = [];  s.trajPts = [];  s.trajGIdx = [];
    Q = s.snap;
    if isempty(Q) || size(Q,1) < 1, return; end
    cfg = s.model.cfg;
    n = size(Q,1);
    if n < 2
        Qc = Q;  ups = 1;
    else
        % 快照已是完整真实轨迹（Snapshot=1，求解器保证无碰撞），仅 2 倍轻平滑去折角；
        % ups=1 时跳过逐帧碰撞检查（快照本身安全，避免 6000 帧 × obsDistAll 的 8s+ 开销）
        ups = 2;
        if n > 2000, ups = 1; end      % 长轨迹（大采样预算）不插值，性能优先
        Qu = unwrap(Q, [], 1);
        ts = linspace(0, n-1, 1 + (n-1)*ups);
        Qc = interp1(0:n-1, Qu, ts, 'pchip');
    end
    m = size(Qc,1);
    % 逐帧碰撞检查（仅 ups>1 的插值帧需要——快照帧已由求解器保证无碰撞）
    if ups > 1
        free = true(m,1);
        for k = 1:m
            g = obsDistAll(s.model, Qc(k,:));
            if ~isempty(g) && min(g) < cfg.rho0
                free(k) = false;
            end
        end
        if any(~free)
            safeIdx = find(free);
            if isempty(safeIdx)
                % 全部插值帧都碰撞（极端）：退回原始求解快照（求解器保证无碰），避免空索引报错
                Qc = Q;  ups = 1; %#ok<AGROW>
            else
                for k = find(~free)'
                    [~, j] = min(abs(safeIdx - k));
                    Qc(k,:) = Qc(safeIdx(j),:);
                end
            end
        end
    end
    m = size(Qc,1);   % 插值/回退后重新计算帧数
    s.trajQ = Qc;
    s.trajGIdx = max(1, min(n, 1 + floor((0:m-1)/ups)));
    pts = zeros(m,2);
    for k = 1:m
        [~, pe] = planarFK_L(Qc(k,:), s.model.DH, cfg.rod_offset_arr);
        pts(k,:) = pe;
    end
    s.trajPts = pts;
end

function s = drawAll(s)
    cla(s.ax); hold(s.ax,'on'); axis(s.ax,'equal');
    cfg = s.model.cfg;
    R = cfg.N * cfg.L_seg(1);
    th = linspace(0, 2*pi, 100);
    plot(s.ax, R*cos(th), R*sin(th), ':', 'Color',[0.4 0.4 0.45]);
    for k = 1:size(cfg.obstacles.circles,1)
        c = cfg.obstacles.circles(k,:);
        viscircles(c(1:2), c(3), 'Color',[1 0.45 0.3], 'LineWidth',1.2);
    end
    for k = 1:size(cfg.obstacles.rects,1)
        r = cfg.obstacles.rects(k,:);
        c = cos(r(3)); sg = sin(r(3));
        hw = r(4)/2; hh = r(5)/2;
        corners = [r(1)-hw*c+hh*sg, r(2)-hw*sg-hh*c;
                   r(1)+hw*c+hh*sg, r(2)+hw*sg-hh*c;
                   r(1)+hw*c-hh*sg, r(2)+hw*sg+hh*c;
                   r(1)-hw*c-hh*sg, r(2)-hw*sg+hh*c];
        patch(s.ax, corners(:,1), corners(:,2), [1 0.5 0.3], ...
            'FaceAlpha',0.25,'EdgeColor',[1 0.45 0.3],'LineWidth',1.2);
    end
    xt = cfg.X_target(1); yt = cfg.X_target(2); tt = cfg.theta_target;
    plot(s.ax, xt, yt, 'g+', 'MarkerSize', 12, 'LineWidth', 2);
    quiver(s.ax, xt, yt, 0.25*cos(tt), 0.25*sin(tt), 0, 'g', 'LineWidth', 1.5);
    if isfield(s, 'hDx') && ishandle(s.hDx)
        dx = str2double(get(s.hDx,'String')); dy = str2double(get(s.hDy,'String'));
        if isfinite(dx) && isfinite(dy)
            plot(s.ax, dx, dy, 'ms', 'MarkerSize', 9, 'LineWidth', 1.5);
            text(s.ax, dx+0.08, dy+0.08, '放置', 'Color',[1 0.6 1], 'FontSize', 7);
        end
    end
    [p_all, ~] = planarFK_L(s.q, s.model.DH, cfg.rod_offset_arr);
    p_nodes = planarFK_SimpleNode(s.q, s.model.DH, cfg.rod_offset_arr);
    hArmNode = plot(s.ax, p_nodes(:,1), p_nodes(:,2), 'o-', 'Color',[0.3 0.75 1], ...
        'MarkerFaceColor',[0.3 0.75 1],'MarkerSize',5,'LineWidth',2.2);
    hArmDot  = plot(s.ax, p_all(:,1), p_all(:,2), ':', 'Color',[0.5 0.6 0.7]);
    hArmEnd  = plot(s.ax, p_nodes(end,1), p_nodes(end,2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor','r');
    g_cur = 0;
    if ~isempty(s.gripperSeq) && s.snapIdx >= 1 && s.snapIdx <= numel(s.gripperSeq)
        g_cur = s.gripperSeq(s.snapIdx);
    end
    gname = {'保持','张开','闭合'};
    gc = [0.7 0.7 0.7; 0.3 1 0.3; 1 0.4 0.4];
    hGrip = text(s.ax, 0.2, R+0.1, sprintf('夹爪: %s', gname{g_cur+1}), ...
        'Color', gc(g_cur+1,:), 'FontSize', 10, 'FontWeight','bold');
    % 保存静态臂/夹爪句柄，供 onPlay 播放前删除（避免播放时“静态臂+动画臂”双重显示）
    s.hArm = [hArmNode hArmDot hArmEnd hGrip];
    % 轨迹线（碰撞安全插值末端路径）
    if isfield(s,'trajPts') && ~isempty(s.trajPts) && size(s.trajPts,1) > 1
        plot(s.ax, s.trajPts(:,1), s.trajPts(:,2), '-', 'Color',[1 0.85 0.3], 'LineWidth',1.0);
    end
    % 视图自适应：按全部物体（臂/障碍/目标/放置点/轨迹/可达圆）自动缩放平移
    [xmin, xmax, ymin, ymax] = computeViewBounds(s);
    cx = (xmin + xmax)/2;  cy = (ymin + ymax)/2;
    half = max(xmax - xmin, ymax - ymin)/2;
    xlim(s.ax, [cx - half, cx + half]);
    ylim(s.ax, [cy - half, cy + half]);
    grid(s.ax,'on');
    % 关键：绘图对象关闭 HitTest——点击任何图形（臂线/障碍/十字）都穿透到 axes，
    % 保证 onAxClick 拖拽命中检测始终触发（否则事件被图形对象吃掉）
    set(findall(s.ax), 'HitTest', 'off');
    set(s.ax, 'HitTest', 'on');   % axes 自身保持可接收（子对象穿透到这里）

    % 走廊图叠加显示（method_graph 求解后）：边淡灰、节点青点、A* 序列亮黄
    if isfield(s, 'graphDisp') && ~isempty(s.graphDisp) && ~isempty(s.graphDisp.nodes)
        gd = s.graphDisp;
        if ~isempty(gd.edges)
            for k = 1:size(gd.edges, 1)
                plot(s.ax, gd.edges(k,[1 3]), gd.edges(k,[2 4]), '-', ...
                    'Color',[0.55 0.55 0.6 0.30], 'LineWidth',0.8);
            end
        end
        if ~isempty(gd.seq)
            plot(s.ax, gd.nodes(gd.seq,1), gd.nodes(gd.seq,2), '-', ...
                'Color',[1 0.85 0.2], 'LineWidth',1.8);
        end
        plot(s.ax, gd.nodes(:,1), gd.nodes(:,2), 'o', ...
            'Color',[0.4 0.9 0.9], 'MarkerSize',4, 'MarkerFaceColor',[0.4 0.9 0.9]);
    end
end

function [xmin, xmax, ymin, ymax] = computeViewBounds(s)
    % 视图包围盒：臂（当前构型）+ 障碍 + 目标 + 放置点 + 轨迹 + 基座 + 可达参考圆
    cfg = s.model.cfg;
    px = [];  py = [];
    [p_all, ~] = planarFK_L(s.q, s.model.DH, cfg.rod_offset_arr);
    px = [px; p_all(:,1)];  py = [py; p_all(:,2)]; %#ok<AGROW>
    for k = 1:size(cfg.obstacles.circles,1)
        c = cfg.obstacles.circles(k,:);
        px = [px; c(1)-c(3); c(1)+c(3)];  py = [py; c(2)-c(3); c(2)+c(3)]; %#ok<AGROW>
    end
    for k = 1:size(cfg.obstacles.rects,1)
        r = cfg.obstacles.rects(k,:);
        ct = cos(r(3)); st = sin(r(3));  hw = r(4)/2; hh = r(5)/2;
        corners = [r(1)-hw*ct+hh*st, r(2)-hw*st-hh*ct;
                   r(1)+hw*ct+hh*st, r(2)+hw*st-hh*ct;
                   r(1)+hw*ct-hh*st, r(2)+hw*st+hh*ct;
                   r(1)-hw*ct-hh*st, r(2)-hw*st+hh*ct];
        px = [px; corners(:,1)];  py = [py; corners(:,2)]; %#ok<AGROW>
    end
    px = [px; cfg.X_target(1)];  py = [py; cfg.X_target(2)]; %#ok<AGROW>
    if isfield(s,'hDx') && ishandle(s.hDx)
        dx = str2double(get(s.hDx,'String'));  dy = str2double(get(s.hDy,'String'));
        if isfinite(dx) && isfinite(dy), px = [px; dx]; py = [py; dy]; end %#ok<AGROW>
    end
    if isfield(s,'trajPts') && ~isempty(s.trajPts)
        px = [px; s.trajPts(:,1)];  py = [py; s.trajPts(:,2)]; %#ok<AGROW>
    end
    R = cfg.N * cfg.L_seg(1);
    % 基座(0,0) + 可达参考圆（半径 R）四向边界，保证参考圆与整臂始终可见
    px = [px; 0;  R; -R;  0;  0]; %#ok<AGROW>
    py = [py; 0;  0;  0;  R; -R]; %#ok<AGROW>
    m = 0.15*R + 0.2;              % 边距
    xmin = min(px) - m;  xmax = max(px) + m;
    ymin = min(py) - m;  ymax = max(py) + m;
end

%% ---------- 关节角回调 ----------
function onQjEdit(src, ~)
    s = guidata(ancestor(src,'figure'));
    idx = find(s.hQj == src, 1);
    if isempty(idx), return; end
    v = str2double(get(src,'String'));
    if ~isfinite(v), return; end
    s.q(idx) = max(s.model.cfg.q_min(idx), min(s.model.cfg.q_max(idx), v));
    set(src,'String',num2str(s.q(idx),'%.3f'));
    s = drawAll(s);
    guidata(ancestor(src,'figure'), s);
end

function onSetLevel(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    s = rebuildModel(s);
    s.q = zeros(1, s.model.cfg.N);   % 全水平：所有关节 0
    s = syncJointEditors(s);
    s = drawAll(s);
    guidata(f, s);
end

function onSetFold(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    s = rebuildModel(s);
    N = s.model.cfg.N;
    q = zeros(1, N);
    q(1:2:N) =  3.142;    % 奇数关节 +π
    q(2:2:N) = -3.142;    % 偶数关节 -π（相邻交替，之字折叠最紧凑）
    q = max(s.model.cfg.q_min, min(s.model.cfg.q_max, q));
    s.q = q;
    s = syncJointEditors(s);
    s = drawAll(s);
    guidata(f, s);
end

function onFitView(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    s = drawAll(s);   % drawAll 已内置「适应视图」逻辑（按全部物体自动缩放）
    guidata(f, s);
end

function onParamEdit(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    % 参数（N/杆长/限位/权重等）改变 → 旧轨迹/求解结果失效（N/杆长变化会致维度不匹配回放崩溃）
    s.graphDisp = [];  s.snap = [];  s.trajQ = [];  s.trajPts = [];  s.gripperSeq = [];  s.snapIdx = 1;  s.info = [];
    s = rebuildModel(s);          % 重新读参数建模型
    s = syncJointEditors(s);      % N 变化时关节编辑器重建
    s = drawAll(s);               % 重置图像与参数一致
    guidata(f, s);
end

%% ---------- 单次求解回调 ----------
function onRun(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    try
        s = rebuildModel(s);
    catch e
        s = setLog(s, ['参数错误: ' e.message]);
        guidata(f, s); return;
    end
    method = {'auto','momentum','sa','rrt','rrtstar','prm','graph','rl','cvae','multilayer'};
    m = method{get(s.hMethod,'Value')};
    target = [s.model.cfg.X_target, s.model.cfg.theta_target];
    t0 = tic;
    q_start = s.q;   % 记录求解前臂位姿：回放锚定到真实起点 q0
    if strcmp(m, 'multilayer')
        % 多层：骨架连通图 → 候选路径(k-shortest) → 逐段求解（快速+稳健回退）→ 末尾精修
        % 开关：GUI 勾选“强迫多层(use_multilayer)”控制是否走多层；取消→多层退化为单段兜底
        opts = struct('use_multilayer', get(s.hMulti,'Value'), ...
            'k_paths', 2, 'max_segments', 9, 'GRID', 64, 'Snapshot', 1);
        info = method_multilayer(s.model, s.q, target, opts);
    elseif strcmp(m, 'cvae')
        % L2 策略推理（部署闭环：策略优先 + 安全回退）
        pfile = get(s.hPolicyFile, 'String');
        try
            out = cvaePolicyDeploy(pfile, s.model, s.q, target);
        catch e
            s = setLog(s, ['CVAE 策略失败: ' e.message]);
            guidata(f, s); return;
        end
        [~, pe] = planarFK_L(out.q_final, s.model.DH, s.model.cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(out.q_final, s.model.DH, s.model.cfg.rod_offset_arr);
        info = struct('q_snapshot', out.traj, 't_seq', (1:size(out.traj,1))', ...
            'V_hist', [], 'q_final', out.q_final, ...
            'converged', false, 'cancelled', false, 'iter', size(out.traj,1), ...
            'dist_end', norm(pe - target(1:2)), ...
            'err_ang', abs(wrapAngle(target(3) - th)), ...
            'method_used', ['cvae(' out.via ')'], ...
            'stats', struct('policy_file', pfile));
        info.success = info.dist_end < 0.1 && info.err_ang < 0.2;
        if info.success, info.error_code = 0; else, info.error_code = 2; end
    else
        % 采样预算：自适应（按场景难度）或用户手动指定
        rkv = rrtKV(s);   % RRT/RRT* 参数（采样方法专用；对其他方法无害，方法用 of() 自取）
        if isfield(s,'hAdaptive') && ishandle(s.hAdaptive) && get(s.hAdaptive,'Value')
            b0 = adaptiveBudget(s.model, s.q, target);
            info = simulateMotion(s.model, m, s.q, target, 'Snapshot', 1, ...
                'max_samples', b0, 'strict', get(s.hStrict,'Value'), rkv{:});
            if ~info.success   % 失败升档（×2 一次）
                info = simulateMotion(s.model, m, s.q, target, 'Snapshot', 1, ...
                    'max_samples', min(b0*2, s.model.cfg.rrt_max_samples), ...
                    'strict', get(s.hStrict,'Value'), rkv{:});
            end
        else
            info = simulateMotion(s.model, m, s.q, target, 'Snapshot', 1, ...
                'max_samples', max(500, round(str2double(get(s.hSamp,'String')))), ...
                'strict', get(s.hStrict,'Value'), rkv{:});
        end
    end
    dt = toc(t0);
    s.q = info.q_final;
    s.snap = info.q_snapshot;
    % 回放锚定：确保轨迹覆盖「真实起点 → 终点」（momentum 等快照从 snapshot_m 步起，缺 q0）
    qf0 = info.q_final;
    if isempty(s.snap) || size(s.snap,1) < 1
        s.snap = q_start;
    elseif norm(s.snap(1,:) - q_start) > 1e-6
        s.snap = [q_start; s.snap];
    end
    if size(s.snap,1) >= 1 && norm(s.snap(end,:) - qf0) > 1e-6
        s.snap = [s.snap; qf0];
    end
    s.gripperSeq = [];
    s.snapIdx = 1;
    s.info = info;
    % 走廊图叠加显示：graph 方法（或 auto 走 graph）求解后保存节点/边/A* 序列
    if isfield(info,'stats') && isfield(info.stats,'nodes_xy') && ~isempty(info.stats.nodes_xy)
        s.graphDisp = struct('nodes', info.stats.nodes_xy, ...
            'edges', info.stats.edges_xy, 'seq', info.stats.seq_used);
    else
        s.graphDisp = [];
    end
    log = sprintf('单次求解: 方法=%s | success=%d | error_code=%d\npos=%.4f m | ang=%.4f rad | 耗时=%.2fs', ...
        m, info.success, info.error_code, info.dist_end, info.err_ang, dt);
    % 距收敛差距：按【实际执行方法】取阈值（auto 会变成 momentum/rrtstar/graph 等；
    % 之前按用户选择 m 取导致 auto+rrtstar 时显示 tol_pos=1e-4 而实际判据是 0.1 的矛盾）
    mu = info.method_used;
    if contains(mu, 'momentum') || contains(mu, 'sa')
        tp = s.model.cfg.tol_pos;  ta = s.model.cfg.tol_ang;
    elseif contains(mu, 'cvae')
        tp = 0.05;  ta = 0.2;   % 直出+精修后部署端 pos≤0.01，显示阈值收紧到 0.05
    elseif contains(mu, 'multilayer')
        tp = 0.03;  ta = 0.3;   % 多层判定：末尾精修后 pos<0.03m、角度放开仅报告
    else   % rrt / rrtstar / prm / graph / auto(全部失败) 兜底：采样类阈值
        tp = s.model.cfg.rrt_goal_eps;  ta = s.model.cfg.rrt_goal_ang;
    end
    if info.success
        if isfield(info,'conv_type') && strcmp(info.conv_type,'item')
            log = [log sprintf('\n已收敛 ✓（二次精修：目标函数逐项变化 < 0.001）')]; %#ok<AGROW>
        else
            log = [log sprintf('\n已收敛 ✓（pos ≤ %.4g | ang ≤ %.4g）', tp, ta)]; %#ok<AGROW>
        end
    else
        log = [log sprintf('\n距收敛: pos ×%.1f 倍 | ang ×%.1f 倍（阈值 %.4g / %.4g）', ...
            info.dist_end/max(tp,1e-12), info.err_ang/max(ta,1e-12), tp, ta)]; %#ok<AGROW>
    end
    if ~info.success
        if isfield(info,'stats') && isfield(info.stats,'collision') && info.stats.collision
            log = [log sprintf('\n[碰撞] 结果侵入障碍（error_code=9），已标记失败，不会导出')]; %#ok<AGROW>
        elseif isfield(info,'stalled') && info.stalled
            log = [log sprintf('\n[卡住] 误差不再改善提前终止（error_code=7），建议切换方法或调整目标')]; %#ok<AGROW>
        else
            log = [log sprintf('\n[未收敛] 迭代耗尽（error_code=2），建议切换方法或调整障碍/目标')]; %#ok<AGROW>
        end
    end
    s = setLog(s, log);
    s = computeTraj(s);
    s = syncJointEditors(s);   % 刷新关节角显示（q 已更新为 q_final）
    s = drawAll(s);
    guidata(f, s);
end

%% ---------- 模拟视觉任务回调 ----------
function onTaskRun(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    try
        s = rebuildModel(s);
    catch e
        s = setLog(s, ['参数错误: ' e.message]);
        guidata(f, s); return;
    end
    ctList = {'move_near_target','pick_target','pick_and_place','dock_to_interface','home'};
    ct = ctList{get(s.hTaskType,'Value')};
    xt = str2double(get(s.hTx,'String')); yt = str2double(get(s.hTy,'String'));
    tt = str2double(get(s.hTth,'String'));
    dx = str2double(get(s.hDx,'String')); dy = str2double(get(s.hDy,'String'));
    cmd = mockTaskCommand(ct, xt, yt, tt, dx, dy);
    if ~isempty(s.taskOutbox)
        if ~exist(s.taskOutbox, 'dir'), mkdir(s.taskOutbox); end
        fid = fopen(fullfile(s.taskOutbox, [cmd.command_id '.json']), 'w');
        fprintf(fid, '%s', jsonencode(cmd)); fclose(fid);
    end
    s = setLog(s, sprintf('模拟视觉任务 %s → RECEIVED → ACCEPTED → PLANNING …', ct));
    drawnow limitrate;
    % 任务模拟遵循 GUI 方法选择（与单次求解策略一致）
    method = {'auto','momentum','sa','rrt','rrtstar','prm','graph','rl','cvae','multilayer'};
    mth = method{get(s.hMethod,'Value')};
    info = taskExecute(s.model, cmd, struct('inbox', s.taskInbox, 'snapshot_m', 1, 'method', mth));
    q_all = []; g_all = []; n_fail = 0;
    for k = 1:numel(info.seg_infos)
        si = info.seg_infos(k);
        % 段失败时 info 可能缺 q_snapshot：跳过该段并计数（不崩）
        if ~isfield(si, 'info') || ~isfield(si.info, 'q_snapshot') || isempty(si.info.q_snapshot)
            n_fail = n_fail + 1;
            continue; %#ok<AGROW>
        end
        qs = si.info.q_snapshot;
        q_all = [q_all; qs]; %#ok<AGROW>
        g_all = [g_all, repmat(si.gripper, 1, size(qs,1))]; %#ok<AGROW>
    end
    % 任务失败分支 motor_cmd 可能为空：判空后取 q_final，否则保持当前构型
    if ~isempty(info.motor_cmd) && isfield(info.motor_cmd,'q_final') && ~isempty(info.motor_cmd.q_final)
        s.q = info.motor_cmd.q_final;
    else
        s.q = zeros(1, s.model.cfg.N);
    end
    s.snap = q_all; s.gripperSeq = g_all; s.snapIdx = 1;
    s.info = info;
    log = sprintf('模拟视觉任务: %s (id=%s) | 方法=%s\n', ct, cmd.command_id, mth);
    if n_fail > 0
        log = [log sprintf('  [警告] %d 个段求解失败已跳过\n', n_fail)]; %#ok<AGROW>
    end
    if ~isempty(s.taskInbox)
        log = [log sprintf('文件桥: outbox→%s | inbox←%s\n', s.taskOutbox, s.taskInbox)]; %#ok<AGROW>
    end
    for k = 1:numel(info.seg_infos)
        si = info.seg_infos(k);
        gname = {'保持','张开','闭合'};
        gv = si.gripper;  if isempty(gv), gv = 0; end
        ok_s = isfield(si,'info') && isfield(si.info,'success') && ~isempty(si.info.success);
        log = [log sprintf('  %s | gripper=%s | success=%d\n', si.name, ... %#ok<AGROW>
            gname{gv+1}, ok_s && si.info.success)];
    end
    log = [log sprintf('最终: %s | error_code=%d\n', info.status, info.error_code)]; %#ok<AGROW>
    if ~isempty(info.seg_infos)
        si = info.seg_infos(end);
        if isfield(si,'info') && isfield(si.info,'dist_end') && ~isempty(si.info.dist_end)
            log = [log sprintf('末端误差: pos=%.4f m | ang=%.4f rad\n', si.info.dist_end, si.info.err_ang)]; %#ok<AGROW>
        end
    end
    s = setLog(s, log);
    s = computeTraj(s);
    s = syncJointEditors(s);
    s = drawAll(s);
    guidata(f, s);
end

function onExportMotor(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    if isempty(s.info) || isempty(s.info.motor_cmd)
        s = setLog(s, '暂无任务结果，先“下发任务并执行”。');
        guidata(f, s); return;
    end
    [fn, pth] = uiputfile({'*.mat','MATLAB 数据 (*.mat)'; '*.csv','CSV 文本 (*.csv)'}, ...
        '导出 motorCmd（电控加载用）', 'motorCmd.mat');
    if isequal(fn, 0), return; end
    fname = fullfile(pth, fn);
    exportMotorCmd(s.info.motor_cmd, fname);
    s = setLog(s, sprintf('motorCmd 已导出: %s\n（dSPACE 经 ControlDesk 加载）', fname));
    guidata(f, s);
end

%% ---------- 障碍编辑回调 ----------
function onObsEdit(src, ~)
    s = guidata(ancestor(src,'figure'));
    s = rebuildModel(s);
    s = drawAll(s);
    guidata(ancestor(src,'figure'), s);
end

function onObsAdd(src, ~)
    s = guidata(ancestor(src,'figure'));
    data = get(s.hObsTable,'Data');
    if get(s.hObsAdd,'Value') == 1
        data(end+1,:) = {'circle', '1.5, 0.6, 0.35'}; %#ok<AGROW>
    else
        data(end+1,:) = {'rect', '1.0, 1.0, 0.3, 0.6, 0.3'}; %#ok<AGROW>
    end
    set(s.hObsTable,'Data',data);
    s = rebuildModel(s);
    s = drawAll(s);
    guidata(ancestor(src,'figure'), s);
end

function onObsDel(src, ~)
    s = guidata(ancestor(src,'figure'));
    data = get(s.hObsTable,'Data');
    if isempty(data), return; end
    data(end,:) = [];
    set(s.hObsTable,'Data',data);
    s = rebuildModel(s);
    s = drawAll(s);
    guidata(ancestor(src,'figure'), s);
end

%% ---------- 随机场景 ----------
function onRandomScene(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    s = randomScene(s);
    guidata(f, s);
end

function s = randomScene(s)
    % 一键随机场景：随机障碍组 + 臂初始姿态 + 目标。
    % 仅随机化场景内容，不修改基础参数（N/杆长/角度区间等——沿用当前模型）。
    s = rebuildModel(s);          % 先从编辑框同步当前基础参数（防 N/杆长等未失焦的改动）
    cfg = s.model.cfg;
    [~, ~, tgt, obs_desc] = sampleTask2D(struct('N', cfg.N, 'L_seg', cfg.L_seg(1), ...
        'difficulty', 2));
    % 写障碍表（circle [x,y,r] / rect [x,y,θ,w,h]）
    data = cell(0, 2);
    for k = 1:size(obs_desc.circles, 1)
        data(end+1, :) = {'circle', vec2str(obs_desc.circles(k, :))}; %#ok<AGROW>
    end
    for k = 1:size(obs_desc.rects, 1)
        data(end+1, :) = {'rect', vec2str(obs_desc.rects(k, :))}; %#ok<AGROW>
    end
    set(s.hObsTable, 'Data', data);
    % 写目标位姿
    set(s.hTx, 'String', num2str(tgt(1), '%.3f'));
    set(s.hTy, 'String', num2str(tgt(2), '%.3f'));
    set(s.hTth, 'String', num2str(tgt(3), '%.3f'));
    % 重建模型（读障碍 + 目标；基础参数保持当前值）
    s = rebuildModel(s);
    % 臂初始姿态：在当前 q_min/q_max 内随机且碰撞自由
    s.q = sampleFreeQ0(s.model);
    % 清空上次求解/回放结果，避免旧轨迹/旧信息残留
    s.snap = [];  s.trajQ = [];  s.trajPts = [];  s.gripperSeq = [];  s.snapIdx = 1;
    s.info = [];  s.graphDisp = [];
    s = syncJointEditors(s);
    s = drawAll(s);
    s = setLog(s, sprintf('🎲 随机场景：%d 圆 %d 矩形 | q0=%s | 目标(%.2f, %.2f, θ=%.2f)', ...
        size(obs_desc.circles,1), size(obs_desc.rects,1), mat2str(s.q, 2), tgt(1), tgt(2), tgt(3)));
end

function q0 = sampleFreeQ0(model)
    % 在当前 q_min/q_max 区间内采样碰撞自由的初始构型（最多 50 次）
    cfg = model.cfg;
    q0 = zeros(1, cfg.N);
    for tries = 1:50
        qc = cfg.q_min + (cfg.q_max - cfg.q_min) .* rand(1, cfg.N);
        g = obsDistAll(model, qc);
        if isempty(g) || min(g) >= cfg.rho0 + 0.12
            q0 = qc; return;
        end
    end
end

function str = vec2str(v)
    str = num2str(v(1), '%.3f');
    for k = 2:numel(v)
        str = [str ', ' num2str(v(k), '%.3f')]; %#ok<AGROW>
    end
end

%% ---------- 回放 / 鼠标 / 键盘 ----------
function onPlay(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    if isempty(s.snap) || size(s.snap,1) < 1
        s = setLog(s, '暂无轨迹，先“求解运动”或“下发任务并执行”。');
        guidata(f, s); return;
    end
    % 自定义回放速度（倍率，>1 更快）
    spd = str2double(get(s.hSpeed,'String'));
    if ~isfinite(spd) || spd <= 0, spd = 1; end
    cfg = s.model.cfg;
    % 使用 computeTraj 生成的碰撞安全插值轨迹（无穿障中间帧）
    if isempty(s.trajQ)
        s = computeTraj(s);
    end
    Qc = s.trajQ;  gIdx = s.trajGIdx;
    m = size(Qc,1);
    if m < 1
        s = setLog(s, '暂无可用轨迹。');
        guidata(f, s); return;
    end
    % 快速回放路径：只更新臂线/末端/夹爪文字，避免整图重建（大幅提速）
    hold(s.ax,'on');
    % 删除上一帧 drawAll 画的静态臂/夹爪文字，避免“静态臂+动画臂”双重显示（幽灵臂）
    if isfield(s,'hArm') && ~isempty(s.hArm), delete(s.hArm); s.hArm = []; end
    hA = plot(s.ax, NaN, NaN, 'o-', 'Color',[0.3 0.75 1], ...
        'MarkerFaceColor',[0.3 0.75 1],'MarkerSize',5,'LineWidth',2.2);
    gname = {'保持','张开','闭合'};
    gc = [0.7 0.7 0.7; 0.3 1 0.3; 1 0.4 0.4];
    hT = text(s.ax, 0.2, cfg.N*cfg.L_seg(1)+0.1, '夹爪: 保持', ...
        'Color',[0.7 0.7 0.7], 'FontSize', 10, 'FontWeight','bold');
    set(hA,'HitTest','off');  set(hT,'HitTest','off');   % 播放动画不拦截鼠标
    for k = 1:m
        if ~ishandle(f), return; end
        p = planarFK_SimpleNode(Qc(k,:), s.model.DH, cfg.rod_offset_arr);
        set(hA, 'XData', p(:,1), 'YData', p(:,2));
        g = 0;
        if ~isempty(s.gripperSeq), g = s.gripperSeq(gIdx(k)); end
        set(hT, 'String', sprintf('夹爪: %s', gname{g+1}), 'Color', gc(g+1,:));
        s.q = Qc(k,:);
        s.snapIdx = gIdx(k);
        drawnow limitrate;
        pause(0.05 / spd / (m / max(1, size(s.snap,1))));   % 总时长与快照帧率解耦
    end
    delete(hA); delete(hT);
    s.snapIdx = max(1, size(s.snap,1));
    s = drawAll(s);   % 恢复完整视图（含末端轨迹线）
    guidata(f, s);
end

function onAxClick(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    cp = get(s.ax,'CurrentPoint');
    if isfield(s,'dragTestPos') && ~isempty(s.dragTestPos), cp = s.dragTestPos; end   % 测试钩子
    x = cp(1,1); y = cp(1,2);
    cfg = s.model.cfg;
    s.dragMode = 'none';  s.dragRow = 0;
    % 命中检测优先级：目标点 > 放置点 > 障碍
    if norm([x,y] - cfg.X_target) < 0.25
        s.dragMode = 'target';
    else
        if isfield(s,'hDx') && ishandle(s.hDx)
            dx = str2double(get(s.hDx,'String')); dy = str2double(get(s.hDy,'String'));
            if isfinite(dx) && isfinite(dy) && norm([x,y]-[dx,dy]) < 0.25
                s.dragMode = 'place';
            end
        end
        if strcmp(s.dragMode,'none')
            s.dragRow = hitObstacle(s, x, y);
            if s.dragRow > 0
                s.dragMode = 'obs';
            end
        end
    end
    if ~strcmp(s.dragMode,'none')
        set(f,'WindowButtonMotionFcn',@onDrag);
        set(f,'WindowButtonUpFcn',@onDrop);
    end
    guidata(f, s);
end

function row = hitObstacle(s, x, y)
    % 命中障碍 → 返回障碍表行号（1 起）；未命中 → 0
    row = 0;
    cfg = s.model.cfg;
    for k = 1:size(cfg.obstacles.circles,1)
        c = cfg.obstacles.circles(k,:);
        if norm([x,y]-c(1:2)) < c(3)+0.2, row = k; return; end
    end
    nc = size(cfg.obstacles.circles,1);
    for k = 1:size(cfg.obstacles.rects,1)
        r = cfg.obstacles.rects(k,:);
        ct = cos(r(3)); st = sin(r(3));
        lx = (x-r(1))*ct + (y-r(2))*st;
        ly = -(x-r(1))*st + (y-r(2))*ct;
        if abs(lx) < r(4)/2+0.2 && abs(ly) < r(5)/2+0.2, row = nc+k; return; end
    end
end

function onDrag(src, ~)
    f = ancestor(src,'figure');
    s = guidata(f);
    if isempty(s.dragMode) || strcmp(s.dragMode,'none'), return; end
    cp = get(s.ax,'CurrentPoint');
    if isfield(s,'dragTestPos') && ~isempty(s.dragTestPos), cp = s.dragTestPos; end   % 测试钩子
    R = s.model.cfg.N * s.model.cfg.L_seg(1);
    x = max(-0.5, min(R+0.5, cp(1,1)));
    y = max(-0.8, min(R+0.5, cp(1,2)));
    switch s.dragMode
        case 'target'
            set(s.hTx,'String',num2str(x,'%.3f'));
            set(s.hTy,'String',num2str(y,'%.3f'));
            s = rebuildModel(s);
        case 'place'
            set(s.hDx,'String',num2str(x,'%.3f'));
            set(s.hDy,'String',num2str(y,'%.3f'));
            % 放置点仅存于 GUI 编辑框（不入模型），无需 rebuild
        case 'obs'
            s = updateObsFromDrag(s, x, y);
    end
    s = drawAll(s);
    guidata(f, s);
end

function s = updateObsFromDrag(s, x, y)
    % 拖拽障碍：更新障碍表对应行的 (x,y)，保留其余参数，重建模型
    row = s.dragRow;
    data = get(s.hObsTable,'Data');
    if row < 1 || row > size(data,1), return; end
    v = str2num(data{row,2}); %#ok<ST2NM>
    if isempty(v), return; end
    v(1) = x;  v(2) = y;
    data{row,2} = sprintf('%.3f', v(1));
    for k = 2:numel(v)
        data{row,2} = [data{row,2} ', ' num2str(v(k),'%.3f')]; %#ok<AGROW>
    end
    set(s.hObsTable,'Data',data);
    s = rebuildModel(s);
end

function onDrop(src, ~)
    s = guidata(ancestor(src,'figure'));
    s.dragMode = 'none';
    s.dragRow = 0;
    set(ancestor(src,'figure'),'WindowButtonMotionFcn','');
    set(ancestor(src,'figure'),'WindowButtonUpFcn','');
    guidata(ancestor(src,'figure'), s);
end

function onKey(src, evt)
    s = guidata(ancestor(src,'figure'));
    switch evt.Key
        case 'return',  onRun(src,[]);
        case 'space',   onPlay(src,[]);
    end
end

function onClose(src, ~)
    delete(ancestor(src,'figure'));
end

%% ---------- 工具 ----------
function cmd = mockTaskCommand(ct, xt, yt, tt, dx, dy)
    cmd = struct();
    cmd.schema_version = '1.0';
    cmd.command_id = sprintf('CMD-GUI-%s', datestr(now,'HHMMSS'));
    cmd.timestamp = posixtime(datetime('now'));
    cmd.source = 'SpaceSnakeVisionUI(mock)';
    cmd.command_type = ct;
    cmd.selected_target = struct('target_id','TGT-GUI','class_name','payload_module', ...
        'confidence', 0.95, ...
        'pose_camera', struct('frame_id','camera_left', ...
            'position', struct('x',xt,'y',yt,'z',0.0), ...
            'orientation_quat', struct('x',0,'y',0,'z',0,'w',1)), ...
        'pose_base', []);
    cmd.destination = struct('name','Assembly_Port_A', ...
        'pose_base', struct('frame_id','robot_base', ...
            'position', struct('x',dx,'y',dy,'z',0.0), ...
            'orientation_quat', struct('x',0,'y',0,'z',0,'w',1)));
    cmd.motion_params = struct('approach_distance_m', 0.15, ...
        'gripper_mode', 'demo_grip', 'speed_mode', 'normal');
    cmd.safety = struct('require_user_confirm', false, ...
        'allow_execute', true, 'estop_active', false);
end

function h = sectionTitle(hp, py, str)
    h = uicontrol(hp,'Style','text','String',str,'Units','normalized', ...
        'Position',[0.03 py-0.018 0.94 0.015],'Background',[0.14 0.15 0.18], ...
        'Foreground',[1 0.7 0.3],'FontSize',7,'HorizontalAlignment','center');
end

function [h1, h2, py, hAll] = twoCol(hp, py, rH, lab1, val1, lab2, val2)
    hl1 = uicontrol(hp,'Style','text','String',lab1,'Units','normalized', ...
        'Position',[0.03 py-rH 0.24 rH],'Background',[0.14 0.15 0.18], ...
        'Foreground',[0.85 0.85 0.95],'FontSize',7,'HorizontalAlignment','left');
    h1 = uicontrol(hp,'Style','edit','String',val1,'Units','normalized', ...
        'Position',[0.27 py-rH 0.20 rH],'Background',[0.2 0.2 0.3], ...
        'Foreground',[1 1 1],'FontSize',8);
    hl2 = [];  h2 = [];
    if ~isempty(lab2)
        hl2 = uicontrol(hp,'Style','text','String',lab2,'Units','normalized', ...
            'Position',[0.50 py-rH 0.24 rH],'Background',[0.14 0.15 0.18], ...
            'Foreground',[0.85 0.85 0.95],'FontSize',7,'HorizontalAlignment','left');
        h2 = uicontrol(hp,'Style','edit','String',val2,'Units','normalized', ...
            'Position',[0.74 py-rH 0.22 rH],'Background',[0.2 0.2 0.3], ...
            'Foreground',[1 1 1],'FontSize',8);
    end
    hAll = [hl1 h1 hl2 h2];   % 本行创建的全部句柄（label1/edit1/label2/edit2，空则省略）
    py = py - rH - 0.005;
end

function s = setLog(s, msg)
    if ~isfield(s,'hLog') || ~ishandle(s.hLog), return; end
    lines = strsplit(msg, sprintf('\n'));
    set(s.hLog, 'String', lines(:), 'Value', numel(lines));   % listbox 自动滚动到底
end

function kv = rrtKV(s)
% rrtKV 从「RRT/RRT* 参数」编辑框读取采样方法专用参数，返回 {'Key',val,...} 供 varargin 透传
%   仅对 rrt / rrtstar（以及内部调它们的 auto/graph）有意义；其它方法用 of() 自行忽略未知键
    cfg = s.model.cfg;
    mstep = gnum(s, 'hMaxStep', cfg.rrt_max_step);
    geps  = gnum(s, 'hGoalEps', cfg.rrt_goal_eps);
    gan   = gnum(s, 'hGoalAng', cfg.rrt_goal_ang);
    rf    = gnum(s, 'hRfSteps', 150);
    gam   = gnum(s, 'hRGamma', 2.5);
    kv = {'max_step', mstep, 'goal_eps', geps, 'goal_ang', gan, ...
          'rf_steps', round(rf), 'rrt_gamma', gam};
end

function v = gnum(s, f, d)
% gnum 读编辑框数值，非法/非正则回退到默认 d
    v = str2double(get(s.(f),'String'));
    if ~isfinite(v) || v <= 0, v = d; end
end
