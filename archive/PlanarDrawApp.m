%% PlanarDrawApp.m — 平面绘图程序
%  调用方式：
%    app = PlanarDrawApp();
%    app = PlanarDrawApp('N', 6, 'L', 0.15);
%
function app = PlanarDrawApp(varargin)
    if nargin > 0, GlobalParams(varargin{:}); end
    gp = GlobalParams();
    N = gp.N; L_seg = gp.L_seg;
    udpPort = gp.udp_port; framerate = gp.framerate;
    max_reach = N * L_seg;
    
    %% 持久状态
    s = struct();
    s.N=N; s.L_seg=L_seg; s.q=zeros(1,N);
    s.X_tgt=[max_reach*0.6, max_reach*0.4];
    s.mode='debug'; s.drawOn=false; s.segDrawOn=false; s.segPt=[]; s.segLines={};
    s.drawLines={}; s.drawCurLine=[]; s.drawIsDrawing=false;
    s.ifActive=false; s.udpPort=udpPort; s.udpSock=[]; s.uCount=0; s.uBytes=0;
    s.dragMode='none'; s.dragIdx=0;
    s.simRunning=false; s.simStopReq=false; s.simIter=0; s.simDist=Inf; s.simConverged=false;
    s.theta_tgt=0;
    % 重播状态
    s.replayFrames = {};   % 录制的关节角序列 {q1, q2, ...}
    s.replayActive = false;
    s.replayIdx = 0;
    s.replaySpeed = 2;     % 每 tick 跳过的帧数
    s.paramMode='custom';
    s.faultMotors=false(1,N); s.freeMotorCount=false; s.freeAngleStep=false; s.zeroBaseError=false;
    s.savedParams=struct();
    s.saved_q = zeros(1,N);
    s.poseRunning=false; s.poseStopReq=false; s.poseIter=0; s.poseConverged=false;
    s.pose_tgt_q=zeros(1,N); s.pose_sigma_pos=0.02; s.pose_sigma_ang=0.1;
    s.pose_lambda_joint=0.1; s.pose_use_rrt=false;
    if isempty(gp.m_arr), s.m_arr=zeros(1,N); else, s.m_arr=gp.m_arr(:)'; end
    if isempty(gp.sig0_arr), s.sig0_arr=[1,0.06*ones(1,N-1)]; else, s.sig0_arr=gp.sig0_arr(:)'; end
    if isempty(gp.tau_arr), s.tau_arr=[1,0.7*ones(1,N-1)]; else, s.tau_arr=gp.tau_arr(:)'; end
    
    %% 构建界面
    fig = figure('Name','PlanarDrawApp','NumberTitle','off','Units','normalized',...
        'Position',[0.05 0.04 0.90 0.94],'Color',[0.15 0.15 0.15],...
        'CloseRequestFcn',@onClose,'KeyPressFcn',@onKey);
    
    % --- 左侧参数面板 ---
    hParamPanel = uipanel('Parent',fig,'Units','normalized',...
        'Position',[0.02 0.02 0.23 0.95],'Background',[0.14 0.14 0.18],...
        'Title','参数菜单','Foreground',[0.9 0.9 1],'FontSize',9);
    
    rH = 0.032;   % 紧凑行高
    hC = {};      % 自定义模式专属控件列表
    
    py = 0.95;
    
    % === 模式切换 ===
    uicontrol(hParamPanel,'Style','text','String','菜单模式:','Units','normalized',...
        'Position',[0.03 py-rH 0.26 rH],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.8 0.5],'FontSize',8,'HorizontalAlignment','left');
    hParamMenuMode = uicontrol(hParamPanel,'Style','popupmenu',...
        'String',{'自定义模式','预设模式'},'Value',1,...
        'Units','normalized','Position',[0.30 py-rH 0.63 rH],...
        'Callback',@onParamModeChange,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    py = py - rH - 0.005;
    
    % === 自定义参数行（紧凑排列） ===
    [hN, hL, py] = addTwoCol(hParamPanel,py,rH,'N:',num2str(N),'L(m):',num2str(L_seg),hC);
    hParamN=hN; hParamL=hL;
    [hQmin, hQmax, py] = addTwoCol(hParamPanel,py,rH,'关节下限:',num2str(gp.q_min),'关节上限:',num2str(gp.q_max),hC);
    hParamQmin=hQmin; hParamQmax=hQmax;
    [hSigM, hKap, py] = addTwoCol(hParamPanel,py,rH,'方差下限:',num2str(gp.sig_min2),'电机精度:',num2str(gp.kappa),hC);
    hParamSigMin=hSigM; hParamKappa=hKap;
    [hDamp, hGamS, py] = addTwoCol(hParamPanel,py,rH,'阻尼系数:',num2str(gp.lambda_damp),'位置权重:',num2str(gp.gamma_soft),hC);
    hParamDamp=hDamp; hParamGammaS=hGamS;
    [hLamM, hSigW, py] = addTwoCol(hParamPanel,py,rH,'收敛阈值:',num2str(gp.lambda_m),'角度衰减:',num2str(gp.sigma_weight),hC);
    hParamLambdaM=hLamM; hParamSigmaW=hSigW;
    [hSig0, hTau, py] = addTwoCol(hParamPanel,py,rH,'方差系数sig0:','1','方差宽度tau:','1',hC);
    hParamSig0=hSig0; hParamTau=hTau;
    [hAngB, hAngP, py] = addTwoCol(hParamPanel,py,rH,'角度基础权重:',num2str(gp.gamma_ang_base),'角度峰值权重:',num2str(gp.gamma_ang_peak),hC);
    hParamAngBase=hAngB; hParamAngPeak=hAngP;
    [hRrtTrees, hRrtSamp, py] = addTwoCol(hParamPanel,py,rH,'树数:',num2str(gp.rrt_num_trees),'最大采样:',num2str(gp.rrt_max_samples),hC);
    hParamRrtTrees=hRrtTrees; hParamRrtSamp=hRrtSamp;
    [hRrtStep, hRrtEps, py] = addTwoCol(hParamPanel,py,rH,'步长:',num2str(gp.rrt_max_step),'目标容差:',num2str(gp.rrt_goal_eps),hC);
    hParamRrtStep=hRrtStep; hParamRrtEps=hRrtEps;
    [hRrtStarIter, hRrtStarRad, py] = addTwoCol(hParamPanel,py,rH,'RRT*迭代:',num2str(gp.rrt_star_max_iter),'邻居半径:',num2str(gp.rrt_star_radius),hC);
    hParamRrtStarIter=hRrtStarIter; hParamRrtStarRad=hRrtStarRad;
    % === 障碍对数屏障 ===
    uicontrol(hParamPanel,'Style','text','String','── 障碍屏障 ──','Units','normalized',...
        'Position',[0.03 py-0.022 0.90 0.018],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.6 0.3],'FontSize',7,'HorizontalAlignment','center');
    py = py - 0.028;
    [hObsSig, hObsGam, py] = addTwoCol(hParamPanel,py,rH,'屏障上限C:',num2str(gp.barrier_C),'动量β:',num2str(gp.momentum_beta),hC);
    hParamObsSigma=hObsSig; hParamObsGamma=hObsGam;


    % RRT 单独一行
    cT = uicontrol(hParamPanel,'Style','text','String','RRT:','Units','normalized',...
        'Position',[0.03 py-rH 0.15 rH],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.8 0.8],'FontSize',7,'HorizontalAlignment','left');
    hParamRRT = uicontrol(hParamPanel,'Style','popupmenu',...
        'String',{'关闭','多启动RRT','RRT*最优','PRM*最优','FMM+DAG'},'Value',1,...
        'Units','normalized','Position',[0.19 py-rH 0.40 rH],...
        'Callback',@onReconfig,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    hC{end+1}=cT; hC{end+1}=hParamRRT;
    py = py - rH - 0.005;
    
    % === 运动经济性权重 ===
    uicontrol(hParamPanel,'Style','text','String','────────────────','Units','normalized',...
        'Position',[0.03 py-0.022 0.90 0.018],'Background',[0.14 0.14 0.18],...
        'Foreground',[0.4 0.8 0.6],'FontSize',7,'HorizontalAlignment','center');
    py = py - 0.030;
    
    uicontrol(hParamPanel,'Style','text','String','关节代价:','Units','normalized',...
        'Position',[0.03 py-rH 0.17 rH],'Background',[0.14 0.14 0.18],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hParamLambdaPart = uicontrol(hParamPanel,'Style','edit','String',num2str(gp.lambda_part),...
        'Units','normalized','Position',[0.21 py-rH 0.24 rH],...
        'Callback',@onReconfig,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(hParamPanel,'Style','text','String','激活代价:','Units','normalized',...
        'Position',[0.48 py-rH 0.17 rH],'Background',[0.14 0.14 0.18],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hParamLambdaAct = uicontrol(hParamPanel,'Style','edit','String',num2str(gp.lambda_activate),...
        'Units','normalized','Position',[0.65 py-rH 0.27 rH],...
        'Callback',@onReconfig,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    py = py - rH - 0.003;
    
    uicontrol(hParamPanel,'Style','text','String','转动代价:','Units','normalized',...
        'Position',[0.03 py-rH 0.17 rH],'Background',[0.14 0.14 0.18],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hParamLambdaMotor = uicontrol(hParamPanel,'Style','edit','String',num2str(gp.lambda_motor),...
        'Units','normalized','Position',[0.21 py-rH 0.24 rH],...
        'Callback',@onReconfig,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    py = py - rH - 0.006;
    
    % === 预设模式面板 ===
    % 故障电机多行布局：每行最多4个, 行数=ceil(N/4)
    nMotorCols = 4;
    nMotorRows = ceil(N / nMotorCols);
    motorRowH = 0.08;        % 每行高度
    presetPanelH = 0.22 + nMotorRows * motorRowH;  % 自适应面板高度
    hPresetPanel = uipanel('Parent',hParamPanel,'Units','normalized',...
        'Position',[0.01 py-presetPanelH-0.02 0.98 presetPanelH],'Visible','off',...
        'Background',[0.08 0.08 0.12],'BorderType','line','HighlightColor',[0.3 0.6 0.3],...
        'Title','预设模式开关','Foreground',[0.5 1 0.5],'FontSize',8);
    ppy = 0.94;  % 面板内 top 位置
    colX = [0.02, 0.26, 0.50, 0.74];  % 4 列 x 位置
    colW = 0.22;                        % 每列宽度
    uicontrol(hPresetPanel,'Style','text','String','故障电机:','Units','normalized',...
        'Position',[0.02 ppy-motorRowH 0.24 motorRowH],'Background',[0.08 0.08 0.12],...
        'Foreground',[1 0.6 0.6],'FontSize',8,'HorizontalAlignment','left');
    ppy = ppy - motorRowH;
    hFaultChecks = zeros(1,N);
    for j = 1:N
        row = ceil(j / nMotorCols) - 1;  % 0-based
        col = mod(j-1, nMotorCols);
        xp = colX(col+1);
        yp = ppy - row * motorRowH;
        hFaultChecks(j) = uicontrol(hPresetPanel,'Style','checkbox',...
            'String',sprintf('M%d',j),'Value',0,'Units','normalized',...
            'Position',[xp yp colW motorRowH*0.85],...
            'Callback',@onPresetChange,...
            'Background',[0.08 0.08 0.12],'Foreground',[1 0.6 0.6],'FontSize',7);
    end
    ppy = ppy - nMotorRows * motorRowH - 0.02;
    hPresetFreeCount = uicontrol(hPresetPanel,'Style','checkbox',...
        'String','自由电机数 (激活代价=0)','Value',0,'Units','normalized',...
        'Position',[0.02 ppy-0.06 0.96 0.06],'Callback',@onPresetChange,...
        'Background',[0.08 0.08 0.12],'Foreground',[0.8 1 0.8],'FontSize',8);
    ppy = ppy-0.10;
    hPresetFreeAngle = uicontrol(hPresetPanel,'Style','checkbox',...
        'String','自由角度步长 (转动代价=0)','Value',0,'Units','normalized',...
        'Position',[0.02 ppy-0.06 0.96 0.06],'Callback',@onPresetChange,...
        'Background',[0.08 0.08 0.12],'Foreground',[0.8 1 0.8],'FontSize',8);
    ppy = ppy-0.10;
    hPresetZeroErr = uicontrol(hPresetPanel,'Style','checkbox',...
        'String','零基础误差 (方差代价=0)','Value',0,'Units','normalized',...
        'Position',[0.02 ppy-0.06 0.96 0.06],'Callback',@onPresetChange,...
        'Background',[0.08 0.08 0.12],'Foreground',[0.8 1 0.8],'FontSize',8);
    
    % === 应用按钮 ===
    uicontrol(hParamPanel,'Style','pushbutton','String','应用参数并重建',...
        'Units','normalized','Position',[0.04 0.01 0.92 0.055],...
        'Callback',@onReconfig,'Background',[0.2 0.5 0.2],'Foreground',[1 1 1],...
        'FontWeight','bold','FontSize',9);
    
    % --- 绘图轴 ---
    ax = axes('Parent',fig,'Units','normalized','Position',[0.27 0.08 0.49 0.88],...
        'Color',[0.2 0.2 0.2],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7],...
        'GridColor',[0.35 0.35 0.35],'Box','on');
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    margin=0.3*max_reach;
    xlim(ax,[-margin,max_reach+margin]); ylim(ax,[-max_reach*0.5-margin,max_reach+margin]);
    xlabel(ax,'X (m)','Color',[0.8 0.8 0.8],'FontSize',10);
    ylabel(ax,'Y (m)','Color',[0.8 0.8 0.8],'FontSize',10);
    
    % --- 右侧面板 ---
    panel = uipanel('Parent',fig,'Units','normalized',...
        'Position',[0.72 0.05 0.26 0.92],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.9 0.9 0.9],'Title','控制面板','FontSize',10,'FontWeight','bold');
    y0=0.94;
    uicontrol(panel,'Style','text','String','运行模式:','Units','normalized',...
        'Position',[0.05 y0-0.05 0.35 0.04],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.8 0.8 0.8],'HorizontalAlignment','left');
    hMode = uicontrol(panel,'Style','popupmenu',...
        'String',{'调试模式','接口模式','仿真模式','矫姿模式'},'Value',1,...
        'Units','normalized','Position',[0.40 y0-0.05 0.55 0.04],...
        'Callback',@onModeChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1]);
    y0=y0-0.08;
    hDrawBtn = uicontrol(panel,'Style','togglebutton',...
        'String','自由绘图: 关闭','Value',0,'Units','normalized',...
        'Position',[0.05 y0-0.04 0.44 0.045],'Callback',@onDrawToggle,...
        'Background',[0.3 0.3 0.3],'Foreground',[1 1 1],'FontWeight','bold');
    hSegBtn = uicontrol(panel,'Style','togglebutton',...
        'String','线段绘图: 关闭','Value',0,'Units','normalized',...
        'Position',[0.52 y0-0.04 0.43 0.045],'Callback',@onSegToggle,...
        'Background',[0.3 0.3 0.3],'Foreground',[1 1 1],'FontWeight','bold','FontSize',8);
    y0=y0-0.065;
    uicontrol(panel,'Style','pushbutton','String','清除红色绘画',...
        'Units','normalized','Position',[0.05 y0-0.04 0.90 0.04],...
        'Callback',@onClearDraw,'Background',[0.4 0.2 0.2],'Foreground',[1 1 1]);
    y0=y0-0.055;
    uicontrol(panel,'Style','pushbutton','String','绘画->障碍',...
        'Units','normalized','Position',[0.05 y0-0.04 0.44 0.04],...
        'Callback',@onDrawToObs,'Background',[0.5 0.3 0.1],'Foreground',[1 1 1],...
        'FontWeight','bold','FontSize',8);
    uicontrol(panel,'Style','pushbutton','String','清除障碍',...
        'Units','normalized','Position',[0.52 y0-0.04 0.43 0.04],...
        'Callback',@onClearObs,'Background',[0.4 0.2 0.2],'Foreground',[1 1 1],...
        'FontSize',8);
    hShowPot = uicontrol(panel,'Style','togglebutton','String','势场图: 关闭',...
        'Units','normalized','Position',[0.05 y0-0.09 0.90 0.035],...
        'Callback',@onTogglePotential,'Background',[0.2 0.3 0.5],...
        'Foreground',[1 1 1],'FontSize',7,'Value',0);
    s.showPotential = false;

    y0=y0-0.065;
    y0=y0-0.055;
    uicontrol(panel,'Style','text','String','屏障C:','Units','normalized',...
        'Position',[0.05 y0-0.03 0.30 0.025],'Background',[0.18 0.18 0.18],...
        'Foreground',[1 0.8 0.5],'FontSize',7,'HorizontalAlignment','left');
    hObsSigma = uicontrol(panel,'Style','edit','String',num2str(gp.barrier_C),...
        'Units','normalized','Position',[0.36 y0-0.032 0.18 0.03],...
        'Callback',@(src,~) GlobalParams('barrier_C',str2double(get(src,'String'))),...
        'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(panel,'Style','text','String','动量β:','Units','normalized',...
        'Position',[0.56 y0-0.03 0.15 0.025],'Background',[0.18 0.18 0.18],...
        'Foreground',[1 0.8 0.5],'FontSize',7,'HorizontalAlignment','left');
    hObsGamma = uicontrol(panel,'Style','edit','String',num2str(gp.momentum_beta),...
        'Units','normalized','Position',[0.72 y0-0.032 0.23 0.03],...
        'Callback',@(src,~) GlobalParams('momentum_beta',str2double(get(src,'String'))),...
        'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    y0 = y0 - 0.05;

    uicontrol(panel,'Style','text','String','目标点坐标:','Units','normalized',...
        'Position',[0.05 y0-0.03 0.90 0.025],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.8 1 0.8],'FontSize',8);
    y0=y0-0.04;
    uicontrol(panel,'Style','text','String','X:','Units','normalized',...
        'Position',[0.05 y0-0.032 0.08 0.03],'Background',[0.18 0.18 0.18],'Foreground',[1 1 1],'FontSize',8);
    hTgtX = uicontrol(panel,'Style','edit','String',num2str(s.X_tgt(1)),...
        'Units','normalized','Position',[0.14 y0-0.032 0.28 0.03],...
        'Callback',@onTgtChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',8);
    uicontrol(panel,'Style','text','String','Y:','Units','normalized',...
        'Position',[0.45 y0-0.032 0.08 0.03],'Background',[0.18 0.18 0.18],'Foreground',[1 1 1],'FontSize',8);
    hTgtY = uicontrol(panel,'Style','edit','String',num2str(s.X_tgt(2)),...
        'Units','normalized','Position',[0.54 y0-0.032 0.28 0.03],...
        'Callback',@onTgtChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.06;
    uicontrol(panel,'Style','pushbutton','String','复位关节角 (全零)',...
        'Units','normalized','Position',[0.05 y0-0.035 0.90 0.035],...
        'Callback',@onReset,'Background',[0.25 0.25 0.4],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.06;
    uicontrol(panel,'Style','text','String','关节角度 (rad, -pi~pi):','Units','normalized',...
        'Position',[0.05 y0-0.03 0.90 0.025],'Background',[0.18 0.18 0.18],...
        'Foreground',[1 1 0.5],'FontSize',8);
    y0=y0-0.04;
    hJointEdits = zeros(1,N);
    jrH = min(0.038,0.28/N);
    for j=1:N
        uicontrol(panel,'Style','text','String',sprintf('q%d:',j),'Units','normalized',...
            'Position',[0.05 y0-jrH 0.10 jrH],'Background',[0.18 0.18 0.18],...
            'Foreground',[1 1 1],'FontSize',7);
        hJointEdits(j) = uicontrol(panel,'Style','edit','String','0.000','Units','normalized',...
            'Position',[0.16 y0-jrH 0.60 jrH],'Callback',{@onJointEdit,j},...
            'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
        uicontrol(panel,'Style','slider','Min',-pi,'Max',pi,'Value',0,...
            'Units','normalized','Position',[0.78 y0-jrH 0.19 jrH],...
            'Callback',{@onJointSlider,j},'Background',[0.3 0.3 0.3]);
        y0=y0-jrH-0.005;
    end
    y0=y0-0.008;

    uicontrol(panel,'Style','pushbutton','String','初始','Units','normalized',...
        'Position',[0.05 0.04 0.28 0.03],'Callback',@onQRst,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(panel,'Style','pushbutton','String','保存','Units','normalized',...
        'Position',[0.36 0.04 0.28 0.03],'Callback',@onQSave,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(panel,'Style','pushbutton','String','设置','Units','normalized',...
        'Position',[0.67 0.04 0.28 0.03],'Callback',@onQSet,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    
    % ---- 仿真模式区 ----
    hSimPanel = uipanel('Parent',panel,'Units','normalized',...
        'Position',[0.02 0.08 0.96 0.23],'Visible','off','Background',[0.16 0.16 0.16],...
        'Title','仿真模式','Foreground',[0.9 0.9 0.9],'FontSize',9);
    hSimStat = uicontrol(hSimPanel,'Style','text','String','就绪',...
        'Units','normalized','Position',[0.05 0.80 0.90 0.12],...
        'Background',[0.16 0.16 0.16],'Foreground',[0.5 1 0.5],'FontSize',9);
    uicontrol(hSimPanel,'Style','text','String','目标末端角(rad):','Units','normalized',...
        'Position',[0.05 0.62 0.55 0.12],'Background',[0.16 0.16 0.16],'Foreground',[0.8 0.8 1],'FontSize',9);
    hTgtTheta = uicontrol(hSimPanel,'Style','edit','String','0.0','Units','normalized',...
        'Position',[0.60 0.62 0.35 0.12],'Callback',@onSimThetaChange,...
        'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',9);
    hSimRun = uicontrol(hSimPanel,'Style','pushbutton','String','运行','Units','normalized',...
        'Position',[0.05 0.30 0.28 0.24],'Callback',@onSimRun,...
        'Background',[0.2 0.5 0.2],'Foreground',[1 1 1],'FontWeight','bold');
    hSimStop = uicontrol(hSimPanel,'Style','pushbutton','String','停止','Units','normalized',...
        'Position',[0.36 0.30 0.28 0.24],'Callback',@onSimStop,...
        'Background',[0.5 0.2 0.2],'Foreground',[1 1 1],'FontWeight','bold','Enable','off');
    hSimReplay = uicontrol(hSimPanel,'Style','pushbutton','String','重播','Units','normalized',...
        'Position',[0.67 0.30 0.28 0.24],'Callback',@onReplay,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontWeight','bold','Enable','off');
    hReplayStat = uicontrol(hSimPanel,'Style','text','String','','Units','normalized',...
        'Position',[0.05 0.06 0.90 0.16],'Background',[0.16 0.16 0.16],...
        'Foreground',[1 0.8 0.5],'FontSize',8,'HorizontalAlignment','center');
    uicontrol(hSimPanel,'Style','text','String','帧:','Units','normalized',...
        'Position',[0.05 0.18 0.10 0.08],'Background',[0.16 0.16 0.16],...
        'Foreground',[1 0.8 0.5],'FontSize',7,'HorizontalAlignment','left');
    hReplayFrame = uicontrol(hSimPanel,'Style','edit','String','1','Units','normalized',...
        'Position',[0.16 0.18 0.25 0.08],'Callback',@onReplayJump,...
        'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(hSimPanel,'Style','pushbutton','String','跳转','Units','normalized',...
        'Position',[0.44 0.18 0.14 0.08],'Callback',@onReplayJump,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);

    
    % ---- 矫姿模式区 ----
    hPosePanel = uipanel('Parent',panel,'Units','normalized',...
        'Position',[0.02 0.08 0.96 0.30],'Visible','off','Background',[0.16 0.16 0.16],...
        'Title','矫姿模式 — 末端保持 + 关节追踪','Foreground',[0.9 0.9 0.9],'FontSize',9);
    hPoseStat = uicontrol(hPosePanel,'Style','text','String','就绪',...
        'Units','normalized','Position',[0.05 0.88 0.90 0.10],...
        'Background',[0.16 0.16 0.16],'Foreground',[0.5 1 0.5],'FontSize',8);
    hPoseQEdits = zeros(1,N);
    for j=1:N
        uicontrol(hPosePanel,'Style','text','String',sprintf('qT%d:',j),'Units','normalized',...
            'Position',[0.03+(j-1)*0.24 0.80 0.06 0.07],'Background',[0.16 0.16 0.16],...
            'Foreground',[1 0.8 0.8],'FontSize',7,'HorizontalAlignment','left');
        hPoseQEdits(j) = uicontrol(hPosePanel,'Style','edit','String','0.0','Units','normalized',...
            'Position',[0.09+(j-1)*0.24 0.80 0.15 0.07],...
            'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    end
    uicontrol(hPosePanel,'Style','text','String','位置偏差(m):','Units','normalized',...
        'Position',[0.03 0.70 0.22 0.07],'Background',[0.16 0.16 0.16],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hPoseSigPos = uicontrol(hPosePanel,'Style','edit','String','0.02','Units','normalized',...
        'Position',[0.26 0.70 0.18 0.07],'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(hPosePanel,'Style','text','String','角度偏差(rad):','Units','normalized',...
        'Position',[0.48 0.70 0.22 0.07],'Background',[0.16 0.16 0.16],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hPoseSigAng = uicontrol(hPosePanel,'Style','edit','String','0.1','Units','normalized',...
        'Position',[0.71 0.70 0.18 0.07],'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(hPosePanel,'Style','text','String','追踪权重:','Units','normalized',...
        'Position',[0.03 0.60 0.20 0.07],'Background',[0.16 0.16 0.16],...
        'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
    hPoseLambdaJt = uicontrol(hPosePanel,'Style','edit','String','0.1','Units','normalized',...
        'Position',[0.26 0.60 0.18 0.07],'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
    hPoseUseRRT = uicontrol(hPosePanel,'Style','checkbox','String','启用RRT全局搜索',...
        'Value',0,'Units','normalized','Position',[0.50 0.60 0.45 0.07],...
        'Background',[0.16 0.16 0.16],'Foreground',[1 0.8 0.8],'FontSize',7);
    hPoseRun = uicontrol(hPosePanel,'Style','pushbutton','String','矫姿运行','Units','normalized',...
        'Position',[0.05 0.08 0.42 0.40],'Callback',@onPoseRun,...
        'Background',[0.2 0.5 0.2],'Foreground',[1 1 1],'FontWeight','bold');
    hPoseStop = uicontrol(hPosePanel,'Style','pushbutton','String','停止','Units','normalized',...
        'Position',[0.53 0.08 0.42 0.40],'Callback',@onPoseStop,...
        'Background',[0.5 0.2 0.2],'Foreground',[1 1 1],'FontWeight','bold','Enable','off');
    
    % ---- 接口模式区 ----
    hIfPanel = uipanel('Parent',panel,'Units','normalized',...
        'Position',[0.02 0.08 0.96 0.20],'Visible','off','Background',[0.16 0.16 0.16],...
        'Title','UDP 接口','Foreground',[0.9 0.9 0.9],'FontSize',9);
    uicontrol(hIfPanel,'Style','text','String','UDP端口:','Units','normalized',...
        'Position',[0.05 0.70 0.35 0.15],'Background',[0.16 0.16 0.16],...
        'Foreground',[0.8 0.8 0.8]);
    hPort = uicontrol(hIfPanel,'Style','edit','String',num2str(udpPort),...
        'Units','normalized','Position',[0.42 0.70 0.25 0.18],...
        'Callback',@onPortChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1]);
    hIfStat = uicontrol(hIfPanel,'Style','text','String','状态: 未启动',...
        'Units','normalized','Position',[0.05 0.40 0.90 0.15],...
        'Background',[0.16 0.16 0.16],'Foreground',[1 0.5 0.5]);
    hIfStart = uicontrol(hIfPanel,'Style','pushbutton','String','启动监听',...
        'Units','normalized','Position',[0.05 0.08 0.42 0.22],...
        'Callback',@onIfStart,'Background',[0.2 0.5 0.2],'Foreground',[1 1 1]);
    hIfStop = uicontrol(hIfPanel,'Style','pushbutton','String','停止监听',...
        'Units','normalized','Position',[0.53 0.08 0.42 0.22],...
        'Callback',@onIfStop,'Background',[0.5 0.2 0.2],'Foreground',[1 1 1]);
    hIfInfo = uicontrol(hIfPanel,'Style','text','String','接收: 0 包 / 0 B',...
        'Units','normalized','Position',[0.05 0.00 0.90 0.10],...
        'Background',[0.16 0.16 0.16],'Foreground',[0.7 0.7 0.7],'FontSize',7);
    
    uicontrol(fig,'Style','text','String',...
        sprintf('机械臂: %d段 | 臂长: %.2fm | 快捷键: F=自由 C=清除 R=复位 M=模式 S=线段 P=重播',N,L_seg),...
        'Units','normalized','Position',[0.01 0.005 0.68 0.02],...
        'Background',[0.15 0.15 0.15],'Foreground',[0.5 0.5 0.5],'FontSize',7);
    hFps = uicontrol(fig,'Style','text','String','FPS: --','Units','normalized',...
        'Position',[0.85 0.96 0.12 0.025],'Background',[0.15 0.15 0.15],...
        'Foreground',[0.5 1 0.5],'FontSize',9);
    
    drawAll();
    
    %% 定时器
    fpsBuf=zeros(1,30); fpsIdx=1; tPrev=tic;
    timerObj=timer('Name','Timer','ExecutionMode','fixedRate','Period',1/framerate,...
        'TimerFcn',@onTimerTick,'BusyMode','drop','StartDelay',0.1);
    start(timerObj);
    set(fig,'WindowButtonDownFcn',@onMouseDown,'WindowButtonMotionFcn',@onMouseMove,'WindowButtonUpFcn',@onMouseUp);
    initSocket();
    
    app=struct(); app.figure=fig; app.getState=@() s; app.setJoints=@setJointsExt;
    app.setTarget=@setTargetExt; app.close=@() onClose([],[]); app.reset=@onReset; app.runSim=@onSimRun;
    
    %% ========== 嵌套函数 ==========
    function onQRst(~,~)
        for j=1:s.N
            if mod(j,2)==1, s.q(j)=3.142; else, s.q(j)=-3.142; end
            set(hJointEdits(j),'String',sprintf('%.3f',s.q(j)));
        end; drawAll();
    end
    function onQSave(~,~), s.saved_q=s.q; fprintf('[保存] 当前关节角已记录\n'); end
    function onQSet(~,~)
        s.q=s.saved_q; syncUI(); drawAll();
        fprintf('[设置] 已恢复为保存的关节角\n');
    end

    function initSocket()
        try
            s.udpSock=udp('127.0.0.1','LocalPort',s.udpPort,'Timeout',0.1,'InputBufferSize',8192);
            fopen(s.udpSock);
        catch, s.udpSock=[]; end
    end
    function onTimerTick(~,~)
        try
            if strcmp(s.mode,'interface')&&s.ifActive&&~isempty(s.udpSock), readInterface(); end
            % 重播模式
            if s.replayActive && ~isempty(s.replayFrames)
                s.replayIdx = s.replayIdx + s.replaySpeed;
                if s.replayIdx > length(s.replayFrames)
                    % 重播结束，停在最后一帧
                    s.replayActive = false;
                    s.replayIdx = length(s.replayFrames);
                    s.q = s.replayFrames{end};
                    syncUI();
                    set(hSimReplay,'String','重播','Background',[0.2 0.2 0.4]);
                    set(hReplayStat,'String',sprintf('重播结束 (共 %d 帧)',length(s.replayFrames)));
                    set(hReplayFrame,'String',num2str(length(s.replayFrames)));
                    drawAll();
                    return;
                end
                s.q = s.replayFrames{s.replayIdx};
                syncUI();
                set(hReplayStat,'String',sprintf('重播中: %d/%d',s.replayIdx,length(s.replayFrames)));
                set(hReplayFrame,'String',num2str(s.replayIdx));
                drawAll();
            end
            dt=toc(tPrev); tPrev=tic; fpsBuf(fpsIdx)=1/max(dt,0.001); fpsIdx=mod(fpsIdx,30)+1;
            set(hFps,'String',sprintf('FPS: %.1f',mean(fpsBuf(fpsBuf>0)))); drawAll();
        catch, end
    end
    function readInterface()
        try
            nb=s.udpSock.BytesAvailable;
            if nb>0
                data=fread(s.udpSock,nb,'uint8'); str=native2unicode(data','UTF-8');
                s.uBytes=s.uBytes+nb; s.uCount=s.uCount+1;
                set(hIfInfo,'String',sprintf('接收: %d 包 / %d B',s.uCount,s.uBytes));
                try
                    js=jsondecode(str);
                    if isfield(js,'q')&&length(js.q)==s.N, s.q=js.q(:)'; syncUI(); end
                    if isfield(js,'target'), s.X_tgt=js.target(:)'; syncUI(); end
                catch
                    nums=sscanf(str,'%f,');
                    if length(nums)>=s.N+2, s.q=nums(1:s.N)'; s.X_tgt=nums(s.N+1:s.N+2)'; syncUI(); end
                end
            end
        catch, end
    end
    function syncUI()
        for j=1:s.N, set(hJointEdits(j),'String',sprintf('%.3f',s.q(j))); end
        set(hTgtX,'String',num2str(s.X_tgt(1))); set(hTgtY,'String',num2str(s.X_tgt(2)));
    end
    
    %% 绘图
    function drawAll()
        cla(ax); hold(ax,'on'); margin=0.3*max_reach;
        xlim(ax,[-margin,max_reach+margin]); ylim(ax,[-max_reach*0.5-margin,max_reach+margin]);
        if s.showPotential, drawPotentialField(); end
        drawCoord(); drawObstacles(); drawArm(); drawTarget(); drawFree(); drawSegLines();
        [~,pe]=fk(s.q,s.N,s.L_seg);
        if s.replayActive
            modeStr=sprintf('重播|%d/%d',s.replayIdx,length(s.replayFrames));
        elseif strcmp(s.mode,'sim')
            modeStr=sprintf('仿真|iter:%d err:%.4f',s.simIter,s.simDist);
            if s.simConverged, modeStr=[modeStr ' ok']; end
        elseif strcmp(s.mode,'pose'), modeStr=sprintf('矫姿|iter:%d',s.poseIter);
        else, modeStr=s.mode; end
        title(ax,sprintf('末端:[%.3f,%.3f] 目标:[%.3f,%.3f] %s',pe(1),pe(2),s.X_tgt(1),s.X_tgt(2),modeStr),'Color',[1 1 1]);
        drawnow limitrate;
    end
    function drawCoord()
        xl=xlim(ax); yl=ylim(ax); plot(ax,[xl(1) xl(2)],[0 0],'w-',[0 0],[yl(1) yl(2)],'w-');
    end
    function drawArm()
        pn=fkSimple(s.q,s.N,s.L_seg); [~,pe]=fk(s.q,s.N,s.L_seg);
        % 粗白底边 + 亮色内核，热力图上清晰可见
        for k=1:s.N, plot(ax,[pn(k,1) pn(k+1,1)],[pn(k,2) pn(k+1,2)],'w-','LineWidth',6); end
        for k=1:s.N, plot(ax,[pn(k,1) pn(k+1,1)],[pn(k,2) pn(k+1,2)],'c-','LineWidth',3.5); end
        plot(ax,pn(1,1),pn(1,2),'wo','MarkerSize',10,'MarkerFaceColor',[0 0 0],'LineWidth',2);
        for k=2:s.N+1, plot(ax,pn(k,1),pn(k,2),'mo','MarkerSize',8,'MarkerFaceColor',[1 0 1],'LineWidth',1.5); end
        plot(ax,pe(1),pe(2),'wo','MarkerSize',10,'MarkerFaceColor',[0 1 1],'LineWidth',2);
    end
    function drawTarget()
        xt=s.X_tgt(1); yt=s.X_tgt(2);
        plot(ax,xt,yt,'g*','MarkerSize',14,'MarkerFaceColor',[0 1 0]);
        [~,pe]=fk(s.q,s.N,s.L_seg); plot(ax,[pe(1) xt],[pe(2) yt],':','Color',[0.4 0.8 0.4]);
    end
    function drawFree()
        for k=1:length(s.drawLines)
            pts=s.drawLines{k}; if size(pts,1)>=2, plot(ax,pts(:,1),pts(:,2),'r-','LineWidth',2); end
        end
        if s.drawIsDrawing&&size(s.drawCurLine,1)>=2, plot(ax,s.drawCurLine(:,1),s.drawCurLine(:,2),'r-','LineWidth',2); end
    end
    function drawSegLines()
        for k=1:length(s.segLines)
            ln=s.segLines{k}; plot(ax,ln(:,1),ln(:,2),'m-','LineWidth',2.5);
        end
        if s.segDrawOn && ~isempty(s.segPt)
            plot(ax,s.segPt(1),s.segPt(2),'mo','MarkerSize',10,'MarkerFaceColor','m');
        end
    end
    

    function drawObstacles()
        gp = GlobalParams();
        % 圆形障碍物
        if ~isempty(gp.obs)
            for oi = 1:size(gp.obs,1)
                cx = gp.obs(oi,1); cy = gp.obs(oi,2); cr = gp.obs(oi,3);
                t = linspace(0, 2*pi, 50);
                plot(ax, cx+cr*cos(t), cy+cr*sin(t), 'r-', 'LineWidth', 2);
            end
        end
        % 线段障碍物
        if ~isempty(gp.obs_lines)
            for oi = 1:length(gp.obs_lines)
                ln = gp.obs_lines{oi};
                plot(ax, ln(:,1), ln(:,2), 'm-', 'LineWidth', 3);
            end
        end
    end

    %% 运动学
    function [pa,pe]=fk(qq,NS,L)
        pn=zeros(NS+1,2); pn(1,:)=[0 0]; th=0;
        for k=1:NS, th=th+qq(k); pn(k+1,:)=pn(k,:)+L*[cos(th),sin(th)]; end
        pa=pn; pe=pn(NS+1,:);
    end
    function pn=fkSimple(qq,NS,L)
        pn=zeros(NS+1,2); pn(1,:)=[0 0]; th=0;
        for k=1:NS, th=th+qq(k); pn(k+1,:)=pn(k,:)+L*[cos(th),sin(th)]; end
    end
    
    %% UI 回调
    function onModeChange(src,~)
        switch src.Value
            case 1, s.mode='debug'; s.replayActive=false; set(hIfPanel,'Visible','off'); set(hSimPanel,'Visible','off'); set(hPosePanel,'Visible','off');
                if s.ifActive, onIfStop(); end; if s.simRunning, onSimStop(); end; if s.poseRunning, onPoseStop(); end
            case 2, s.mode='interface'; s.replayActive=false; set(hIfPanel,'Visible','on'); set(hSimPanel,'Visible','off'); set(hPosePanel,'Visible','off');
                if s.simRunning, onSimStop(); end; if s.poseRunning, onPoseStop(); end
            case 3, s.mode='sim'; s.replayActive=false; set(hIfPanel,'Visible','off'); set(hSimPanel,'Visible','on'); set(hPosePanel,'Visible','off');
                if s.ifActive, onIfStop(); end; if s.poseRunning, onPoseStop(); end
                set(hSimStat,'String','就绪','Foreground',[0.5 1 0.5]);
            case 4, s.mode='pose'; s.replayActive=false; set(hIfPanel,'Visible','off'); set(hSimPanel,'Visible','off'); set(hPosePanel,'Visible','on');
                if s.ifActive, onIfStop(); end; if s.simRunning, onSimStop(); end
                set(hPoseStat,'String','就绪 — 输入目标关节角后点击矫姿运行','Foreground',[0.5 1 0.5]);
        end; drawAll();
    end
    function onDrawToggle(src,~)
        s.drawOn=src.Value;
        if s.drawOn, s.segDrawOn=false; set(hSegBtn,'Value',0,'String','线段绘图: 关闭','Background',[0.3 0.3 0.3]);
            set(src,'String','自由绘图: 开启','Background',[0.6 0.2 0.2]);
        else, set(src,'String','自由绘图: 关闭','Background',[0.3 0.3 0.3]); end
    end
    function onSegToggle(src,~)
        s.segDrawOn=src.Value; s.segPt=[];
        if s.segDrawOn, s.drawOn=false; set(hDrawBtn,'Value',0,'String','自由绘图: 关闭','Background',[0.3 0.3 0.3]);
            set(src,'String','线段绘图: 开启','Background',[0.5 0.2 0.5]);
        else, set(src,'String','线段绘图: 关闭','Background',[0.3 0.3 0.3]); end
    end
    function onClearDraw(~,~), s.drawLines={}; s.drawCurLine=[]; s.segLines={}; s.segPt=[]; drawAll(); end
    
    function onClearObs(~,~)
        GlobalParams('obs_lines', {});
        fprintf('[清除障碍] 已清除所有线段障碍物\n');
        drawAll();
    end
    
    function onTogglePotential(src,~)
        s.showPotential = src.Value;
        if s.showPotential
            set(src,'String','势场图: 开启','Background',[0.5 0.3 0.1]);
        else
            set(src,'String','势场图: 关闭','Background',[0.2 0.3 0.5]);
        end
        drawAll();
    end
    
    function drawPotentialField()
        % 绘制静态势场热力图：末端距离 + 障碍高斯软约束
        gp = GlobalParams();
        margin = 0.3 * max_reach;
        x_vals = linspace(-margin, max_reach+margin, 120);
        y_vals = linspace(-max_reach*0.5-margin, max_reach+margin, 120);
        [X, Y] = meshgrid(x_vals, y_vals);
        
        % 1. 目标距离势能：梯度等价于 QP 中的 f_pos = 2·γ_soft·s·J'·u
        gamma_soft = gp.gamma_soft;
        s_eff = gp.dt_base * gp.barrier_C;  % 匹配 QP 有效步长
        w_pos = 2 * gamma_soft * s_eff;
        V_dist = w_pos * sqrt((X - s.X_tgt(1)).^2 + (Y - s.X_tgt(2)).^2);
        
        % 2. 障碍对数屏障势能 V(g) = -ln(g-ρ₀+ε) ---
        V_obs = zeros(size(X));
        rho0_v = gp.rho0;
        barrier_eps_v = gp.barrier_eps;
        barrier_C_v = gp.barrier_C;
        
        % 圆形障碍物
        if ~isempty(gp.obs)
            for oi = 1:size(gp.obs,1)
                cx = gp.obs(oi,1); cy = gp.obs(oi,2); cr = gp.obs(oi,3);
                Dc = sqrt((X - cx).^2 + (Y - cy).^2) - cr;
                Dc = max(Dc - rho0_v, barrier_eps_v);  % 对数屏障距离
                V_obs = V_obs - gamma_soft * log(Dc);               % V = -ln(g-ρ₀+ε)
            end
        end
        
        % 线段障碍物
        if ~isempty(gp.obs_lines)
            for oi = 1:length(gp.obs_lines)
                ln = gp.obs_lines{oi};
                p1 = ln(1,:); p2 = ln(2,:);
                dx = p2(1) - p1(1); dy = p2(2) - p1(2);
                len2 = dx^2 + dy^2;
                if len2 < 1e-12
                    Ds = sqrt((X - p1(1)).^2 + (Y - p1(2)).^2);
                else
                    tx = ((X - p1(1))*dx + (Y - p1(2))*dy) / len2;
                    tx = max(0, min(1, tx));
                    px = p1(1) + tx*dx;
                    py = p1(2) + tx*dy;
                    Ds = sqrt((X - px).^2 + (Y - py).^2);
                end
                Ds = max(Ds - rho0_v, barrier_eps_v);
                V_obs = V_obs - gamma_soft * log(Ds);
            end
        end
        
        % 3. 综合势场 (V_dist 已含 w_pos)
        V = V_obs + V_dist;
        
        % 4. 绘制 — 分层着色 + 等势线
        % 4a. 裁剪极值（99百分位），避免离群值压缩色域
        V_max = prctile(V(:), 99);
        V(V > V_max) = V_max;
        V_clip = max(V, 0.001);
        
        h_pc = pcolor(ax, X, Y, V_clip);
        shading(ax, 'interp');
        uistack(h_pc, 'bottom');  % 推到底层，不遮挡arm/obstacle
        colormap(ax, turbo);
        caxis(ax, [0, V_max]);
        
        % 4c. 叠加等势线（白虚线），数量自适应
        n_contours = min(25, round(V_max / max(1, prctile(V(:), 10)) * 8));
        hold(ax, 'on');
        contour(ax, X, Y, V_clip, n_contours, ...
            'LineColor', [0.6 0.6 0.6], 'LineWidth', 0.5, 'LineStyle', ':');
        
        % 4d. 标记硬边界等高线（g=rho0 对应 V 值）
        % 计算 rho0 处的大致势能值作为参考线
        V_rho0 = -gamma_soft * log(barrier_eps_v) + w_pos * 0.02;  % g=ρ₀ 处 ln 势能
        contour(ax, X, Y, V_clip, [V_rho0 V_rho0], ...
            'LineColor', [1 0.3 0.3], 'LineWidth', 1.5, 'LineStyle', '-');
        
    end

    
    function onDrawToObs(~,~)
        lines = {};
        for k=1:length(s.drawLines)
            pts=s.drawLines{k};
            if size(pts,1)<2, continue; end
            simplified=rdp(pts,0.005);
            for i=1:size(simplified,1)-1
                lines{end+1}=simplified(i:i+1,:); %#ok<AGROW>
            end
        end
        for k=1:length(s.segLines)
            lines{end+1}=s.segLines{k}; %#ok<AGROW>
        end
        if isempty(lines)
            warndlg('没有绘画可转换。请先绘制红色线条或线段。','提示'); return;
        end
        GlobalParams('obs_lines', lines);
        fprintf('[绘画->障碍] 已转换 %d 条绘画为 %d 条线段障碍物\n',...
            length(s.drawLines)+length(s.segLines), length(lines));
        drawAll();
    end
    
    function out=rdp(points,eps)
        if size(points,1)<=2, out=points; return; end
        p_start=points(1,:); p_end=points(end,:);
        dx_line=p_end(1)-p_start(1); dy_line=p_end(2)-p_start(2);
        len2=dx_line^2+dy_line^2;
        if len2<1e-12, out=[p_start;p_end]; return; end
        max_dist=0; max_idx=1;
        for i=2:size(points,1)-1
            t=((points(i,1)-p_start(1))*dx_line+(points(i,2)-p_start(2))*dy_line)/len2;
            t=max(0,min(1,t));
            px=p_start(1)+t*dx_line; py=p_start(2)+t*dy_line;
            d=sqrt((points(i,1)-px)^2+(points(i,2)-py)^2);
            if d>max_dist, max_dist=d; max_idx=i; end
        end
        if max_dist<=eps, out=[p_start;p_end];
        else, left=rdp(points(1:max_idx,:),eps); right=rdp(points(max_idx:end,:),eps);
            out=[left(1:end-1,:);right]; end
    end
    
    function onParamModeChange(src,~)
        if src.Value==1
            s.paramMode='custom'; set(hPresetPanel,'Visible','off');
            for kk=1:length(hC), set(hC{kk},'Visible','on'); end
            if isfield(s.savedParams,'lambda_activate')
                GlobalParams('lambda_activate',s.savedParams.lambda_activate);
                set(hParamLambdaAct,'String',num2str(s.savedParams.lambda_activate));
                GlobalParams('lambda_motor',s.savedParams.lambda_motor);
                set(hParamLambdaMotor,'String',num2str(s.savedParams.lambda_motor));
                GlobalParams('sig_min2',s.savedParams.sig_min2);
                set(hParamSigMin,'String',num2str(s.savedParams.sig_min2));
                GlobalParams('sigma_weight',s.savedParams.sigma_weight);
                set(hParamSigmaW,'String',num2str(s.savedParams.sigma_weight));
                GlobalParams('w_part',s.savedParams.w_part);
                GlobalParams('lambda_part',s.savedParams.lambda_part);
                set(hParamLambdaPart,'String',num2str(s.savedParams.lambda_part));
            end
        else
            s.paramMode='preset'; set(hPresetPanel,'Visible','on');
            for kk=1:length(hC), set(hC{kk},'Visible','off'); end
            gp=GlobalParams();
            s.savedParams.lambda_activate=gp.lambda_activate;
            s.savedParams.lambda_motor=gp.lambda_motor;
            s.savedParams.sig_min2=gp.sig_min2;
            s.savedParams.sigma_weight=gp.sigma_weight;
            s.savedParams.w_part=gp.w_part;
            s.savedParams.lambda_part=gp.lambda_part;
        end
        s.faultMotors=false(1,s.N); s.freeMotorCount=false; s.freeAngleStep=false; s.zeroBaseError=false;
        for j=1:s.N, set(hFaultChecks(j),'Value',0); end
        set(hPresetFreeCount,'Value',0); set(hPresetFreeAngle,'Value',0); set(hPresetZeroErr,'Value',0);
        applyPresets();
    end
    function onPresetChange(~,~)
        for j=1:min(s.N, length(hFaultChecks)), s.faultMotors(j)=get(hFaultChecks(j),'Value'); end
        s.freeMotorCount=get(hPresetFreeCount,'Value');
        s.freeAngleStep=get(hPresetFreeAngle,'Value');
        s.zeroBaseError=get(hPresetZeroErr,'Value');
        applyPresets();
    end
    function applyPresets()
        if any(s.faultMotors), w=ones(1,s.N); w(s.faultMotors)=1e9;
        else, w=ones(1,s.N); end
        GlobalParams('w_part',w);
        if strcmp(s.paramMode,'preset')
            if s.freeMotorCount
                set(hParamLambdaAct,'String','0'); GlobalParams('lambda_activate',0);
            else
                set(hParamLambdaAct,'String',num2str(s.savedParams.lambda_activate));
                GlobalParams('lambda_activate',s.savedParams.lambda_activate);
            end
            if s.freeAngleStep
                set(hParamLambdaMotor,'String','0'); GlobalParams('lambda_motor',0);
            else
                set(hParamLambdaMotor,'String',num2str(s.savedParams.lambda_motor));
                GlobalParams('lambda_motor',s.savedParams.lambda_motor);
            end
            if s.zeroBaseError
                set(hParamSigMin,'String','0'); GlobalParams('sig_min2',0);
                GlobalParams('sigma_weight',0);
            else
                set(hParamSigMin,'String',num2str(s.savedParams.sig_min2));
                GlobalParams('sig_min2',s.savedParams.sig_min2);
                GlobalParams('sigma_weight',s.savedParams.sigma_weight);
            end
        end
    end
    
    function onReconfig(~,~)
        try
            newN=str2double(get(hParamN,'String')); newL=str2double(get(hParamL,'String'));
        catch, newN=s.N; newL=s.L_seg; end
        if isnan(newN)||newN<1||newN>20, newN=s.N; end
        if isnan(newL)||newL<=0, newL=s.L_seg; end
        v=@(h,def) safeVal(h,def);
        newQmin=v(hParamQmin,gp.q_min); newQmax=v(hParamQmax,gp.q_max);
        newSig=v(hParamSigMin,gp.sig_min2); newKap=v(hParamKappa,gp.kappa);
        newDamp=v(hParamDamp,gp.lambda_damp); newGam=v(hParamGammaS,gp.gamma_soft);
        newLamM=v(hParamLambdaM,gp.lambda_m); newSig0=v(hParamSig0,1);
        newTau=v(hParamTau,1); newAngB=v(hParamAngBase,gp.gamma_ang_base);
        newAngP=v(hParamAngPeak,gp.gamma_ang_peak); newSW=v(hParamSigmaW,gp.sigma_weight);
        newLP=v(hParamLambdaPart,gp.lambda_part); newLA=v(hParamLambdaAct,gp.lambda_activate);
        newLM=v(hParamLambdaMotor,gp.lambda_motor);
        newBarC=v(hParamObsSigma,gp.barrier_C); newBeta=v(hParamObsGamma,gp.momentum_beta);

        nrrt=get(hParamRRT,'Value')-1;
        nsamp=v(hParamRrtSamp,gp.rrt_max_samples); nstep=v(hParamRrtStep,gp.rrt_max_step);
        neps=v(hParamRrtEps,gp.rrt_goal_eps); nstar=v(hParamRrtStarIter,gp.rrt_star_max_iter);
        nrad=v(hParamRrtStarRad,gp.rrt_star_radius); ntrees=v(hParamRrtTrees,gp.rrt_num_trees);
        GlobalParams('N',newN,'L_seg',newL,'q_min',newQmin,'q_max',newQmax,...
            'sig_min2',newSig,'kappa',newKap,'lambda_damp',newDamp,'gamma_soft',newGam,...
            'barrier_C',newBarC,'momentum_beta',newBeta,...
            'lambda_m',newLamM,'gamma_ang_base',newAngB,'gamma_ang_peak',newAngP,...
            'lambda_m',newLamM,'gamma_ang_base',newAngB,'gamma_ang_peak',newAngP,...
            'sigma_weight',newSW,'lambda_part',newLP,'lambda_activate',newLA,'lambda_motor',newLM,...
            'use_global_search',nrrt,'rrt_num_trees',ntrees,'rrt_max_samples',nsamp,...
            'rrt_max_step',nstep,'rrt_goal_eps',neps,'rrt_star_max_iter',nstar,'rrt_star_radius',nrad);
        s.N=newN; s.L_seg=newL; s.q=zeros(1,newN);
        s.sig0_arr=[newSig0,0.06*ones(1,max(0,newN-1))];
        s.tau_arr=[newTau,0.7*ones(1,max(0,newN-1))]; s.m_arr=zeros(1,newN);
        max_reach=newN*newL; s.X_tgt=[max_reach*0.6,max_reach*0.4];
        if newN~=length(hJointEdits)
            warndlg(sprintf('参数已更新。请关闭窗口后重新运行。'),'重建提示');
        end
        drawAll();
    end
    function val=safeVal(h,def)
        val=str2double(get(h,'String')); if isnan(val)||val<0, val=def; end
    end
    
    function onJointEdit(src,~,idx)
        val=str2double(src.String);
        if ~isnan(val), val=max(-pi,min(pi,val)); s.q(idx)=val;
            set(hJointEdits(idx),'String',sprintf('%.3f',val)); drawAll(); end
    end
    function onJointSlider(src,~,idx), s.q(idx)=src.Value;
        set(hJointEdits(idx),'String',sprintf('%.3f',s.q(idx))); drawAll(); end
    function onTgtChange(~,~)
        xv=str2double(get(hTgtX,'String')); yv=str2double(get(hTgtY,'String'));
        if ~isnan(xv)&&~isnan(yv), s.X_tgt=[xv,yv]; drawAll(); end
    end
    function onReset(~,~), s.q=zeros(1,s.N); for j=1:s.N, set(hJointEdits(j),'String','0.000'); end; drawAll(); end
    function onIfStart(~,~)
        if isempty(s.udpSock)||~strcmp(s.udpSock.Status,'open')
            try, s.udpSock=udp('127.0.0.1','LocalPort',s.udpPort,'Timeout',0.1,'InputBufferSize',8192); fopen(s.udpSock);
            catch ME, set(hIfStat,'String',['错误: ' ME.message]); return; end
        end; flushinput(s.udpSock); s.ifActive=true; s.uCount=0; s.uBytes=0;
        set(hIfStat,'String',sprintf('监听中(%d)',s.udpPort),'Foreground',[0.5 1 0.5]);
        set(hIfStart,'Enable','off'); set(hIfStop,'Enable','on');
    end
    function onIfStop(~,~), s.ifActive=false; set(hIfStat,'String','已停止','Foreground',[1 0.5 0.5]);
        set(hIfStart,'Enable','on'); set(hIfStop,'Enable','off'); end
    function onPortChange(~,~)
        np=str2double(get(hPort,'String'));
        if ~isnan(np)&&np>0&&np<65536
            was=s.ifActive; if was, onIfStop(); end
            if ~isempty(s.udpSock)&&strcmp(s.udpSock.Status,'open'), fclose(s.udpSock); delete(s.udpSock); end
            s.udpPort=np; s.udpSock=[]; initSocket(); if was, onIfStart(); end
            set(hIfStat,'String',sprintf('端口 %d',np));
        end
    end
    
    %% 仿真
    function onSimRun(~,~)
        if s.simRunning, return; end; s.simRunning=true; s.simStopReq=false; s.simIter=0; s.simConverged=false;
        s.replayFrames = {}; s.replayActive = false;
        set(hSimRun,'Enable','off'); set(hSimStop,'Enable','on'); set(hSimReplay,'Enable','off');
        set(hSimStat,'String','运行中...','Foreground',[1 1 0.5]); drawnow;
        try, runSim(); catch ME, set(hSimStat,'String',['错误: ' ME.message],'Foreground',[1 0.3 0.3]);
            s.simRunning=false; set(hSimRun,'Enable','on'); set(hSimStop,'Enable','off'); end
    end
    function onSimStop(~,~)
        if ~s.simRunning, return; end; s.simStopReq=true; s.simRunning=false;
        set(hSimRun,'Enable','on'); set(hSimStop,'Enable','off');
        set(hSimStat,'String',sprintf('已停止 (iter %d)',s.simIter),'Foreground',[1 0.5 0.5]);
    end
    function onReplay(~,~)
        if isempty(s.replayFrames), return; end
        if s.replayActive
            % 停止重播
            s.replayActive = false;
            s.q = s.replayFrames{end};
            syncUI();
            set(hSimReplay,'String','重播','Background',[0.2 0.2 0.4]);
            set(hReplayStat,'String',sprintf('重播停止 (%d 帧)',length(s.replayFrames)));
            drawAll();
            return;
        end
        % 开始重播
        s.replayActive = true;
        s.replayIdx = 1;
        s.q = s.replayFrames{1};
        set(hSimReplay,'String','停止重播','Background',[0.6 0.2 0.2]);
        syncUI();
        set(hReplayStat,'String',sprintf('重播中: 1/%d',length(s.replayFrames)));
        set(hReplayFrame,'String','1');
        drawAll();
    end
    function onReplayJump(~,~)
        if isempty(s.replayFrames), return; end
        fn = str2double(get(hReplayFrame,'String'));
        if isnan(fn) || fn < 1, fn = 1; end
        if fn > length(s.replayFrames), fn = length(s.replayFrames); end
        fn = round(fn);
        s.replayIdx = fn;
        s.q = s.replayFrames{fn};
        set(hReplayFrame,'String',num2str(fn));
        set(hReplayStat,'String',sprintf('跳转至 %d/%d', fn, length(s.replayFrames)));
        syncUI(); drawAll();
    end

    function onSimThetaChange(~,~)
        val=str2double(get(hTgtTheta,'String'));
        if ~isnan(val), s.theta_tgt=val; set(hTgtTheta,'String',sprintf('%.3f',val)); end
    end
    function runSim()
        gp=GlobalParams(); params=struct();
        params.N=s.N; params.L_seg=s.L_seg; params.theta_end_target=s.theta_tgt;
        off=0.06; params.rod_offset_arr=zeros(1,s.N);
        for k=1:s.N, params.rod_offset_arr(k)=off*(-1)^(k+1); end
        params.X_target=s.X_tgt; params.plot_pad=gp.plot_pad; params.bottom_pad=gp.bottom_pad;
        params.q_min=gp.q_min*ones(1,s.N); params.q_max=gp.q_max*ones(1,s.N);
        params.dq_step_max=gp.dq_step_max; params.dq_step_min=gp.dq_step_min; params.kappa=gp.kappa;
        params.m_arr=maybeFill(gp.m_arr,s.N,0); params.sig0_arr=maybeFill(gp.sig0_arr,s.N,[1,0.06]);
        params.tau_arr=maybeFill(gp.tau_arr,s.N,[1,0.7]); params.mu_e_arr=maybeFill(gp.mu_e_arr,s.N,0);
        params.sig_min2=gp.sig_min2; params.lambda_damp=gp.lambda_damp; params.gamma_soft=gp.gamma_soft;
        params.gamma_ang_base=gp.gamma_ang_base; params.gamma_ang_peak=gp.gamma_ang_peak;
        params.sigma_weight=gp.sigma_weight;
        params.lambda_part=gp.lambda_part; params.w_part=gp.w_part;
        params.lambda_activate=gp.lambda_activate; params.lambda_motor=gp.lambda_motor;
        params.dq_stall_thresh=gp.dq_stall_thresh; params.stall_count_max=gp.stall_count_max;
        params.obs=gp.obs; params.rho0=gp.rho0; params.safe_margin=gp.safe_margin;
        params.obs_lines=gp.obs_lines;
        % obs_sigma/gamma_obs 已移除，由 barrier_C/momentum_beta 替代
        params.use_momentum=gp.use_momentum; params.momentum_beta=gp.momentum_beta;
        params.barrier_C=gp.barrier_C; params.barrier_eps=gp.barrier_eps;
        params.dt_base=gp.dt_base; params.rho_critical=gp.rho_critical;
        params.lambdaM=gp.lambdaM; params.lambda_m=gp.lambda_m; params.max_iter=gp.max_iter;
        params.q_init=s.q; params.use_global_search=gp.use_global_search;
        params.rrt_max_samples=gp.rrt_max_samples; params.rrt_max_step=gp.rrt_max_step;
        params.rrt_goal_bias=gp.rrt_goal_bias; params.rrt_goal_eps=gp.rrt_goal_eps;
        params.rrt_num_trees=gp.rrt_num_trees; params.rrt_star_max_iter=gp.rrt_star_max_iter;
        params.rrt_star_radius=gp.rrt_star_radius;
        resultQ=runLArmIK_2D(params,5);
        if ~isempty(resultQ), s.q=resultQ; syncUI(); end
        [~,pe]=planarFK_api(s.q,s.N,s.L_seg); err=norm(s.X_tgt-pe);
        set(hSimStat,'String',sprintf('IK完成|末端[%.3f,%.3f]|err:%.4f',pe(1),pe(2),err),'Foreground',[0.5 1 0.5]);
        s.simIter=params.max_iter; s.simDist=err; s.simConverged=(err<params.lambda_m);
        s.simRunning=false; set(hSimRun,'Enable','on'); set(hSimStop,'Enable','off');
        if isfield(params,'prm_path') && ~isempty(params.prm_path)
            nWpts = size(params.prm_path, 1);
            s.replayFrames = cell(1, nWpts);
            for fi = 1:nWpts
                s.replayFrames{fi} = params.prm_path(fi, :);
            end
            % 追加最终收敛状态
            s.replayFrames{end+1} = s.q;
        else
            q_start = params.q_init;
            q_end = s.q;
            nFrames = max(20, s.simIter);  % 使用实际迭代数，不再限制 200
            s.replayFrames = cell(1, nFrames);
            for fi = 1:nFrames
                t = (fi-1)/(nFrames-1);
                s.replayFrames{fi} = (1-t)*q_start + t*q_end;
            end
        end
        set(hSimReplay,'Enable','on');
        set(hReplayStat,'String',sprintf('录制完成 %d 帧', length(s.replayFrames)));
        drawAll();

    end
    function arr=maybeFill(arr,N,def)
        if isempty(arr)||length(arr)~=N
            if length(def)==1, arr=def*ones(1,N); else, arr=[def(1),def(2)*ones(1,N-1)]; end
        else, arr=arr(:)'; end
    end
    function [pa,pe]=planarFK_api(qq,NS,L)
        pn=zeros(NS+1,2); pn(1,:)=[0 0]; th=0;
        for k=1:NS, th=th+qq(k); pn(k+1,:)=pn(k,:)+L*[cos(th),sin(th)]; end
        pa=pn; pe=pn(NS+1,:);
    end
    
    %% 矫姿模式
    function onPoseRun(~,~)
        if s.poseRunning, return; end
        qT=zeros(1,s.N);
        for j=1:s.N, qT(j)=str2double(get(hPoseQEdits(j),'String')); end
        s.pose_tgt_q=qT;
        s.pose_sigma_pos=str2double(get(hPoseSigPos,'String'));
        s.pose_sigma_ang=str2double(get(hPoseSigAng,'String'));
        s.pose_lambda_joint=str2double(get(hPoseLambdaJt,'String'));
        s.pose_use_rrt=get(hPoseUseRRT,'Value');
        s.poseRunning=true; s.poseStopReq=false; s.poseIter=0; s.poseConverged=false;
        set(hPoseRun,'Enable','off'); set(hPoseStop,'Enable','on');
        set(hPoseStat,'String','矫姿运行中...','Foreground',[1 1 0.5]); drawnow;
        try, runPose(); catch ME, set(hPoseStat,'String',['错误: ' ME.message],'Foreground',[1 0.3 0.3]);
            s.poseRunning=false; set(hPoseRun,'Enable','on'); set(hPoseStop,'Enable','off'); end
    end
    function onPoseStop(~,~)
        if ~s.poseRunning, return; end; s.poseStopReq=true; s.poseRunning=false;
        set(hPoseRun,'Enable','on'); set(hPoseStop,'Enable','off');
        set(hPoseStat,'String',sprintf('已停止 (iter %d)',s.poseIter),'Foreground',[1 0.5 0.5]);
    end
    function runPose()
        gp=GlobalParams();
        off=0.06; rod=zeros(1,s.N);
        for k=1:s.N, rod(k)=off*(-1)^(k+1); end
        params=struct();
        params.N=s.N; params.L_seg=s.L_seg; params.q_min=gp.q_min*ones(1,s.N); params.q_max=gp.q_max*ones(1,s.N);
        params.kappa=gp.kappa; params.obs=gp.obs; params.rho0=gp.rho0; params.safe_margin=gp.safe_margin;
        params.lambda_joint_pose=s.pose_lambda_joint;
        params.sigma_pos_pose=s.pose_sigma_pos; params.sigma_ang_pose=s.pose_sigma_ang;
        params.pose_move_max_iter=gp.pose_move_max_iter; params.pose_move_eps=gp.pose_move_eps;
        params.pose_dq_step_max=gp.pose_dq_step_max; params.use_rrt_pose_move=s.pose_use_rrt;
        params.rrt_max_samples=gp.rrt_max_samples; params.rrt_max_step=gp.rrt_max_step;
        params.rrt_goal_bias=gp.rrt_goal_bias; params.rrt_goal_eps=gp.rrt_goal_eps;
        params.obs_lines=gp.obs_lines;
        params.lambda_part=gp.lambda_part; params.w_part=gp.w_part;
        params.lambda_motor=gp.lambda_motor;
        [qRes,iters,conv]=moveWithPoseConstraint(s.q,s.pose_tgt_q,params,rod);
        if ~isempty(qRes), s.q=qRes; syncUI(); end
        s.poseIter=iters; s.poseConverged=conv;
        if conv, set(hPoseStat,'String',sprintf('收敛 (iter %d)',iters),'Foreground',[0.5 1 0.5]);
        else, set(hPoseStat,'String',sprintf('完成 (iter %d, 未收敛)',iters),'Foreground',[1 0.8 0.5]); end
        s.poseRunning=false; set(hPoseRun,'Enable','on'); set(hPoseStop,'Enable','off'); drawAll();
    end
    
    %% 鼠标
    function onMouseDown(~,~)
        cp=get(ax,'CurrentPoint'); cx=cp(1,1); cy=cp(1,2);
        if s.segDrawOn
            if isempty(s.segPt)
                s.segPt=[cx,cy]; drawAll(); return;
            else
                p0=s.segPt; p1=[cx,cy];
                if norm(p1-p0)>0.01
                    s.segLines{end+1}=[p0;p1];
                end
                s.segPt=[]; drawAll(); return;
            end
        end
        if s.drawOn, s.drawIsDrawing=true; s.drawCurLine=[cx,cy]; return; end
        if ~strcmp(s.mode,'debug'), return; end
        pn=fkSimple(s.q,s.N,s.L_seg); [~,pe]=fk(s.q,s.N,s.L_seg); pt=[cx,cy]; hr=s.L_seg*0.3;
        if norm(pt-s.X_tgt)<hr, s.dragMode='target'; s.X_tgt=pt; drawAll(); return; end
        if norm(pt-pe)<hr, s.dragMode='effector'; dragJoint(pt,s.N+1); return; end
        for k=s.N+1:-1:2, if norm(pt-pn(k,:))<hr, s.dragMode='joint'; s.dragIdx=k; dragJoint(pt,k); return; end, end
    end
    function dragJoint(pt,idx)
        if idx<2, return; end; jc=idx-1; pn=fkSimple(s.q,s.N,s.L_seg); prev=pn(idx-1,:); vec=pt-prev;
        if norm(vec)<1e-8, return; end; ang=atan2(vec(2),vec(1)); prevSum=sum(s.q(1:jc-1));
        nr=ang-prevSum; nr=atan2(sin(nr),cos(nr)); s.q(jc)=nr; drawAll();
    end
    function onMouseMove(~,~)
        if s.drawOn&&s.drawIsDrawing, cp=get(ax,'CurrentPoint'); s.drawCurLine(end+1,:)=[cp(1,1),cp(1,2)]; return; end
        if strcmp(s.dragMode,'none'), return; end; cp=get(ax,'CurrentPoint'); pt=[cp(1,1),cp(1,2)];
        switch s.dragMode
            case 'target', s.X_tgt=pt;
            case 'effector', dragJoint(pt,s.N+1);
            case 'joint', dragJoint(pt,s.dragIdx);
        end; drawAll();
    end
    function onMouseUp(~,~)
        if s.drawOn&&s.drawIsDrawing, s.drawIsDrawing=false;
            if size(s.drawCurLine,1)>=2, s.drawLines{end+1}=s.drawCurLine; end; s.drawCurLine=[]; return; end
        s.dragMode='none'; s.dragIdx=0;
    end
    
    %% 键盘
    function onKey(~,evt)
        switch evt.Key
            case 'f', nv=~s.drawOn; s.drawOn=nv; set(hDrawBtn,'Value',nv);
                if nv, s.segDrawOn=false; set(hSegBtn,'Value',0,'String','线段绘图: 关闭','Background',[0.3 0.3 0.3]); end
            case 's', nv=~s.segDrawOn; s.segDrawOn=nv; set(hSegBtn,'Value',nv);
                if nv, s.drawOn=false; set(hDrawBtn,'Value',0,'String','自由绘图: 关闭','Background',[0.3 0.3 0.3]); end
                s.segPt=[]; drawAll();
            case 'p', onReplay();
            case 'c', s.drawLines={}; s.drawCurLine=[]; s.segLines={}; s.segPt=[]; drawAll();
            case 'r', s.q=zeros(1,s.N); case 'm'
                if strcmp(s.mode,'debug'), set(hMode,'Value',2);
                elseif strcmp(s.mode,'interface'), set(hMode,'Value',3);
                else, set(hMode,'Value',1); end; onModeChange(hMode);
            case 'escape'
                s.drawIsDrawing=false; s.drawCurLine=[]; s.dragMode='none'; s.dragIdx=0;
                s.segDrawOn=false; s.segPt=[]; set(hSegBtn,'Value',0,'String','线段绘图: 关闭','Background',[0.3 0.3 0.3]); drawAll();
        end
    end
    function onClose(~,~)
        if s.simRunning, onSimStop(); end; try stop(timerObj); delete(timerObj); catch, end
        try if ~isempty(s.udpSock)&&strcmp(s.udpSock.Status,'open'), fclose(s.udpSock); delete(s.udpSock); end, catch, end
        try delete(fig); catch, end
    end
    function setJointsExt(qv), if length(qv)==s.N, s.q=qv(:)'; syncUI(); drawAll(); end, end
    function setTargetExt(xy), if length(xy)>=2, s.X_tgt=xy(1:2); syncUI(); drawAll(); end, end
    
    function [h1,h2,np]=addTwoCol(par,py,rH,lb1,d1,lb2,d2,hList)
        uicontrol(par,'Style','text','String',lb1,'Units','normalized',...
            'Position',[0.03 py-rH 0.16 rH],'Background',[0.14 0.14 0.18],...
            'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
        h1=uicontrol(par,'Style','edit','String',d1,'Units','normalized',...
            'Position',[0.20 py-rH 0.27 rH],'Callback',@onReconfig,...
            'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
        uicontrol(par,'Style','text','String',lb2,'Units','normalized',...
            'Position',[0.50 py-rH 0.15 rH],'Background',[0.14 0.14 0.18],...
            'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
        h2=uicontrol(par,'Style','edit','String',d2,'Units','normalized',...
            'Position',[0.66 py-rH 0.27 rH],'Callback',@onReconfig,...
            'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
        hList{end+1}=h1; hList{end+1}=h2;
        np=py-rH-0.003;
    end
end