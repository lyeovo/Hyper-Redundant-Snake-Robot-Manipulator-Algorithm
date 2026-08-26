%% PlanarDrawApp_Simple.m -- Adapted from PlanarDrawApp.m for runLArmIK_2D_Simple
function app = PlanarDrawApp_Simple(varargin)
    if nargin > 0, GlobalParams(varargin{:}); end
    gp = GlobalParams();
    N = gp.N; L_seg = gp.L_seg;
    udpPort = gp.udp_port; framerate = gp.framerate;
    max_reach = N * L_seg;
    
    s = struct();
    s.N=N; s.L_seg=L_seg; s.q=zeros(1,N);
    s.X_tgt=[max_reach*0.6, max_reach*0.4];
    s.mode='debug'; s.drawOn=false; s.segDrawOn=false; s.segPt=[]; s.segLines={};
    s.drawLines={}; s.drawCurLine=[]; s.drawIsDrawing=false;
    s.ifActive=false; s.udpPort=udpPort; s.udpSock=[]; s.uCount=0; s.uBytes=0;
    s.dragMode='none'; s.dragIdx=0;
    s.simRunning=false; s.simStopReq=false; s.simIter=0; s.simDist=Inf; s.simConverged=false;
    s.theta_tgt=0;
    s.replayFrames={}; s.replayActive=false; s.replayIdx=0; s.replaySpeed=2;
    s.saved_q=zeros(1,N); s.showPotential=false;
    if isempty(gp.m_arr), s.m_arr=zeros(1,N); else, s.m_arr=gp.m_arr(:)'; end
    if isempty(gp.sig0_arr), s.sig0_arr=[1,0.06*ones(1,N-1)]; else, s.sig0_arr=gp.sig0_arr(:)'; end
    if isempty(gp.tau_arr), s.tau_arr=[1,0.7*ones(1,N-1)]; else, s.tau_arr=gp.tau_arr(:)'; end
    
    %% Build UI
    fig=figure('Name','平面绘图程序 (Simple)','NumberTitle','off','Units','normalized',...
        'Position',[0.02 0.02 0.96 0.94],'Color',[0.15 0.15 0.15],...
        'CloseRequestFcn',@onClose,'KeyPressFcn',@onKey);
    
    % --- LEFT: 参数面板 ---
    hParamPanel=uipanel('Parent',fig,'Units','normalized',...
        'Position',[0.01 0.01 0.17 0.97],'Background',[0.14 0.14 0.18],...
        'Title','参数','Foreground',[0.9 0.9 1],'FontSize',8);
    rH=0.035; py=0.96; hC={};
    [hN,hL,py]=addTwoCol(hParamPanel,py,rH,'N:',num2str(N),'L(m):',num2str(L_seg),hC);
    [hQmin,hQmax,py]=addTwoCol(hParamPanel,py,rH,'q下限:',num2str(gp.q_min),'q上限:',num2str(gp.q_max),hC);
    [hDqMax,hBeta,py]=addTwoCol(hParamPanel,py,rH,'dq最大:',num2str(gp.dq_step_max),'β:',num2str(gp.momentum_beta),hC);
    [hMaxIter,hTol,py]=addTwoCol(hParamPanel,py,rH,'迭代:',num2str(gp.max_iter),'容差:',num2str(gp.lambda_m),hC);
    uicontrol(hParamPanel,'Style','text','String','-- 权重 --','Units','normalized',...
        'Position',[0.02 py-0.015 0.96 0.015],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.6 0.3],'FontSize',6,'HorizontalAlignment','center');
    py=py-0.02;
    [hWPos,hWAng,py]=addTwoCol(hParamPanel,py,rH,'w_pos:',num2str(gp.w_pos),'w_ang:',num2str(gp.w_ang),hC);
    [hWObs,hWVar,py]=addTwoCol(hParamPanel,py,rH,'w_obs:',num2str(gp.w_obs),'w_var:',num2str(gp.w_var),hC);
    [hWAcc,~,py]=addTwoCol(hParamPanel,py,rH,'w_acc:',num2str(gp.w_acc),'','',hC);
    uicontrol(hParamPanel,'Style','text','String','-- 障碍 --','Units','normalized',...
        'Position',[0.02 py-0.015 0.96 0.015],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.6 0.3],'FontSize',6,'HorizontalAlignment','center');
    py=py-0.02;
    [hRho0,hSafeM,py]=addTwoCol(hParamPanel,py,rH,'ρ0:',num2str(gp.rho0),'安全:',num2str(gp.safe_margin),hC);
    [hBarEps,~,py]=addTwoCol(hParamPanel,py,rH,'ε:',num2str(gp.barrier_eps),'','',hC);
    uicontrol(hParamPanel,'Style','text','String','-- 全局搜索 --','Units','normalized',...
        'Position',[0.02 py-0.015 0.96 0.015],'Background',[0.14 0.14 0.18],...
        'Foreground',[1 0.6 0.3],'FontSize',6,'HorizontalAlignment','center');
    py=py-0.02;
    hParamRRT=uicontrol(hParamPanel,'Style','popupmenu',...
        'String',{'关闭','多启动RRT','RRT*最优','PRM*路图'},'Value',1,...
        'Units','normalized','Position',[0.05 py-rH 0.90 rH],...
        'Callback',@onReconfig,'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
    py=py-rH-0.003;
    [hRrtTrees,hRrtSamp,py]=addTwoCol(hParamPanel,py,rH,'树数:',num2str(gp.rrt_num_trees),'采样:',num2str(gp.rrt_max_samples),hC);
    [hRrtStep,hRrtEps,py]=addTwoCol(hParamPanel,py,rH,'步长:',num2str(gp.rrt_max_step),'容差:',num2str(gp.rrt_goal_eps),hC);
    [hRrtStarIter,hRrtStarRad,py]=addTwoCol(hParamPanel,py,rH,'RRT*迭:',num2str(gp.rrt_star_max_iter),'半径:',num2str(gp.rrt_star_radius),hC);
    uicontrol(hParamPanel,'Style','pushbutton','String','应用并重建',...
        'Units','normalized','Position',[0.04 py-0.06 0.92 0.04],...
        'Callback',@onReconfig,'Background',[0.2 0.5 0.2],'Foreground',[1 1 1],'FontWeight','bold','FontSize',8);

    ax=axes('Parent',fig,'Units','normalized','Position',[0.27 0.08 0.49 0.88],...
        'Color',[0.2 0.2 0.2],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7],...
        'GridColor',[0.35 0.35 0.35],'Box','on');
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    margin=0.3*max_reach;
    xlim(ax,[-margin,max_reach+margin]); ylim(ax,[-max_reach*0.5-margin,max_reach+margin]);
    xlabel(ax,'X (m)','Color',[0.8 0.8 0.8],'FontSize',10);
    ylabel(ax,'Y (m)','Color',[0.8 0.8 0.8],'FontSize',10);
    
    % --- 右侧控制面板 ---
    panel=uipanel('Parent',fig,'Units','normalized',...
        'Position',[0.77 0.02 0.22 0.95],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.9 0.9 0.9],'Title','控制','FontSize',9);
    y0=0.97;
    uicontrol(panel,'Style','text','String','模式:','Units','normalized',...
        'Position',[0.05 y0-0.04 0.35 0.03],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.8 0.8 0.8],'HorizontalAlignment','left');
    hMode=uicontrol(panel,'Style','popupmenu',...
        'String',{'调试','接口','仿真'},'Value',1,...
        'Units','normalized','Position',[0.40 y0-0.04 0.55 0.03],...
        'Callback',@onModeChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1]);
    y0=y0-0.06;
    hDrawBtn=uicontrol(panel,'Style','togglebutton',...
        'String','绘图: 关','Value',0,'Units','normalized',...
        'Position',[0.05 y0-0.035 0.44 0.04],'Callback',@onDrawToggle,...
        'Background',[0.3 0.3 0.3],'Foreground',[1 1 1],'FontWeight','bold');
    hSegBtn=uicontrol(panel,'Style','togglebutton',...
        'String','线段: 关','Value',0,'Units','normalized',...
        'Position',[0.52 y0-0.035 0.43 0.04],'Callback',@onSegToggle,...
        'Background',[0.3 0.3 0.3],'Foreground',[1 1 1],'FontWeight','bold','FontSize',8);
    y0=y0-0.055;
    uicontrol(panel,'Style','pushbutton','String','清除绘图',...
        'Units','normalized','Position',[0.05 y0-0.035 0.90 0.035],...
        'Callback',@onClearDraw,'Background',[0.4 0.2 0.2],'Foreground',[1 1 1]);
    y0=y0-0.045;
    uicontrol(panel,'Style','pushbutton','String','绘图→障碍',...
        'Units','normalized','Position',[0.05 y0-0.035 0.44 0.035],...
        'Callback',@onDrawToObs,'Background',[0.5 0.3 0.1],'Foreground',[1 1 1],...
        'FontWeight','bold','FontSize',8);
    uicontrol(panel,'Style','pushbutton','String','清除障碍',...
        'Units','normalized','Position',[0.52 y0-0.035 0.43 0.035],...
        'Callback',@onClearObs,'Background',[0.4 0.2 0.2],'Foreground',[1 1 1],'FontSize',8);
    hShowPot=uicontrol(panel,'Style','togglebutton','String','势场: 关',...
        'Units','normalized','Position',[0.05 y0-0.07 0.90 0.03],...
        'Callback',@onTogglePot,'Background',[0.2 0.3 0.5],...
        'Foreground',[1 1 1],'FontSize',7,'Value',0);
    y0=y0-0.05;
    uicontrol(panel,'Style','text','String','目标:','Units','normalized',...
        'Position',[0.05 y0-0.025 0.90 0.02],'Background',[0.18 0.18 0.18],...
        'Foreground',[0.8 1 0.8],'FontSize',8);
    y0=y0-0.03;
    uicontrol(panel,'Style','text','String','X:','Units','normalized',...
        'Position',[0.05 y0-0.025 0.15 0.025],'Background',[0.18 0.18 0.18],'Foreground',[1 1 1],'FontSize',8);
    hTgtX=uicontrol(panel,'Style','edit','String',num2str(s.X_tgt(1)),...
        'Units','normalized','Position',[0.20 y0-0.025 0.75 0.025],...
        'Callback',@onTgtChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.035;
    uicontrol(panel,'Style','text','String','Y:','Units','normalized',...
        'Position',[0.05 y0-0.025 0.15 0.025],'Background',[0.18 0.18 0.18],'Foreground',[1 1 1],'FontSize',8);
    hTgtY=uicontrol(panel,'Style','edit','String',num2str(s.X_tgt(2)),...
        'Units','normalized','Position',[0.20 y0-0.025 0.75 0.025],...
        'Callback',@onTgtChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.035;
    uicontrol(panel,'Style','text','String','角度:', 'Units','normalized',...
        'Position',[0.05 y0-0.025 0.20 0.025],'Background',[0.18 0.18 0.18],'Foreground',[1 1 1],'FontSize',8);
    hTgtTheta=uicontrol(panel,'Style','edit','String','0',...
        'Units','normalized','Position',[0.26 y0-0.025 0.69 0.025],...
        'Callback',@onSimThetaChange,'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.04;
    uicontrol(panel,'Style','pushbutton','String','复位归零',...
        'Units','normalized','Position',[0.05 y0-0.025 0.90 0.025],...
        'Callback',@onReset,'Background',[0.25 0.25 0.4],'Foreground',[1 1 1],'FontSize',8);
    y0=y0-0.04;
    uicontrol(panel,'Style','text','String','关节角度 (rad):','Units','normalized',...
        'Position',[0.05 y0-0.025 0.90 0.02],'Background',[0.18 0.18 0.18],...
        'Foreground',[1 1 0.5],'FontSize',8);
    y0=y0-0.03;
    hJointEdits=zeros(1,N); hJointSliders=zeros(1,N);
    jrH=min(0.03,0.25/N);
    for j=1:N
        uicontrol(panel,'Style','text','String',sprintf('q%d',j),'Units','normalized',...
            'Position',[0.05 y0-jrH 0.10 jrH],'Background',[0.18 0.18 0.18],...
            'Foreground',[1 1 1],'FontSize',7);
        hJointEdits(j)=uicontrol(panel,'Style','edit','String','0','Units','normalized',...
            'Position',[0.16 y0-jrH 0.55 jrH],'Callback',{@onJointEdit,j},...
            'Background',[0.25 0.25 0.25],'Foreground',[1 1 1],'FontSize',7);
        hJointSliders(j)=uicontrol(panel,'Style','slider','Min',-pi,'Max',pi,'Value',0,...
            'Units','normalized','Position',[0.73 y0-jrH 0.24 jrH],...
            'Callback',{@onJointSlider,j},'Background',[0.3 0.3 0.3]);
        y0=y0-jrH-0.003;
    end
    y0=y0-0.005;
    uicontrol(panel,'Style','pushbutton','String','初始','Units','normalized',...
        'Position',[0.05 y0-0.025 0.28 0.025],'Callback',@onQRst,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(panel,'Style','pushbutton','String','保存','Units','normalized',...
        'Position',[0.36 y0-0.025 0.28 0.025],'Callback',@onQSave,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    uicontrol(panel,'Style','pushbutton','String','设置','Units','normalized',...
        'Position',[0.67 y0-0.025 0.28 0.025],'Callback',@onQSet,...
        'Background',[0.3 0.3 0.5],'Foreground',[1 1 1],'FontSize',7);
    y0=y0-0.04;
    
    % --- SIM panel ---
    hSimPanel=uipanel('Parent',panel,'Units','normalized',...
        'Position',[0.02 y0-0.18 0.96 0.18],'Visible','off','Background',[0.16 0.16 0.16],...
        'Title','仿真','Foreground',[0.9 0.9 0.9],'FontSize',9);
    hSimStat=uicontrol(hSimPanel,'Style','text','String','就绪',...
        'Units','normalized','Position',[0.05 0.65 0.90 0.28],...
        'Background',[0.16 0.16 0.16],'Foreground',[0.5 1 0.5],'FontSize',9);
    hSimRun=uicontrol(hSimPanel,'Style','pushbutton','String','运行','Units','normalized',...
        'Position',[0.05 0.30 0.28 0.30],'Callback',@onSimRun,...
        'Background',[0.2 0.5 0.2],'Foreground',[1 1 1],'FontWeight','bold');
    hSimStop=uicontrol(hSimPanel,'Style','pushbutton','String','Stop','Units','normalized',...
        'Position',[0.37 0.30 0.28 0.30],'Callback',@onSimStop,...
        'Background',[0.5 0.2 0.2],'Foreground',[1 1 1],'Enable','off');
    hSimReplay=uicontrol(hSimPanel,'Style','pushbutton','String','Replay','Units','normalized',...
        'Position',[0.69 0.30 0.26 0.30],'Callback',@onReplay,...
        'Background',[0.2 0.2 0.4],'Foreground',[1 1 1],'Enable','off');
    hReplayFrame=uicontrol(hSimPanel,'Style','edit','String','1','Units','normalized',...
        'Position',[0.69 0.05 0.12 0.18],'Callback',@onReplayJump,...
        'Background',[0.25 0.25 0.25],'Foreground',[1 1 1]);
    hReplayStat=uicontrol(hSimPanel,'Style','text','String','','Units','normalized',...
        'Position',[0.05 0.05 0.62 0.18],'Background',[0.16 0.16 0.16],...
        'Foreground',[0.5 1 0.5],'FontSize',7,'HorizontalAlignment','left');
    hFps=uicontrol(fig,'Style','text','String','FPS: --','Units','normalized',...
        'Position',[0.85 0.96 0.12 0.025],'Background',[0.15 0.15 0.15],...
        'Foreground',[0.5 1 0.5],'FontSize',9);
    
    drawAll();
    
    fpsBuf=zeros(1,30); fpsIdx=1; tPrev=tic;
    timerObj=timer('Name','Timer','ExecutionMode','fixedRate','Period',1/gp.framerate,...
        'TimerFcn',@onTimerTick,'BusyMode','drop','StartDelay',0.1);
    start(timerObj);
    set(fig,'WindowButtonDownFcn',@onMouseDown,'WindowButtonMotionFcn',@onMouseMove,...
        'WindowButtonUpFcn',@onMouseUp);
    initSocket();
    
    app=struct(); app.figure=fig; app.getState=@() s;
    
    %% ====== Nested functions ======
    function onReconfig(~,~)
        try, nN=str2double(get(hN,'String')); nL=str2double(get(hL,'String'));
        catch, nN=s.N; nL=s.L_seg; end
        if isnan(nN)||nN<1||nN>20, nN=s.N; end; if isnan(nL)||nL<=0, nL=s.L_seg; end
        v=@(h,def) safeVal(h,def);
        GlobalParams('N',nN,'L_seg',nL,...
            'q_min',v(hQmin,gp.q_min),'q_max',v(hQmax,gp.q_max),...
            'dq_step_max',v(hDqMax,gp.dq_step_max),'momentum_beta',v(hBeta,gp.momentum_beta),...
            'max_iter',v(hMaxIter,gp.max_iter),'lambda_m',v(hTol,gp.lambda_m),...
            'w_pos',v(hWPos,gp.w_pos),'w_ang',v(hWAng,gp.w_ang),...
            'w_obs',v(hWObs,gp.w_obs),'w_var',v(hWVar,gp.w_var),'w_acc',v(hWAcc,gp.w_acc),...
            'rho0',v(hRho0,gp.rho0),'safe_margin',v(hSafeM,gp.safe_margin),...
            'barrier_eps',v(hBarEps,gp.barrier_eps),...
            'use_global_search',get(hParamRRT,'Value')-1,...
            'rrt_num_trees',v(hRrtTrees,gp.rrt_num_trees),'rrt_max_samples',v(hRrtSamp,gp.rrt_max_samples),...
            'rrt_max_step',v(hRrtStep,gp.rrt_max_step),'rrt_goal_eps',v(hRrtEps,gp.rrt_goal_eps),...
            'rrt_star_max_iter',v(hRrtStarIter,gp.rrt_star_max_iter),'rrt_star_radius',v(hRrtStarRad,gp.rrt_star_radius));
        s.N=nN; s.L_seg=nL; s.q=zeros(1,nN);
        s.sig0_arr=[1,0.06*ones(1,max(0,nN-1))]; s.tau_arr=[1,0.7*ones(1,max(0,nN-1))]; s.m_arr=zeros(1,nN);
        max_reach=nN*nL; s.X_tgt=[max_reach*0.6,max_reach*0.4];
        if nN~=length(s.q), warndlg('Params updated. Reopen for full refresh.','Tip'); end
        syncUI(); drawAll();
    end
    function val=safeVal(h,def)
        val=str2double(get(h,'String')); if isnan(val)||val<0, val=def; end
    end
    
    function onModeChange(~,~)
        modes={'debug','interface','sim'}; s.mode=modes{get(hMode,'Value')};
        set(hSimPanel,'Visible','off');
        if strcmp(s.mode,'sim'), set(hSimPanel,'Visible','on'); onSimRun(); end
    end
    function onTgtChange(~,~)
        xv=str2double(get(hTgtX,'String')); yv=str2double(get(hTgtY,'String'));
        if ~isnan(xv)&&~isnan(yv), s.X_tgt=[xv,yv]; drawAll(); end
    end
    function onSimThetaChange(~,~)
        val=str2double(get(hTgtTheta,'String'));
        if ~isnan(val), s.theta_tgt=val; set(hTgtTheta,'String',sprintf('%.3f',val)); end
    end
    function onReset(~,~), s.q=zeros(1,s.N); syncUI(); drawAll(); end
    function onQRst(~,~)
        for j=1:s.N
            if mod(j,2)==1, s.q(j)=pi; else, s.q(j)=-pi; end
        end; syncUI(); drawAll();
    end
    function onQSave(~,~), s.saved_q=s.q; fprintf('[Saved] joints recorded\n'); end
    function onQSet(~,~), s.q=s.saved_q; syncUI(); drawAll(); end
    function onClearObs(~,~), s.obs=[]; s.obsLines={}; GlobalParams('obs',[],'obs_lines',{}); drawAll(); end
    function syncUI()
        for j=1:s.N
            set(hJointEdits(j),'String',sprintf('%.3f',s.q(j)*180/pi));
            if ishandle(hJointSliders(j)), set(hJointSliders(j),'Value',s.q(j)); end
        end
        set(hTgtX,'String',num2str(s.X_tgt(1))); set(hTgtY,'String',num2str(s.X_tgt(2)));
    end
    function onJointEdit(src,~,j)
        val=str2double(src.String);
        if ~isnan(val), s.q(j)=val*pi/180; syncUI(); drawAll(); end
    end
    function onJointSlider(src,~,j)
        s.q(j)=src.Value; set(hJointEdits(j),'String',sprintf('%.3f',s.q(j)*180/pi)); drawAll();
    end
    function onTogglePot(~,~)
        s.showPotential=get(hShowPot,'Value');
        if s.showPotential, set(hShowPot,'String','势场: 开','Background',[0.5 0.3 0.1]);
        else, set(hShowPot,'String','势场: 关','Background',[0.2 0.3 0.5]); end
        drawAll();
    end
    function onDrawToggle(~,~)
        s.drawOn=get(hDrawBtn,'Value');
        if s.drawOn, s.segDrawOn=false; set(hSegBtn,'Value',0); set(hDrawBtn,'String','绘图: 开','Background',[0.2 0.6 0.2]);
        else, set(hDrawBtn,'String','绘图: 关','Background',[0.3 0.3 0.3]); end
    end
    function onSegToggle(~,~)
        s.segDrawOn=get(hSegBtn,'Value');
        if s.segDrawOn, s.drawOn=false; set(hDrawBtn,'Value',0,'String','绘图: 关','Background',[0.3 0.3 0.3]);
            s.segPt=[]; set(hSegBtn,'String','线段: 开','Background',[0.2 0.6 0.2]);
        else, s.segPt=[]; set(hSegBtn,'String','线段: 关','Background',[0.3 0.3 0.3]); end
    end
    function onClearDraw(~,~), s.drawLines={}; s.drawCurLine=[]; s.segLines={}; s.segPt=[]; drawAll(); end
    function onDrawToObs(~,~)
        obsLines={}; obs=[];
        for k=1:length(s.drawLines)
            pts=s.drawLines{k};
            for i=2:size(pts,1), obsLines{end+1}=[pts(i-1,:); pts(i,:)]; end
        end
        for k=1:length(s.segLines)
            obsLines{end+1}=s.segLines{k};
        end
        s.obs=obs; s.obsLines=obsLines;
        GlobalParams('obs',obs,'obs_lines',obsLines);
        set(hSimStat,'String',sprintf('障碍: %d条线段',length(obsLines)),'Foreground',[1 0.8 0.3]);
    end
    
    function onSimRun(~,~)
        if s.simRunning, return; end
        s.simRunning=true; set(hSimRun,'Enable','off'); set(hSimStop,'Enable','on');
        runSim();
    end
    function onSimStop(~,~)
        s.simStopReq=true; s.simRunning=false;
        set(hSimRun,'Enable','on'); set(hSimStop,'Enable','off');
    end
    function runSim()
        gp=GlobalParams();
        params=struct();
        params.N=s.N; params.L_seg=s.L_seg; params.theta_end_target=s.theta_tgt;
        off=0.06; params.rod_offset_arr=zeros(1,s.N);
        for k=1:s.N, params.rod_offset_arr(k)=off*(-1)^(k+1); end
        params.X_target=s.X_tgt;
        params.q_min=gp.q_min*ones(1,s.N); params.q_max=gp.q_max*ones(1,s.N);
        params.dq_step_max=gp.dq_step_max;
        params.max_iter=gp.max_iter; params.lambda_m=gp.lambda_m;
        params.w_pos=gp.w_pos; params.w_ang=gp.w_ang;
        params.w_obs=gp.w_obs; params.w_var=gp.w_var; params.w_acc=gp.w_acc;
        params.momentum_beta=gp.momentum_beta; params.barrier_eps=gp.barrier_eps;
        params.m_arr=maybeFill(gp.m_arr,s.N,0);
        params.sig0_arr=maybeFill(gp.sig0_arr,s.N,[1,0.06]);
        params.tau_arr=maybeFill(gp.tau_arr,s.N,[1,0.7]);
        s.m_arr=params.m_arr; s.sig0_arr=params.sig0_arr; s.tau_arr=params.tau_arr;
        params.obs=[]; params.rho0=gp.rho0; params.safe_margin=gp.safe_margin;
        params.obs_lines={};
        if isfield(s,'obs')&&~isempty(s.obs), params.obs=s.obs; end
        if isfield(s,'obsLines')&&~isempty(s.obsLines), params.obs_lines=s.obsLines; end
        params.q_init=s.q;
        params.use_global_search=gp.use_global_search;
        params.rrt_max_samples=gp.rrt_max_samples; params.rrt_max_step=gp.rrt_max_step;
        params.rrt_goal_bias=gp.rrt_goal_bias; params.rrt_goal_eps=gp.rrt_goal_eps;
        params.rrt_num_trees=gp.rrt_num_trees;
        params.rrt_star_max_iter=gp.rrt_star_max_iter; params.rrt_star_radius=gp.rrt_star_radius;
        
        resultQ=runLArmIK_2D_Simple(params,100);
        if ~isempty(resultQ), s.q=resultQ; syncUI(); end
        [~,pe]=planarFK_api(s.q,s.N,s.L_seg); err=norm(s.X_tgt-pe);
        set(hSimStat,'String',sprintf('Done|end[%.2f,%.2f]|err:%.4f',pe(1),pe(2),err),'Foreground',[0.5 1 0.5]);
        s.simConverged=(err<params.lambda_m);
        s.simRunning=false; set(hSimRun,'Enable','on'); set(hSimStop,'Enable','off');
        
        % Record replay
        nFrames=max(20,100);
        s.replayFrames=cell(1,nFrames);
        q_start=params.q_init; q_end=s.q;
        for fi=1:nFrames
            t=(fi-1)/(nFrames-1);
            s.replayFrames{fi}=(1-t)*q_start+t*q_end;
        end
        set(hSimReplay,'Enable','on');
        set(hReplayStat,'String',sprintf('Recorded %d frames',nFrames));
        drawAll();
    end
    function arr=maybeFill(arr,Nn,def)
        if isempty(arr)||length(arr)~=Nn
            if length(def)==1, arr=def*ones(1,Nn); else, arr=[def(1),def(2)*ones(1,Nn-1)]; end
        else, arr=arr(:)'; end
    end
    function [pa,pe]=planarFK_api(qq,NS,L)
        pn=zeros(NS+1,2); pn(1,:)=[0 0]; th=0;
        for k=1:NS, th=th+qq(k); pn(k+1,:)=pn(k,:)+L*[cos(th),sin(th)]; end
        pa=pn; pe=pn(NS+1,:);
    end
    
    function onReplay(~,~)
        if isempty(s.replayFrames), set(hReplayStat,'String','No data'); return; end
        if s.replayActive
            s.replayActive=false; set(hSimReplay,'String','Replay','Background',[0.2 0.2 0.4]);
            set(hReplayStat,'String',sprintf('Stopped (%d)',length(s.replayFrames))); drawAll(); return;
        end
        s.replayActive=true; s.replayIdx=1; s.q=s.replayFrames{1};
        set(hSimReplay,'String','StopR','Background',[0.6 0.2 0.2]);
        set(hReplayStat,'String',sprintf('Replay: 1/%d',length(s.replayFrames)));
        set(hReplayFrame,'String','1'); drawAll();
    end
    function onReplayJump(~,~)
        if isempty(s.replayFrames), return; end
        fn=str2double(get(hReplayFrame,'String'));
        if isnan(fn)||fn<1, fn=1; end
        if fn>length(s.replayFrames), fn=length(s.replayFrames); end
        fn=round(fn); s.replayIdx=fn; s.q=s.replayFrames{fn};
        set(hReplayFrame,'String',num2str(fn));
        set(hReplayStat,'String',sprintf('Jump %d/%d',fn,length(s.replayFrames))); drawAll();
    end
    
    function initSocket()
        try, s.udpSock=udp('127.0.0.1','LocalPort',s.udpPort,'Timeout',0.1,'InputBufferSize',8192);
            fopen(s.udpSock); catch, s.udpSock=[]; end
    end
    function onTimerTick(~,~)
        try
            if strcmp(s.mode,'interface')&&s.ifActive&&~isempty(s.udpSock), readInterface(); end
            if s.replayActive&&~isempty(s.replayFrames)
                s.replayIdx=s.replayIdx+s.replaySpeed;
                if s.replayIdx>length(s.replayFrames)
                    s.replayActive=false; s.replayIdx=length(s.replayFrames);
                    s.q=s.replayFrames{end}; set(hSimReplay,'String','Replay','Background',[0.2 0.2 0.4]);
                    set(hReplayStat,'String',sprintf('End (%d)',length(s.replayFrames)));
                    set(hReplayFrame,'String',num2str(length(s.replayFrames))); drawAll(); return;
                end
                s.q=s.replayFrames{s.replayIdx};
                set(hReplayStat,'String',sprintf('Replay: %d/%d',s.replayIdx,length(s.replayFrames)));
                set(hReplayFrame,'String',num2str(s.replayIdx)); drawAll();
            end
            dt=toc(tPrev); tPrev=tic; fpsBuf(fpsIdx)=1/max(dt,0.001); fpsIdx=mod(fpsIdx,30)+1;
            set(hFps,'String',sprintf('FPS: %.1f',mean(fpsBuf(fpsBuf>0))));
        catch, end
    end
    function readInterface()
        try
            nb=s.udpSock.BytesAvailable;
            if nb>0
                data=fread(s.udpSock,nb,'uint8'); str=native2unicode(data','UTF-8');
                s.uBytes=s.uBytes+nb; s.uCount=s.uCount+1;
                try
                    parts=strsplit(str,','); nums=str2double(parts);
                    if length(nums)>=2+s.N
                        s.X_tgt=nums(1:2);
                        if length(nums)>=3+s.N, s.theta_tgt=nums(3); end
                        if length(nums)>=3+s.N, s.q=nums(4:end); end
                    end
                catch, end
            end
        catch, end
    end

    function drawAll()
        cla(ax); hold(ax,'on'); grid(ax,'on');
        xlim(ax,[-0.3 s.N*s.L_seg+0.3]); ylim(ax,[-0.3 s.N*s.L_seg+0.3]); axis(ax,'equal');
        % Draw potential field FIRST (behind everything)
        if s.showPotential, drawPotential(); end
        % Draw arm + target on top
        [p_all,p_end]=planarFK_api(s.q,s.N,s.L_seg);
        plot(ax,p_all(:,1),p_all(:,2),'b-','LineWidth',2.5);
        plot(ax,p_all(1,1),p_all(1,2),'ko','MarkerFaceColor','k','MarkerSize',6);
        plot(ax,p_all(2:end-1,1),p_all(2:end-1,2),'ko','MarkerFaceColor','w','MarkerSize',5);
        plot(ax,p_end(1),p_end(2),'ro','MarkerFaceColor','r','MarkerSize',8);
        plot(ax,s.X_tgt(1),s.X_tgt(2),'gx','MarkerSize',12,'LineWidth',2);
        for k=1:length(s.drawLines)
            if ~isempty(s.drawLines{k}), plot(ax,s.drawLines{k}(:,1),s.drawLines{k}(:,2),'r-','LineWidth',1); end
        end
        for k=1:length(s.segLines)
            L=s.segLines{k}; plot(ax,[L(1,1) L(2,1)],[L(1,2) L(2,2)],'r-','LineWidth',1);
        end
        if isfield(s,'obs')&&~isempty(s.obs)
            for o=1:size(s.obs,1)
                rectangle(ax,'Position',[s.obs(o,1)-s.obs(o,3),s.obs(o,2)-s.obs(o,3),...
                    2*s.obs(o,3),2*s.obs(o,3)],'Curvature',[1 1],'FaceColor',[1 0.5 0 0.3],'EdgeColor','r');
            end
        end
        if isfield(s,'obsLines')&&~isempty(s.obsLines)
            for k=1:length(s.obsLines)
                L=s.obsLines{k}; plot(ax,[L(1,1) L(2,1)],[L(1,2) L(2,2)],'r-','LineWidth',2.5);
            end
        end
    end
    
    function drawPotential()
        if ~isfield(s,'obs'), s.obs=[]; end
        if ~isfield(s,'obsLines'), s.obsLines={}; end
        if isempty(s.obs)&&isempty(s.obsLines)
            % No obstacles: show a flat neutral background
            xl=xlim(ax); yl=ylim(ax);
            [X,Y]=meshgrid(linspace(xl(1),xl(2),10),linspace(yl(1),yl(2),10));
            Z=zeros(10,10);
            pcolor(ax,X,Y,Z); shading(ax,'interp'); colormap(ax,hot); caxis(ax,[0,8]);
            return;
        end
        xl=xlim(ax); yl=ylim(ax);
        nx=60; ny=60;
        xs=linspace(xl(1),xl(2),nx); ys=linspace(yl(1),yl(2),ny);
        [X,Y]=meshgrid(xs,ys);
        Z=zeros(ny,nx);
        rho0_v=gp.rho0; safe=gp.safe_margin; d_safe=rho0_v+safe;
        eps_v=0.001;
        for ix=1:nx
            for iy=1:ny
                pt=[X(iy,ix),Y(iy,ix)];
                d_min=Inf;
                if ~isempty(s.obs)
                    for o=1:size(s.obs,1)
                        d=norm(pt-s.obs(o,1:2))-s.obs(o,3);
                        if d<d_min, d_min=d; end
                    end
                end
                if ~isempty(s.obsLines)
                    for k=1:length(s.obsLines)
                        L=s.obsLines{k};
                        d=segPointDist(L(1,:),L(2,:),pt);
                        if d<d_min, d_min=d; end
                    end
                end
                if d_min<d_safe+1.0
                    margin=max(d_min-d_safe,eps_v);
                    Z(iy,ix)=-log(margin);
                else, Z(iy,ix)=0; end
            end
        end
        Z(Z>8)=8; Z(Z<0)=0;
        h_pc=pcolor(ax,X,Y,Z);
        shading(ax,'interp');
        uistack(h_pc,'bottom');
        colormap(ax,hot);
        caxis(ax,[0,8]);
        hold(ax,'on');
        contour(ax,X,Y,Z,15,'LineColor',[0.5 0.5 0.5],'LineWidth',0.4,'LineStyle',':');
    end
    function d=segPointDist(a,b,p)
        ab=b-a; ap=p-a; t=dot(ap,ab)/max(dot(ab,ab),1e-12);
        t=max(0,min(1,t)); near=a+t*ab; d=norm(p-near);
    end

    function onMouseDown(~,~)
        cp=get(ax,'CurrentPoint'); pt=[cp(1,1),cp(1,2)];
        if s.drawOn, s.drawIsDrawing=true; s.drawCurLine=pt; return; end
        if s.segDrawOn
            if isempty(s.segPt), s.segPt=pt;
            else, s.segLines{end+1}=[s.segPt;pt]; s.segPt=pt; end
            drawAll(); return;
        end
        [p_all,~]=planarFK_api(s.q,s.N,s.L_seg);
        for j=s.N+1:-1:1
            if norm(pt-p_all(j,:))<0.15
                if j==s.N+1, s.dragMode='effector'; else, s.dragMode='joint'; s.dragIdx=j; end
                return;
            end
        end
        if norm(pt-s.X_tgt)<0.2, s.dragMode='target'; end
    end
    function dragJoint(pt,jc)
        [p_all,~]=planarFK_api(s.q,s.N,s.L_seg);
        prev=p_all(max(jc-1,1),:); vec=pt-prev;
        if norm(vec)<1e-8, return; end
        ang=atan2(vec(2),vec(1)); prevSum=sum(s.q(1:jc-1));
        nr=ang-prevSum; nr=atan2(sin(nr),cos(nr)); s.q(jc)=nr; drawAll();
    end
    function onMouseMove(~,~)
        if s.drawOn&&s.drawIsDrawing, cp=get(ax,'CurrentPoint'); s.drawCurLine(end+1,:)=[cp(1,1),cp(1,2)]; return; end
        if strcmp(s.dragMode,'none'), return; end
        cp=get(ax,'CurrentPoint'); pt=[cp(1,1),cp(1,2)];
        switch s.dragMode
            case 'target', s.X_tgt=pt;
            case 'effector', dragJoint(pt,s.N+1);
            case 'joint', dragJoint(pt,s.dragIdx);
        end; drawAll();
    end
    function onMouseUp(~,~)
        if s.drawOn&&s.drawIsDrawing, s.drawIsDrawing=false;
            if size(s.drawCurLine,1)>=2, s.drawLines{end+1}=s.drawCurLine; end
            s.drawCurLine=[]; return;
        end
        s.dragMode='none'; s.dragIdx=0;
    end
    
    function onKey(~,evt)
        switch evt.Key
            case 'f', s.drawOn=~s.drawOn; set(hDrawBtn,'Value',s.drawOn);
                if s.drawOn, s.segDrawOn=false; set(hSegBtn,'Value',0); end
            case 's', s.segDrawOn=~s.segDrawOn; set(hSegBtn,'Value',s.segDrawOn);
                if s.segDrawOn, s.drawOn=false; set(hDrawBtn,'Value',0); s.segPt=[]; end; drawAll();
            case 'p', onReplay();
            case 'c', s.drawLines={}; s.drawCurLine=[]; s.segLines={}; s.segPt=[]; drawAll();
            case 'r', s.q=zeros(1,s.N); drawAll();
            case 'm', v=get(hMode,'Value'); set(hMode,'Value',mod(v,3)+1); onModeChange(hMode);
            case 'escape', s.drawIsDrawing=false; s.drawCurLine=[]; s.dragMode='none'; s.dragIdx=0;
                s.segDrawOn=false; s.segPt=[]; set(hSegBtn,'Value',0); drawAll();
        end
    end
    function onClose(~,~)
        if s.simRunning, onSimStop(); end
        try stop(timerObj); delete(timerObj); catch, end
        try if ~isempty(s.udpSock)&&strcmp(s.udpSock.Status,'open'), fclose(s.udpSock); delete(s.udpSock); end, catch, end
        try delete(fig); catch, end
    end
    function setJointsExt(qv), if length(qv)==s.N, s.q=qv(:)'; drawAll(); end, end
    function setTargetExt(xy), if length(xy)>=2, s.X_tgt=xy(1:2); drawAll(); end, end
    
    function [h1,h2,np]=addTwoCol(par,py0,rHi,lb1,d1,lb2,d2,hList)
        uicontrol(par,'Style','text','String',lb1,'Units','normalized',...
            'Position',[0.03 py0-rHi 0.14 rHi],'Background',[0.14 0.14 0.18],...
            'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
        h1=uicontrol(par,'Style','edit','String',d1,'Units','normalized',...
            'Position',[0.18 py0-rHi 0.28 rHi],'Callback',@onReconfig,...
            'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
        uicontrol(par,'Style','text','String',lb2,'Units','normalized',...
            'Position',[0.48 py0-rHi 0.14 rHi],'Background',[0.14 0.14 0.18],...
            'Foreground',[0.9 0.9 0.9],'FontSize',7,'HorizontalAlignment','left');
        h2=uicontrol(par,'Style','edit','String',d2,'Units','normalized',...
            'Position',[0.63 py0-rHi 0.28 rHi],'Callback',@onReconfig,...
            'Background',[0.2 0.2 0.3],'Foreground',[1 1 1],'FontSize',7);
        hList{end+1}=h1; hList{end+1}=h2;
        np=py0-rHi-0.003;
    end
end