function out = tcpSendMotion(host, port, traj, q0, varargin)
%tcpSendMotion 按步进电机约束【锁步】下发：相对角度（度，最小步进整数倍）+ 夹爪 0/1
%   out = tcpSendMotion(host, port, traj, q0, opts)
%   traj : struct('q',[K×6] 绝对关节角 rad, 'gripper',[1×K] 0=闭合/1=打开, 't',[1×K]|[])
%   q0   : [1×6] 初始绝对关节角 rad（第一组相对增量从其算；缺省 zeros）
%   opts : .step_per_rev(40000) → step_deg = 360/step_per_rev = 0.009°
%          .decimate(n) 每 n 步发一组；.command_id；.estop @()bool
%          .onMove @(msg)  每收到一帧 ACK 后回调一行进度（GUI 用它流式刷新）
%   返回 out：.log(cell) .q_actual([nSend×6] rad, 每次发送后【应用后的真实位姿】)
%             .q_actual_final([1×6] rad, 下发完后的最终真实位姿)——可用于回写修正仿真位姿
%   协议：HELLO → ACK；然后逐组 MOVE{rel_deg[6],gripper} → 等 MOVE_ACK{ok:1/0}；ok==1 才发下一组。
    p = inputParser;
    addParameter(p,'step_per_rev', 40000);
    addParameter(p,'decimate', 1);
    addParameter(p,'command_id','CMD-STP');
    addParameter(p,'estop', @() false);
    addParameter(p,'onMove', @(m) []);   % 每帧进度回调（异步流式）
    parse(p,varargin{:});
    step_deg = 360/p.Results.step_per_rev;       % 最小步进角(度)
    decim = max(1, round(p.Results.decimate));
    cid = p.Results.command_id;  estop = p.Results.estop;  onMove = p.Results.onMove;
    if nargin < 4 || isempty(q0), q0 = zeros(1, size(traj.q,2)); end

    qdeg = traj.q*180/pi;                        % 绝对关节角(度)
    g    = traj.gripper(:).';
    K    = size(qdeg,1);
    idx  = [1:decim:K];  if isempty(idx) || idx(end) ~= K, idx = [idx K]; end  % 每 decim 步 + 末点
    qprev = q0*180/pi;                           % 当前已应用绝对角(度)

    t = tcpclient(host, port, 'ConnectTimeout', 5);
    log = {};  seq = 0;  q_actual = zeros(numel(idx), size(traj.q,2));
    try
        write(t, tcpEncodeFrame('HELLO', struct('device','matlab','node','planner','version','1.0'), struct('seq',seq+1)));
        seq = seq+1;  ack = tcpclientReadFrame(t);
        log{end+1} = sprintf('HELLO_ACK ok=%d', ack.data.ok);
        onMove(log{end});

        for i = 1:numel(idx)
            k = idx(i);
            rel = qdeg(k,:) - qprev;             % 相对增量(度)
            rel_steps = round(rel/step_deg);     % 最小步进整数倍
            rel_sent  = rel_steps*step_deg;      % 量化后的相对角(度)
            gr = g(min(k, end));
            write(t, tcpEncodeFrame('MOVE', struct('seq', k, 'rel_deg', rel_sent, 'gripper', gr), ...
                struct('seq', seq+1, 'command_id', cid)));
            seq = seq+1;
            ma = tcpclientReadFrame(t);          % 等电控 MOVE_ACK（任务完成信号）
            log{end+1} = sprintf('MOVE k=%d rel=%s gripper=%d ack=%d', ...
                k, mat2str(rel_sent, 4), gr, ma.data.ok);
            onMove(log{end});
            if ma.data.ok ~= 1
                log{end+1} = 'ABORT: 电控返回失败(0)';
                onMove(log{end});
                break;
            end
            qprev = qprev + rel_sent;            % 应用后绝对角（累计，防步进累计漂移）
            q_actual(i,:) = qprev*pi/180;        % 记录本次发送后【真实位姿】(rad)
            if estop(), break; end
        end
        delete(t);
        out = struct('log', {log}, 'q_actual', q_actual, ...
            'q_actual_final', qprev*pi/180);     % 回写仿真用：下发完后的真实位姿(rad)
    catch e
        try, delete(t); catch, end
        rethrow(e);
    end
end
