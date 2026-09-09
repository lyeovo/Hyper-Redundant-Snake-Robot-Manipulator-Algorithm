function h = tcpPushMotorCmd(port_or_cp, varargin)
%tcpPushMotorCmd 【视觉/算法侧】MOTOR_CMD 推送服务器 —— 支持「长连接总控」与「一次性推帧」两种用法
%================================================================================================
%  用途：视觉/算法侧 = TCP 服务端，把 motorCmdToControlParams 生成的 26 参帧推给 dSPACE Interpreter 客户端。
%        客户端(interpreter_motor_client.py)作为【客户端】连入后持续收帧（本服务端保持连接）。
%
%  用法一（长连接总控，推荐，配合视觉 GUI 一键启动）：
%      h = tcpPushMotorCmd('open', port [, 'run',run])
%      tcpPushMotorCmd('accept', h)                % 等客户端连入(一次)，之后保持连接
%      tcpPushMotorCmd('push', h, cp [, 'run',run]) % 推一个任务的 26 参帧(不关闭连接)
%      tcpPushMotorCmd('close', h)                  % 全部结束，关闭并清理
%
%  用法二（一次性，每任务一连接，自测用）：log_cell = tcpPushMotorCmd(cp, port [, 'run',run])
%
%  h : struct('method','push','server',ServerSocket,'sock',Socket|[],'dos',DataOutputStream|[],
%             'ready_file',char,'port',port,'run',run,'interval',double,'connected',bool)
%
%  帧协议与 tcpEncodeFrame/tcpSendControl 一致：[4 字节大端长度][JSON]。
    if ischar(port_or_cp) && any(strcmp(port_or_cp, {'open','accept','push','close','byeclose'}))
        out = push_dispatch(port_or_cp, varargin{:});
        if isstruct(out), h = out; else, h = out; end
    else
        % 用法二（一次性）：cp 在第一个参数
        cp = port_or_cp;  port = varargin{1};
        p = inputParser;
        addParameter(p, 'run', 1); addParameter(p, 'interval', 0);
        parse(p, varargin{2:end});
        h0 = tcpPushMotorCmd('open', port, 'run', p.Results.run, 'interval', p.Results.interval);
        tcpPushMotorCmd('accept', h0);
        tcpPushMotorCmd('push', h0, cp, 'run', p.Results.run);
        tcpPushMotorCmd('byeclose', h0);
        h = h0.fired_log;                          % 一次性：返回推帧日志(简化)
    end
end

function out = push_dispatch(op, varargin)
    switch op
        case 'open'
            p = inputParser;
            addParameter(p, 'run', 1);
            addParameter(p, 'interval', 0.03);
            addParameter(p, 'speed_dps', 1.8);      % 电机角速度(度/秒)：用于估算运动时间
            addParameter(p, 'extra_s', 5.0);        % 每帧驱动后额外等待(秒)
            addParameter(p, 'read_settle_s', 0.05); % 读帧(trigger=0)后短等待(让模型采样)
            addParameter(p, 'ready_file', fullfile(tempdir, 'motor_push_ready.txt'));
            parse(p, varargin{2:end});              % varargin{1}=port
            port = varargin{1};
            server = java.net.ServerSocket(port);
            if exist(p.Results.ready_file, 'file'), delete(p.Results.ready_file); end
            fid = fopen(p.Results.ready_file, 'w'); if fid>0, fprintf(fid,'%d',port); fclose(fid); end
            fprintf('[motorPush] listen %d\n', port);
            out = struct('method','push','server',server,'sock',[],'dos',[], ...
                'ready_file',p.Results.ready_file,'port',port, ...
                'run',p.Results.run,'interval',p.Results.interval, ...
                'speed_dps',p.Results.speed_dps,'extra_s',p.Results.extra_s, ...
                'read_settle_s',p.Results.read_settle_s,'connected',false,'fired_log',{{}});

        case 'accept'
            h = varargin{1};
            p = inputParser;
            addParameter(p, 'timeout_ms', 0);       % 0=无限等待；>0 超时则返回(仍 disconnected)
            parse(p, varargin{2:end});
            if p.Results.timeout_ms > 0
                h.server.setSoTimeout(p.Results.timeout_ms);
            else
                h.server.setSoTimeout(0);
            end
            fprintf('[motorPush] waiting client on %d...\n', h.port);
            try
                sock = h.server.accept();
            catch
                % 超时(无客户端连入)：保持 disconnected，调用方可稍后重试
                h.connected = false;
                fprintf('[motorPush] no client within timeout\n');
                out = h; return;
            end
            h.sock = sock;
            h.dos = java.io.DataOutputStream(sock.getOutputStream());
            h.connected = true;
            fprintf('[motorPush] client %s connected\n', char(sock.getInetAddress().toString()));
            out = h;

        case 'push'
            h = varargin{1};  cp = varargin{2};
            p = inputParser;
            addParameter(p, 'run', h.run);
            addParameter(p, 'interval', h.interval);
            addParameter(p, 'speed_dps', h.speed_dps);
            addParameter(p, 'extra_s', h.extra_s);
            addParameter(p, 'read_settle_s', h.read_settle_s);
            parse(p, varargin{3:end});
            run = double(p.Results.run);
            speed = double(p.Results.speed_dps);
            if ~h.connected
                fprintf('[motorPush] client not connected, skip push\n');
                h.fired_log = {};  out = h; return;
            end
            M = numel(cp);  lg = {};
            try
                for k = 1:M
                    f = cp(k);  f.run = run;      % 服务端运行参数覆盖到每帧
                    tcpJavaWriteFrame(h.dos, tcpEncodeFrame('MOTOR_CMD', f, struct('seq', k)));
                    % 计算本帧后需等待的时间：驱动帧按"最长运动臂时间+额外时间"，读帧只短等待
                    wait_s = p.Results.read_settle_s;
                    trg = [];  ang = [];
                    if isfield(f, 'motors') && ~isempty(f.motors)
                        trg = double([f.motors.trigger]);
                        ang = double([f.motors.angle_deg]);
                    end
                    if ~isempty(trg) && any(trg == 1)
                        move_ang = max(ang(trg == 1));             % 移动电机里最大角度(度)
                        wait_s = move_ang / max(speed, eps) + p.Results.extra_s;
                    end
                    lg{end+1} = sprintf('MOTOR_CMD seq=%d wait=%.2fs', k, wait_s); %#ok<AGROW>
                    if wait_s > 0, pause(wait_s); end
                end
            catch e
                % 客户端断开(reset)：干净关闭该连接并标记 disconnected，下次任务可等重连
                fprintf('[motorPush] write failed (client may have disconnected): %s\n', e.message);
                try, h.sock.close(); catch, end
                h.connected = false;
            end
            h.fired_log = lg;
            out = h;

        case 'byeclose'   % 发 BYE 后关闭（一次性用）
            h = varargin{1};
            if h.connected
                n = numel(h.fired_log);
                tcpJavaWriteFrame(h.dos, tcpEncodeFrame('BYE', struct(), struct('seq', n+1)));
                try, h.sock.close(); catch, end
            end
            out = close_push(h);

        case 'close'
            h = varargin{1};
            out = close_push(h);
    end
end

function h = close_push(h)
    try, if h.connected, h.sock.close(); end; catch, end
    try, h.server.close(); catch, end
    if exist(h.ready_file, 'file'), delete(h.ready_file); end
    fprintf('[motorPush] closed\n');
    h.server = []; h.sock = []; h.dos = []; h.connected = false;
end
