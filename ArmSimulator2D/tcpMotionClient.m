function log = tcpMotionClient(host, port, motor_cmd, varargin)
%tcpMotionClient MATLAB 规划端 → dSPACE（TCP 服务器）客户端
%   log = tcpMotionClient(host, port, motor_cmd, opts)
%   host      : '127.0.0.1' / '192.168.x.x'
%   port      : 端口号
%   motor_cmd : taskExecute 返回的 .motor_cmd，或 simulateMotion 返回的 info
%               （含 q_snapshot/q_seq、t_seq、gripper_seq、dq_max 等）
%   opts      : .command_id (默认 'CMD-TCP')  .estop @()bool  .max_state 读取上限
%
%   流程：TCP 连接 → HELLO → 下发 CMD(motorCmd) → CMD_ACK → 循环读 STATE/DONE → 关闭
%   帧格式：4 字节大端长度 + JSON（见 tcpEncodeFrame / tcpDecodeFrame）
    p = inputParser;
    addParameter(p, 'command_id', 'CMD-TCP');
    addParameter(p, 'estop', @() false);
    addParameter(p, 'max_state', 200);
    parse(p, varargin{:});
    cmd_id = p.Results.command_id;  estop = p.Results.estop;  max_state = p.Results.max_state;

    t = tcpclient(host, port, 'ConnectTimeout', 5);
    log = {};  seq = 0;
    try
        % 1) 握手
        write(t, tcpEncodeFrame('HELLO', struct('device','matlab','node','planner','version','1.0'), struct('seq',seq+1)));
        seq = seq + 1;
        ack = tcpclientReadFrame(t);
        log{end+1} = sprintf('HELLO_ACK: ok=%d session=%s server=%s', ...
            ack.data.ok, ack.data.session_id, ack.data.server);
        if ack.data.ok == 0, error('tcpMotionClient:handshake', '服务器拒绝握手'); end

        % 2) 下发 motorCmd（载荷规整见 motorCmdPayload）
        payload = motorCmdPayload(motor_cmd);
        write(t, tcpEncodeFrame('CMD', payload, struct('seq',seq+1,'command_id',cmd_id)));
        seq = seq + 1;
        ca = tcpclientReadFrame(t);
        log{end+1} = sprintf('CMD_ACK: ok=%d samples=%s msg=%s', ...
            ca.data.ok, num2str(ca.data.samples), ca.data.message);

        % 3) 执行状态回环：读 STATE / DONE / ERROR
        done = false;  nst = 0;
        while ~done && nst < max_state
            m = tcpclientReadFrame(t);
            if estop()
                write(t, tcpEncodeFrame('PING', struct('note','estop_request'), struct('seq',seq+1)));
                seq = seq + 1;
            end
            switch m.type
                case 'STATE'
                    nst = nst + 1;
                    log{end+1} = sprintf('STATE: status=%s progress=%.2f (j%d...)', ...
                        m.data.status, m.data.progress, numel(m.data.joint_positions));
                case 'DONE'
                    log{end+1} = sprintf('DONE: success=%d error_code=%d', m.data.success, m.data.error_code);
                    done = true;
                case 'PONG'
                    % 心跳
                case 'ERROR'
                    log{end+1} = sprintf('ERROR: code=%d msg=%s', m.data.code, m.data.message);
                    done = true;
                otherwise
                    log{end+1} = sprintf('? 未知类型 %s', m.type);
            end
        end
        % 4) 断开
        try, write(t, tcpEncodeFrame('BYE', struct(), struct('seq',seq+1))); catch, end
        delete(t);
    catch e
        try, delete(t); catch, end
        rethrow(e);
    end
end
