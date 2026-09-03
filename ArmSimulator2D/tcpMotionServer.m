function info = tcpMotionServer(port, varargin)
%tcpMotionServer 参考 TCP 服务器（Java ServerSocket），模拟 dSPACE 电控侧
%   info = tcpMotionServer(port, opts)
%   port: 监听端口；opts: .session(默认 'dSPACE-ref')
%
%   语义（dSPACE 侧应实现的契约）：
%     收到 HELLO   → 回 HELLO_ACK
%     收到 CMD     → 回 CMD_ACK，随后按进度发若干 STATE，最后 DONE
%     收到 PING    → 回 PONG（心跳）
%     收到 BYE/断开→ 结束
%   帧格式：[4 字节大端长度][JSON]（与客户端 tcpEncodeFrame 一致）
    p = inputParser; addParameter(p,'session','dSPACE-ref'); parse(p,varargin{:});
    session = p.Results.session;

    server = java.net.ServerSocket(port);
    info = struct('port', port, 'session', session);
    fprintf('[tcpMotionServer] listen on %d\n', port);
    sock = server.accept();
    info.client = char(sock.getInetAddress().toString());
    fprintf('[tcpMotionServer] client connected: %s\n', info.client);
    dis = java.io.DataInputStream(sock.getInputStream());
    dos = java.io.DataOutputStream(sock.getOutputStream());

    running = true;
    while running
        try
            m = javaReadFrame(dis);
        catch
            fprintf('[tcpMotionServer] client closed / EOF -> exit\n');
            break;   % 客户端断开视为正常会话结束
        end
        fprintf('[tcpMotionServer] <- %s seq=%d cid=%s\n', m.type, m.seq, m.command_id);
        switch m.type
            case 'HELLO'
                ack = struct('ok', true, 'session_id', session, 'server', 'dSPACE');
                tcpJavaWriteFrame(dos, tcpEncodeFrame('HELLO_ACK', ack, struct('seq', m.seq)));
            case 'CMD'
                n = size(m.data.q_seq, 1);
                tcpJavaWriteFrame(dos, tcpEncodeFrame('CMD_ACK', ...
                    struct('ok', true, 'message', 'received', 'samples', n), ...
                    struct('seq', m.seq, 'command_id', m.command_id)));
                % 模拟执行：按进度发 STATE，最后 DONE
                for k = 0:4
                    row = max(1, min(n, 1 + round(k/4*(n-1))));
                    st = struct('command_id', m.command_id, 'status', 'EXECUTING', ...
                        'progress', k/4, 'joint_positions', m.data.q_seq(row, :));
                    tcpJavaWriteFrame(dos, tcpEncodeFrame('STATE', st, struct('seq', m.seq, 'command_id', m.command_id)));
                    pause(0.1);
                end
                tcpJavaWriteFrame(dos, tcpEncodeFrame('DONE', ...
                    struct('command_id', m.command_id, 'success', true, 'error_code', 0), ...
                    struct('seq', m.seq, 'command_id', m.command_id)));
            case 'PING'
                tcpJavaWriteFrame(dos, tcpEncodeFrame('PONG', struct('ts', m.data.ts), struct('seq', m.seq)));
            case 'MOTORS'
                tcpJavaWriteFrame(dos, tcpEncodeFrame('MOTORS_ACK', ...
                    struct('ok', true, 'count', m.data.count), struct('seq', m.seq, 'command_id', m.command_id)));
            case 'DRIVE'
                mv = m.data.motors;   % [j1..j6, ori, gap] = 8
                fprintf('[tcpMotionServer] DRIVE t=%.3f motors=[%s]\n', m.data.t, num2str(mv, '%.3f'));
                % 实时：不回 ACK（避免往返），电控侧直接应用到 8 电机
            case 'MOVE'
                % 锁步：电控返回 1(成功)/0(失败)，作为"任务完成"信号
                ok = (numel(m.data.rel_deg) == 6) && (m.data.gripper == 0 || m.data.gripper == 1);
                fprintf('[tcpMotionServer] MOVE k=%d rel=[%s] gripper=%d -> ack %d\n', ...
                    m.data.seq, num2str(m.data.rel_deg, '%.4f'), m.data.gripper, ok);
                tcpJavaWriteFrame(dos, tcpEncodeFrame('MOVE_ACK', ...
                    struct('ok', ok, 'seq', m.data.seq), struct('seq', m.seq, 'command_id', m.command_id)));
            case 'DONE'
                fprintf('[tcpMotionServer] DONE success=%d sent=%s\n', m.data.success, num2str(m.data.sent));
            case 'BYE'
                running = false;
            otherwise
                tcpJavaWriteFrame(dos, tcpEncodeFrame('ERROR', ...
                    struct('code', 1, 'message', ['unknown type ' m.type]), struct('seq', m.seq)));
        end
    end
    sock.close();  server.close();
    fprintf('[tcpMotionServer] done\n');
end

%% ---- Java 帧读写 ----
function msg = javaReadFrame(dis)
    len = double(dis.readInt());
    if len < 1 || len > 1e7, error('tcpMotionServer:frame', '非法帧长 %d', len); end
    mb = uint8(zeros(1, len));
    for i = 1:len, mb(i) = dis.read(); end   % 逐字节读（可靠，避免数组桥接）
    msg = jsondecode(char(mb));
end
