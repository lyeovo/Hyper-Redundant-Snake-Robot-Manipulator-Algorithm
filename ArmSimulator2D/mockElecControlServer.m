function mockElecControlServer(port, opts)
%mockElecControlServer 电控 mock：接收 MOTOR_CMD(26参)帧 → 应用(trigger 驱动) → 回发 STATE
%   本地替代「ControlDesk+Interpreter+dSPACE」：真实场景由 Interpreter 收帧写模型变量、dSPACE 跑闭环。
%   这里用数值累计模拟（trigger 语义）：读帧 trigger=0 把 sign(direction)*angle_deg 存入目标寄存器(不运动)；
%   驱动帧 trigger=1 按寄存器目标累计绝对角（run=0 时保持当前姿态、不累计）。
%   回发 STATE{t, joints[6](rad), gripper, obj[], obj_frame:'base', obj_absent:true}。
%   帧协议与 tcpEncodeFrame/tcpSendControl 一致。
    if nargin < 2, opts = struct(); end
    N = 6;
    server = java.net.ServerSocket(port);
    % 就绪握手：绑定成功后写标志文件，测试端轮询到再到 accept，避免误连(单次accept)。
    ready = fullfile(tempdir, 'elec_mock_ready.txt');
    fid = fopen(ready, 'w'); if fid>0, fprintf(fid, '%d', port); fclose(fid); end
    fprintf('[elec] listen %d\n', port);
    sock = server.accept();
    fprintf('[elec] client %s connected\n', char(sock.getInetAddress().toString()));
    dos = java.io.DataOutputStream(sock.getOutputStream());
    motor_abs = zeros(1, N);  target_deg = zeros(1, N);  gr = 0;  seq = 0;
    mount = ones(1, N);        % 各电机安装方向符号
    while true
        try
            m = javaReadFrame(sock.getInputStream());
        catch
            fprintf('[elec] client closed -> exit\n'); break;
        end
        if strcmp(m.type, 'MOTOR_CMD') && isfield(m, 'data')
            d = m.data;  seq = seq + 1;
            run = 1;  if isfield(d, 'run'), run = double(d.run); end
            if isfield(d, 'mount_sign') && ~isempty(d.mount_sign)
                mm = double(d.mount_sign);  if numel(mm) >= N, mount = mm(1:N); end
            end
            if isfield(d, 'init') && ~isempty(d.init)
                motor_abs = double(d.init);  target_deg = zeros(1, N);   % 首发帧：初始真实电机角(度)作基准
            end
            for j = 1:numel(d.motors)
                mo = d.motors(j);
                sgn = 1;  if double(mo.direction) == 0, sgn = -1; end
                if isfield(mo, 'trigger') && double(mo.trigger) == 1
                    if run ~= 0
                        % dSPACE 对每台电机等处理；direction 已由算法区分取反
                        motor_abs(j) = motor_abs(j) + target_deg(j);
                    end
                else
                    target_deg(j) = sgn*double(mo.angle_deg);           % 读相：只入寄存器
                end
            end
            sv = d.servo;  gr = double(sv.enable);
            % 空间几何角 = 安装符号 .* 电机累计位（机械反装把电机反向转成空间正向）
            joints = mount .* motor_abs;
            st = struct('t', seq*0.1, 'joints', joints*pi/180, 'gripper', gr, ...
                'obj', [], 'obj_frame', 'base', 'obj_absent', true);
            tcpJavaWriteFrame(dos, tcpEncodeFrame('STATE', st, struct('seq', seq)));
        elseif strcmp(m.type, 'BYE')
            fprintf('[elec] BYE -> exit\n'); break;
        end
    end
    sock.close();  server.close();
    rf = fullfile(tempdir,'elec_mock_ready.txt');
    if exist(rf, 'file'), delete(rf); end
    fprintf('[elec] done\n');
end

%% ---- 帧读取（与 tcpMotionServer.m 的 javaReadFrame 同契约：4字节大端长度 + JSON）----
function msg = javaReadFrame(dis)
    len = double(dis.readInt());
    if len < 1 || len > 1e7, error('mockElecControlServer:frame', 'invalid frame length %d', len); end
    mb = uint8(zeros(1, len));
    for i = 1:len, mb(i) = dis.read(); end   % 逐字节读（可靠，避免数组桥接）
    msg = jsondecode(char(mb));
end
