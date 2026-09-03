function tcpStateStreamServer(port, traj, varargin)
%tcpStateStreamServer 参考【状态反馈流】服务器：接受一个客户端，按 interval 流式发 STATE 帧
%   tcpStateStreamServer(port, traj, opts)
%   真实链路中电控应照此契约回传真实臂/物体状态：
%     STATE { t, joints[6], gripper:0/1, obj:[x y] }
%   traj   : [K×6] 绝对关节角 rad（沿其播报，模拟真实臂跟随轨迹）
%   opts   : .interval(0.05) .gripper(0) .obj([K×2] 物体位置，可空)
%            .obj_frame('base'|'cam', 默认 base——cam 表示 obj 为相机系相对位姿)
%            .absnet_at([k...]，在这些帧标记 obj_absent=true=视野无物体)
    p = inputParser;
    addParameter(p,'interval', 0.05);
    addParameter(p,'gripper', 0);
    addParameter(p,'obj', []);
    addParameter(p,'obj_frame', 'base');
    addParameter(p,'absent_at', []);
    parse(p,varargin{:});
    interval = p.Results.interval;  gr = p.Results.gripper;  obj = p.Results.obj;
    obj_frame = p.Results.obj_frame;  absent_at = p.Results.absent_at;
    K = size(traj,1);  if isempty(obj), obj = repmat(traj(1,1:2), K,1); end

    server = java.net.ServerSocket(port);
    fprintf('[stateStream] listen %d\n', port);
    sock = server.accept();
    fprintf('[stateStream] client %s connected\n', char(sock.getInetAddress().toString()));
    dis = java.io.DataInputStream(sock.getInputStream());
    dos = java.io.DataOutputStream(sock.getOutputStream());
    for k = 1:K
        o = obj(min(k,end), :);
        absent = ismember(k, absent_at);
        j = struct('t', (k-1)*interval, 'joints', traj(k,:), 'gripper', gr, ...
            'obj', o, 'obj_frame', obj_frame, 'obj_absent', absent);
        tcpJavaWriteFrame(dos, tcpEncodeFrame('STATE', j, struct('seq', k)));
        pause(interval);
    end
    sock.close();  server.close();
    fprintf('[stateStream] done\n');
end
