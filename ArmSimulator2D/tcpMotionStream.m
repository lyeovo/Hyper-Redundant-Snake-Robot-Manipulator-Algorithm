function log = tcpMotionStream(host, port, traj, varargin)
%tcpMotionStream 实时向电控下发 8 通道电机驱动数据
%   log = tcpMotionStream(host, port, traj, opts)
%   traj : struct('q',[K×6] 关节角, 'gripper_ori',[K×1]|标量 夹爪朝向, ...
%                 'gripper_gap',[K×1]|标量 0..1 夹爪开合, 't',[1×K] 时间(可空))
%   opts : .rate(Hz，缺省 50；t 为空时用)  .command_id  .estop @()bool
%
%   通道序：id 1..6=关节, 7=GripperOri, 8=GripperGap（见 motorConfig）
%   帧：HELLO→ACK, MOTORS→ACK, DRIVE(t,motors[8])×K（按 dt 节拍, 实时无 ACK 往返）, DONE
    p = inputParser;
    addParameter(p,'rate', 50); addParameter(p,'command_id','CMD-RT'); addParameter(p,'estop',@()false);
    parse(p,varargin{:});
    rate = p.Results.rate;  cmd_id = p.Results.command_id;  estop = p.Results.estop;

    q = traj.q;  K = size(q,1);
    ori = col(traj.gripper_ori, K, 0);       % 夹爪朝向（每采样）
    gap = col(traj.gripper_gap, K, 0);       % 夹爪开合 0..1
    if isfield(traj,'t') && ~isempty(traj.t), ts = traj.t(:).'; else, ts = (0:K-1)/rate; end

    t = tcpclient(host, port, 'ConnectTimeout', 5);
    log = {};  seq = 0;  sent = 0;
    try
        write(t, tcpEncodeFrame('HELLO', struct('device','matlab','node','planner','version','1.0'), struct('seq',seq+1)));
        seq = seq+1;  ack = tcpclientReadFrame(t);
        log{end+1} = sprintf('HELLO_ACK ok=%d', ack.data.ok);

        mcfg = motorConfig();
        write(t, tcpEncodeFrame('MOTORS', mcfg, struct('seq',seq+1,'command_id',cmd_id)));
        seq = seq+1;  ma = tcpclientReadFrame(t);
        log{end+1} = sprintf('MOTORS_ACK ok=%d count=%s', ma.data.ok, num2str(ma.data.count));

        for k = 1:K
            motors = [q(k,:), ori(k), gap(k)];      % 8 通道
            write(t, tcpEncodeFrame('DRIVE', struct('t', ts(k), 'motors', motors), struct('seq',seq+1,'command_id',cmd_id)));
            seq = seq+1;  sent = sent+1;
            if estop(), break; end
            if k < K
                dt = ts(k+1)-ts(k);  if dt <= 0, dt = 1/rate; end
                pause(dt);
            end
        end
        write(t, tcpEncodeFrame('DONE', struct('command_id',cmd_id,'success',true,'error_code',0,'sent',sent), struct('seq',seq+1,'command_id',cmd_id)));
        delete(t);
        log{end+1} = sprintf('streamed %d DRIVE frames', sent);
    catch e
        try, delete(t); catch, end
        rethrow(e);
    end
end

function c = col(v, K, d)
    if isempty(v),  c = d*ones(K,1);
    elseif isscalar(v), c = v*ones(K,1);
    elseif numel(v) == K, c = v(:);
    else, c = d*ones(K,1);
    end
end
