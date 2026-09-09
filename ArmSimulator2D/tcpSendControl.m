function out = tcpSendControl(host, port, cp, varargin)
%tcpSendControl 发送 26 参 MOTOR_CMD 帧（逐帧：读帧 trigger=0 → 驱动帧 trigger=1），锁步读 STATE
%   out = tcpSendControl(host, port, cp, opts)
%   cp : motorCmdToControlParams 输出（每帧一个 struct('motors',.., 'servo',.., 'run',..)）
%   opts: .onState @(st) 每帧回调（可选）；.estop @()bool；.run 服务端运行参数 0/1(缺省 1)
%   返回 out：.q_actual([M×6] rad 电控累计绝对角) .q_actual_final([1×6] rad)
    p = inputParser; addParameter(p,'onState', @(s) []); addParameter(p,'estop', @() false);
    addParameter(p,'run', 1); parse(p,varargin{:});
    onState = p.Results.onState;  estop = p.Results.estop;  run = double(p.Results.run);
    M = numel(cp);  N = numel(cp(1).motors);
    t = tcpclient(host, port, 'ConnectTimeout', 5);
    t.Timeout = 5;   % 读 STATE 超时（防无限等待）；超时抛出→runTaskLoop 记"下发失败"
    q_act = zeros(M, N);  seq = 0;
    try
        for k = 1:M
            f = cp(k);  f.run = run;           % 服务端运行参数覆盖到每帧（0=暂停下发驱动）
            write(t, tcpEncodeFrame('MOTOR_CMD', f, struct('seq', k)));
            st = tcpclientReadFrame(t);
            if isfield(st,'data') && isfield(st.data,'joints')
                q_act(k, :) = double(st.data.joints);  end
            if isfield(st,'data'), onState(st.data); end
            if estop(), break; end
            seq = seq + 1;
        end
        delete(t);
        out = struct('q_actual', q_act, 'q_actual_final', q_act(max(1,size(q_act,1)),:));
    catch e
        try, delete(t); catch, end
        rethrow(e);
    end
end
