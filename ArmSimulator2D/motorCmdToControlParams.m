function cp = motorCmdToControlParams(motor_cmd, varargin)
%motorCmdToControlParams motorCmd(q_seq/t_seq/gripper_seq) → 逐帧 26 参控制帧载荷(trigger 驱动)
%   26 参 = 6 电机 × {enable, angle_deg, direction, trigger} = 24 + 1 舵机 × {angle_deg, enable} = 2。
%   trigger 驱动语义（电控侧接管，不再“改角即驱动”）：
%     · trigger = 0  -> 该电机【读入】angle_deg 到目标寄存器（不做运动）。
%     · trigger = 1  -> 电机【驱动】到寄存器中的目标（据此执行运动）。
%   每个运动档拆为【读帧 trigger=0】→【驱动帧 trigger=1】两帧（读帧只入寄存器、驱动帧才累计运动）；
%   电机各自独立：无变化(enable=0)的电机 trigger 恒为 0（不读不驱）。
%   每帧顶层带 run(0/1)：服务端控制的运行参数，缺省 1=运行；0=暂停（客户端读后暂停下发/驱动，不重连）。
%   每帧顶层带 mount_sign(1×6)：各电机安装方向符号（奇数 +1 / 偶数 -1，交叠反向安装，弹簧状）。
%     方向位 direction 按 model 正向(CCW>=0) 结合 mount_sign 翻转：
%       direction = sign(mount_sign(j)*dq(j))   —— 偶数电机方向位取反，实际发给电控即“修正后”的数据。
%   舵机绝对角度由 gripper_seq 映射：0保持/1张开=90°/2闭合=0°（每帧沿用，读/驱两帧相同）。
%
%   选项：
%     'decimate', n : 抽稀。每 n 个快照取 1 个（并保留末点），把一段运动合并成更少的下发帧。
%                     单段关节运动(默认 20 个快照)取 n>=20 即"一次下发该段总目标角"；
%                     n=1(缺省) = 逐快照下发。电机真实运动本身耗时，无需算法逐快照喂。
%
%   返回 cp(:,1..M)，每帧 cp(k)=struct('motors',[1×6],'servo',[1×1],'run',0/1,'mount_sign',[1×6])。
%   首发帧 cp(1) 额外带 init=[初始绝对关节角(度)] 作电控累计基准。
    p = inputParser;  addParameter(p, 'decimate', 1);  parse(p, varargin{:});
    decim = max(1, round(double(p.Results.decimate)));
    q_seq = motor_cmd.q_seq;  K0 = size(q_seq,1);  N = size(q_seq,2);
    g = motor_cmd.gripper_seq;  if isempty(g), g = zeros(1,K0); end
    if decim > 1 && K0 > 2
        idx = 1:decim:K0;  if idx(end) ~= K0, idx = [idx K0]; end
        q_seq = q_seq(idx, :);  g = g(min(idx, numel(g)));
    end
    K = size(q_seq,1);
    ms = optget(motor_cmd, 'mount_sign', []);     % 各电机安装方向符号（缺省 交替 / 全 1）
    if isempty(ms) || numel(ms) ~= N, ms = (-1).^(0:N-1); end
    ms = ms(:).';
    cp = struct('motors',cell(1,0),'servo',cell(1,0),'run',cell(1,0),'init',cell(1,0),'mount_sign',cell(1,0));
    for k = 1:K
        if k == 1, dq = (q_seq(k,:) - q_seq(1,:))*180/pi;      % 首档相对初始位
        else,      dq = (q_seq(k,:) - q_seq(k-1,:))*180/pi;   % 相对上一档增量(度)
        end
        motors = struct('enable',[],'angle_deg',[],'direction',[],'trigger',[]);
        for j = 1:N
            motors(j).enable    = double(abs(dq(j)) > 1e-9);    % 无变化则该电机不动作
            motors(j).angle_deg = abs(dq(j));                   % 度
            % 方向位修正：model 正向(>=0=CCW) 结合安装方向符号 mount_sign。
            %   奇数(+1) -> direction = (dq>=0)；偶数(-1) -> direction 反转(电机反向安装)。
            motors(j).direction = double(ms(j)*dq(j) >= 0);
            motors(j).trigger   = 0;                            % 读帧：恒 0
        end
        gk = g(min(k,end));
        sv = struct('angle_deg', 0.0, 'enable', 1.0);
        if gk == 1,   sv.angle_deg = 90;          % 张开
        elseif gk == 2, sv.angle_deg = 0;          % 闭合
        end
        if any([motors.enable] == 1)
            % 运动档：读帧(trigger=0) → 驱动帧(trigger=1)
            cp(end+1) = struct('motors', motors, 'servo', sv, 'run', 1, 'init', [], 'mount_sign', ms); %#ok<AGROW>
            drv = motors;  for j = 1:N, drv(j).trigger = drv(j).enable; end
            cp(end+1) = struct('motors', drv, 'servo', sv, 'run', 1, 'init', [], 'mount_sign', ms); %#ok<AGROW>
        else
            % 无运动档：单帧(全 enable=0、trigger=0)，仅占位不读不驱
            cp(end+1) = struct('motors', motors, 'servo', sv, 'run', 1, 'init', [], 'mount_sign', ms); %#ok<AGROW>
        end
    end
    if isempty(cp)   % 空轨迹兜底：给单帧空指令
        motors = struct('enable',zeros(1,N),'angle_deg',zeros(1,N),'direction',zeros(1,N),'trigger',zeros(1,N));
        cp = struct('motors', motors, 'servo', struct('angle_deg',0.0,'enable',1.0), 'run', 1, 'init', [], 'mount_sign', ms);
    end
    if ~isempty(q_seq)
        % 首发帧初始【真实电机角度】(度)：它经机械反装后空间=仿真模型初始关节角 q_model(1)。
        %   即 space_init = mount_sign .* init = q_model(1)  →  init = ms .* q_model(1)。
        % 电机后来按相对步进累积(angle_deg 增量)；direction 已对偶数取反(反向装→空间正向)。
        cp(1).init = ms .* q_seq(1,:)*180/pi;
    end
end
