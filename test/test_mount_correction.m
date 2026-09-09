function test_mount_correction()
%test_mount_correction 验证交叠反向安装(偶数电机方向位取反)的修正
%   1) motorCmdToControlParams：direction 对偶数(j=2,4,6)取反，奇数(1,3,5)保持 (1正/0反，沿 model 正向)。
%   2) mount_sign 默认交替 [+1,-1,+1,-1,+1,-1]。
%   3) 构造 dq，用 mock 的模型(方向位→sign→×mount_sign)推回 model 关节角，应等于原始 dq。
    fprintf('== test_mount_correction ==\n');
    fails = 0;
    addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

    % 构造一个 q_seq：从 0 到已知非零（含正/负），验证方向位
    N = 6;
    q_seq = [zeros(1,N); 0.5, -0.5, 0.4, -0.3, 0.2, -0.1];   % rad
    mc = struct('q_seq', q_seq, 't_seq', [0 1], 'gripper_seq', [0 0]);
    cp = motorCmdToControlParams(mc);
    fprintf('  mount_sign = %s\n', sprintf('%d ', cp(1).mount_sign));
    fails = fails + assertEq(cp(1).mount_sign, [1 -1 1 -1 1 -1], 'mount_sign 交替');
    fails = fails + assertEq(numel(cp(1).motors), N, '6 电机');

    % 取第一个运动档的驱动帧(motors trigger 含 enable)里的 direction
    m0 = cp(1).motors;    % 首帧 k=1: dq=0 -> enable=0, 全占位; 找驱动帧
    % 找 cp 中第一个含 enable=1 的帧
    dq_test = (q_seq(2,:) - q_seq(1,:))*180/pi;   % 第二档相对增量(度)
    dirm = [];
    for f = 1:numel(cp)
        if any([cp(f).motors.enable] == 1)
            dirm = [cp(f).motors.direction];   %#ok<AGROW>
            break;
        end
    end
    fprintf('  dq_deg = %s\n', mat2str(dq_test, 4));
    fprintf('  direction = %s\n', sprintf('%d ', dirm));
    % 期望：direction(j) = (mount_sign(j)*dq_test(j) >= 0)
    expected_dir = double(cp(1).mount_sign .* dq_test >= 0);
    fprintf('  expected_direction = %s\n', sprintf('%d ', expected_dir));
    fails = fails + assertEq(dirm, expected_dir, 'direction 偶数取反/奇数保持');

    % 双向验证：由 direction 重建 model 关节角（用 mock 同款换算）
    %   读相 target(j)=sgn_dir*|dq|；驱动相 joints += mount(j)*target(j)
    dq_recon = zeros(1,N);
    for j = 1:N
        sgn = 1; if dirm(j) == 0, sgn = -1; end
        dq_recon(j) = cp(1).mount_sign(j) * sgn * abs(dq_test(j));
    end
    fprintf('  重建 dq(deg) = %s\n', mat2str(dq_recon,4));
    fails = fails + assertTrue(max(abs(dq_recon - dq_test)) < 1e-9, '方向位+安装符号重建=原始 dq');

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_mount_correction FAILED'); end
end

function f = assertEq(a, b, name)
    if ~isequal(a, b), fprintf('  [FAIL] %s: got %s, expected %s\n', name, mat2str(a), mat2str(b)); f = 1; else, f = 0; end
end
function f = assertTrue(b, name)
    if ~b, fprintf('  [FAIL] %s\n', name); f = 1; else, f = 0; end
end
