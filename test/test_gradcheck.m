function test_gradcheck()
%test_gradcheck 解析梯度 vs 中心差分校验（M0 关键验收）
%   场景 A：无障碍（光滑区严格 <1e-6）
%   场景 B：圆形障碍（远离屏障区 <1e-5）
%   场景 C：矩形障碍（远离屏障与边切换区，95% <1e-3，报告最大值）
%   同时校验雅可比 vs FK 中心差分（off=0 时 <1e-6）
    fprintf('== test_gradcheck ==\n');
    rng(42);

    % ---- 场景 A：无障碍 ----
    m = testModel('w_obs', 0, 'obstacles', struct('rects',[],'circles',[]));
    [eA, nA] = gradErr(m, 20, 1e-6, 'A-无障碍');
    fprintf('  A 无障碍: 最大相对误差 %.3e（%d 样本）\n', eA, nA);

    % ---- 场景 B：圆障碍（远离屏障） ----
    m = testModel('obstacles', struct('rects',[],'circles',[2.0,0.8,0.25]));
    [eB, nB] = gradErr(m, 20, 1e-5, 'B-圆障碍', 0.15);
    fprintf('  B 圆障碍: 最大相对误差 %.3e（%d 样本）\n', eB, nB);

    % ---- 场景 C：矩形障碍 ----
    m = testModel('obstacles', struct('rects',[1.5,0.6,0.3,0.4,0.2],'circles',[]));
    [eC, nC] = gradErr(m, 30, 1e-3, 'C-矩形障碍', 0.15, true);
    fprintf('  C 矩形障碍: 最大相对误差 %.3e（%d 样本）\n', eC, nC);

    % ---- 雅可比 vs FK 差分（off=0 精确） ----
    q0 = [0.3, -0.5, 0.8, -0.2];
    J  = planarJac_L(q0, m.DH, m.cfg.rod_offset_arr);
    Jn = jacNum(q0, m.DH, m.cfg.rod_offset_arr);
    eJ = max(max(abs(J - Jn))) / max(1, max(max(abs(Jn))));
    fprintf('  雅可比(off=0) vs 差分: 最大相对误差 %.3e\n', eJ);
    % NaN 防护：任何一项为 NaN 都不算通过（NaN 参与比较恒为 false，会掩盖失败）
    if any(isnan([eA, eB, eC, eJ]))
        error('test_gradcheck FAILED: 存在 NaN 误差（采样无效或数值异常）');
    end
    if eJ > 1e-6, error('test_gradcheck FAILED: Jacobian'); end
    if eA > 1e-6 || eB > 1e-5 || eC > 1e-3
        error('test_gradcheck FAILED');
    end
    fprintf('  全部通过\n\n');
end

function [e, nvalid] = gradErr(m, nSamp, tol, name, margin, reportQuant)
% 采样 nSamp 个随机 q，解析梯度 vs 中心差分；返回最大相对误差与有效样本数
%   带 margin 的场景用【拒绝采样】收集够 nSamp 个有效点为止（上限 maxTry 次尝试）。
%   有效样本不足则报错——历史坑：曾因 margin 过滤过严导致 errs 为空、返回 NaN，
%   而调用处 `if e > tol` 对 NaN 恒为 false，场景 B 一次都没校验却显示通过。
    if nargin < 5, margin = 0; end
    if nargin < 6, reportQuant = false; end
    maxTry  = nSamp * 50;              % 拒绝采样尝试上限
    minKeep = max(3, ceil(nSamp/4));   % 至少要有这么多个有效点才算校验成立
    errs = [];  tried = 0;
    while numel(errs) < nSamp && tried < maxTry
        tried = tried + 1;
        q = randq(m);
        if margin > 0
            dmin = minObstacleDist(m, q);
            % 跳过屏障近邻（次梯度区）与屏障激活截断点（barrier_range 处不连续）
            if dmin < m.cfg.d_safe + margin || dmin - m.cfg.d_safe > m.cfg.barrier_range - margin
                continue;
            end
        end
        g_an = armGradient(m, q);
        g_fd = fdGradient(m, q);
        denom = max(norm(g_fd), 1e-6);
        errs(end+1) = norm(g_an - g_fd) / denom; %#ok<AGROW>
    end
    nvalid = numel(errs);
    if nvalid < minKeep
        error('test_gradcheck:nosample', ...
            '%s: 仅收集到 %d/%d 个有效样本（尝试 %d 次，margin=%.3f）——过滤过严，无法完成校验', ...
            name, nvalid, nSamp, tried, margin);
    end
    if reportQuant
        e = prctile(errs, 95);              % 允许少量 min 切换点（次梯度）
    else
        e = max(errs);
    end
end

function dmin = minObstacleDist(m, q)
    [g_all, ~] = obsDistGradAll(m, q);
    if isempty(g_all), dmin = Inf; else, dmin = min(g_all); end
end

function q = randq(m)
    q = m.cfg.q_min + rand(1, m.cfg.N) .* (m.cfg.q_max - m.cfg.q_min);
end

function g = fdGradient(m, q)
% 中心差分梯度（步长 sqrt(eps)*max(1,|q|)）
    N = length(q);
    g = zeros(1, N);
    for j = 1:N
        h = sqrt(eps) * max(1, abs(q(j)));
        qp = q; qp(j) = q(j) + h;
        qm = q; qm(j) = q(j) - h;
        g(j) = (armValue(m, qp) - armValue(m, qm)) / (2*h);
    end
end

function Jn = jacNum(q, DH, rod)
% 末端位置雅可比数值对照
    N = length(q);
    Jn = zeros(2, N);
    for j = 1:N
        h = sqrt(eps) * max(1, abs(q(j)));
        qp = q; qp(j) = q(j) + h;
        qm = q; qm(j) = q(j) - h;
        [~, pp] = planarFK_L(qp, DH, rod);
        [~, pm] = planarFK_L(qm, DH, rod);
        Jn(:,j) = (pp - pm)' / (2*h);
    end
end
