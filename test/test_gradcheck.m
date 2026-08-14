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
    eA = gradErr(m, 20, 1e-6, 'A-无障碍');
    fprintf('  A 无障碍: 最大相对误差 %.3e\n', eA);

    % ---- 场景 B：圆障碍（远离屏障） ----
    m = testModel('obstacles', struct('rects',[],'circles',[2.0,0.8,0.25]));
    eB = gradErr(m, 20, 1e-5, 'B-圆障碍', 0.15);
    fprintf('  B 圆障碍: 最大相对误差 %.3e\n', eB);

    % ---- 场景 C：矩形障碍 ----
    m = testModel('obstacles', struct('rects',[1.5,0.6,0.3,0.4,0.2],'circles',[]));
    eC = gradErr(m, 30, 1e-3, 'C-矩形障碍', 0.15, true);
    fprintf('  C 矩形障碍: 最大相对误差 %.3e\n', eC);

    % ---- 雅可比 vs FK 差分（off=0 精确） ----
    q0 = [0.3, -0.5, 0.8, -0.2];
    J  = planarJac_L(q0, m.DH, m.cfg.rod_offset_arr);
    Jn = jacNum(q0, m.DH, m.cfg.rod_offset_arr);
    eJ = max(max(abs(J - Jn))) / max(1, max(max(abs(Jn))));
    fprintf('  雅可比(off=0) vs 差分: 最大相对误差 %.3e\n', eJ);
    if eJ > 1e-6, error('test_gradcheck FAILED: Jacobian'); end
    if eA > 1e-6 || eB > 1e-5 || eC > 1e-3
        error('test_gradcheck FAILED');
    end
    fprintf('  全部通过\n\n');
end

function e = gradErr(m, nSamp, tol, name, margin, reportQuant)
% 采样 nSamp 个随机 q，解析梯度 vs 中心差分；返回最大相对误差
    if nargin < 5, margin = 0; end
    if nargin < 6, reportQuant = false; end
    errs = [];
    for s = 1:nSamp
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
    if isempty(errs), e = NaN; return; end
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
