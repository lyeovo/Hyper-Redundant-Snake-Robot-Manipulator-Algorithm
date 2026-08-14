function [q_best, pos_err, ang_err] = refineRandomGreedy(model, q0, target, opts)
%refineRandomGreedy 分层随机贪心精修（无梯度，绕过梯度势阱）
%   [q_best, pos_err, ang_err] = refineRandomGreedy(model, q0, target, opts)
%   q0    : 起点（如 RRT 粗达节点 / 多起点种子）
%   target: [x, y, θ]
%   opts  : .layers(3) .steps_per_layer(200) .sigma_base(0.15)
%           .goal_eps(位置容差) .goal_ang(角度容差) .isCancel
%
%   机制：从 q0 出发，逐层缩小扰动幅度做随机游走 + 贪心接受（只记最优）。
%   无梯度 → 不受屏障势阱吸引，可沿自由空间逼近目标（解决动量精修失效场景）。
%   返回最优解的 位置误差 与 角度误差 分解（达标判据由调用方按 goal_eps/goal_ang 分开判定）
    cfg = model.cfg;
    if nargin < 4 || isempty(opts), opts = struct(); end
    layers = of(opts, 'layers', 4);
    steps  = of(opts, 'steps_per_layer', 200);
    sigma0 = of(opts, 'sigma_base', 0.15);
    goal_eps = of(opts, 'goal_eps', cfg.rrt_goal_eps);
    goal_ang = of(opts, 'goal_ang', cfg.rrt_goal_ang);
    isCancel = of(opts, 'isCancel', []);
    N = cfg.N;

    function [d, pa, aa] = errFull(qq)
        [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qq, model.DH, cfg.rod_offset_arr);
        pa = norm(target(1:2) - pe);
        aa = abs(wrapAngle(target(3) - th));
        if pa < goal_eps
            d = aa + 0.15*pa;   % 位置已达标：角度主导精修
        else
            d = pa + 0.35*aa;   % 位置未达标：位置主导（角度辅助）
        end
    end
    function ok = isFree(qq)
        g = obsDistAll(model, qq);
        ok = isempty(g) || min(g) >= cfg.rho0;
    end

    q = q0(:)';
    [best_err, pos_err, ang_err] = errFull(q);
    q_best = q;
    for L = 1:layers
        sigma = sigma0 / (1.5*L);   % 0.10→0.05→0.033→0.025 逐层细化
        for k = 1:steps
            if ~isempty(isCancel) && isCancel(), break; end
            qt = q + sigma * randn(1, N);
            qt = max(cfg.q_min, min(cfg.q_max, qt));
            if ~isFree(qt), continue; end
            [e, pe2, ae2] = errFull(qt);
            if e < best_err
                best_err = e;  q_best = qt;  q = qt;
                pos_err = pe2;  ang_err = ae2;
                if pe2 < goal_eps && ae2 < goal_ang, return; end
            end
        end
    end
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
