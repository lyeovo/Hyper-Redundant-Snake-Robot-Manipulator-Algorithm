function [dist, ang, q_final, q_hist] = rlRollout(model, q0, target, theta, lam, max_roll)
%rlRollout RL 策略 rollout（动量梯度 + λ·π_θ(s) residual 集成）
%   [dist, ang, q_final, q_hist] = rlRollout(model, q0, target, theta, lam, max_roll)
%   theta   : 策略参数 [N×D]，D = 2(末端位置误差) + 1(末端角) + 1(最近障碍距离) + N(q 归一化)
%   lam     : residual 权重（θ=0 时退化为纯动量梯度，保证不劣于梯度基线）
%   max_roll: rollout 最大步数
%
%   供 method_rl（推理/任务内训练）与 trainRLPolicy（离线任务族训练）共用。
    cfg = model.cfg;
    N = cfg.N;
    D = 2 + 1 + 1 + N;

    function f = feat(qq)
        % 归一化状态特征
        [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qq, model.DH, cfg.rod_offset_arr);
        e = target(1:2) - pe;
        ea = wrapAngle(target(3) - th);
        g = obsDistAll(model, qq);
        min_g = 1.0; if ~isempty(g), min_g = min(g); end
        f = [e / max(1, norm(e)+1e-6), ea/pi, ...
             min(min_g, 1.0), (qq - cfg.q_min) ./ max(cfg.q_max - cfg.q_min, 1e-9)];
    end

    q = q0(:)';
    q_hist = zeros(0, N);
    for k = 1:max_roll
        [~, pe] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
        dist = norm(target(1:2) - pe);
        ang = abs(wrapAngle(target(3) - th));
        if dist < cfg.tol_pos && ang < cfg.tol_ang, break; end
        grad = armGradient(model, q);
        a = theta * feat(q)';               % π_θ(s)：N×1
        dq = -grad' * 0.05 + lam * a;       % residual 集成
        over = max(abs(dq)) / cfg.dq_max;
        if over > 1, dq = dq / over; end
        q = q + dq';
        q = max(cfg.q_min, min(cfg.q_max, q));
        q_hist(end+1, :) = q; %#ok<AGROW>
    end
    q_final = q;
end
