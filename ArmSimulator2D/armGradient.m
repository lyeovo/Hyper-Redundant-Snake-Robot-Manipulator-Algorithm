function grad = armGradient(model, q, v, v_prev)
%armGradient 统一标量势的解析梯度 ∇V(q)（1×N）
%   grad = armGradient(model, q)
%   grad = armGradient(model, q, v, v_prev)     % 含加速度平滑项
%
%   ∇V = −2·w_pos·e_pos·J − 2·w_ang·e_ang·J_ang
%      − w_obs·Σ_k [ clip(1/g_eff,k, 0, barrier_C) · ∇d_k ]   % 幅值截断，侵入后仍恒排斥
%      + 2·w_var·(q−μ)⊘σ² + 2·w_acc·(v−v_prev)
%
%   注：屏障幅值用 g_eff=max(d−d_safe, g_min) 计算（侵入后饱和为 barrier_C），
%       方向始终用真实 ∇d（推出方向）——保证侵入障碍后仍有最大排斥力（D5）。
    cfg = model.cfg;
    [~, p_end] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
    e_pos = cfg.X_target - p_end;
    e_ang = wrapAngle(cfg.theta_target - th);
    J = planarJac_L(q, model.DH, cfg.rod_offset_arr);
    J_ang = jacEndAngle_L(q, model.DH, cfg.rod_offset_arr);
    grad = -2*cfg.w_pos * (e_pos * J) - 2*cfg.w_ang * e_ang * J_ang;

    if cfg.w_obs > 0
        [g_all, dg_all] = obsDistGradAll(model, q);
        for k = 1:length(g_all)
            g_eff = max(g_all(k) - cfg.d_safe, cfg.g_min);
            if g_eff < cfg.barrier_range
                c = min(1/g_eff, cfg.barrier_C);
                grad = grad - cfg.w_obs * c * dg_all(k,:);
            end
        end
    end
    if cfg.w_var > 0
        grad = grad + 2*cfg.w_var * (q - cfg.m_arr) ./ cfg.sigma2_arr;
    end
    if nargin >= 3 && cfg.w_acc > 0
        grad = grad + 2*cfg.w_acc * (v - v_prev);
    end
end
