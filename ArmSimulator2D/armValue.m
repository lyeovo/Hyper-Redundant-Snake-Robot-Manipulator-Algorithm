function V = armValue(model, q, v, v_prev)
%armValue 统一标量势 V(q)（全部迭代方法共享的单一事实来源）
%   V = armValue(model, q)
%   V = armValue(model, q, v, v_prev)     % 含加速度平滑项
%
%   V(q) = w_pos·‖p(q)−X_t‖² + w_ang·wrap(θ(q)−θ_t)²
%        + w_obs·Σ_k [−log(g_eff,k)]      % 对数屏障，g_eff = max(d_k−d_safe, g_min)
%        + w_var·Σ_j (q_j−μ_j)²/σ_j²      % 关节先验（可选）
%        + w_acc·‖v−v_prev‖²              % 加速度平滑（可选）
    cfg = model.cfg;
    [~, p_end] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(q, model.DH, cfg.rod_offset_arr);
    e_pos = cfg.X_target - p_end;
    e_ang = wrapAngle(cfg.theta_target - th);
    V = cfg.w_pos * (e_pos * e_pos') + cfg.w_ang * e_ang^2;

    if cfg.w_obs > 0
        [g_all, ~] = obsDistGradAll(model, q);
        for k = 1:length(g_all)
            g_eff = max(g_all(k) - cfg.d_safe, cfg.g_min);
            if g_eff < cfg.barrier_range
                % 对数屏障（激活范围内）：g_eff→0 时 +∞，g_eff→range 时 →0（连续）
                V = V + cfg.w_obs * (log(cfg.barrier_range) - log(g_eff));
                % 侵入深度线性惩罚：进入安全区越深惩罚越大
                % （对 RL/SA 等非梯度方法，仅靠 log 项不足以压过位置项，会停在侵入态）
                if g_all(k) < cfg.d_safe
                    V = V + cfg.w_obs * cfg.barrier_C * (cfg.d_safe - g_all(k));
                end
            end
        end
    end
    if cfg.w_var > 0
        V = V + cfg.w_var * sum(((q - cfg.m_arr).^2) ./ cfg.sigma2_arr);
    end
    if nargin >= 3 && cfg.w_acc > 0
        V = V + cfg.w_acc * norm(v - v_prev)^2;
    end
end
