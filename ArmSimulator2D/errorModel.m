function q_actual = errorModel(model, q_cmd, q_prev)
%errorModel 电机执行误差注入（方案 §6.1）
%   q_actual = errorModel(model, q_cmd, q_prev)
%   q_actual = q_cmd + N(0, σ_m) + δ_b·sign(Δq) + (κ·round(Δq/κ) − Δq)
%   - σ_m      : 电机转角高斯噪声
%   - δ_b      : 齿轮回差（方向随运动方向）
%   - κ        : 最小精转量（增量量化）
%   model.cfg.error.on = false 时原样返回（默认）
    cfg = model.cfg;
    q_actual = q_cmd;
    if ~cfg.error.on, return; end
    dq = q_cmd - q_prev;
    q_actual = q_cmd + cfg.error.sigma_motor * randn(size(q_cmd));
    q_actual = q_actual + cfg.error.backlash * sign(dq);
    if cfg.error.kappa > 0
        % 增量量化：实际增量 = κ·round(Δq/κ)，等价于在 q_cmd 上叠加量化残差（±κ/2 量级），
        % 而非叠加 κ·round(Δq/κ) 本身（那会引入 O(Δq) 的全量偏移）
        q_actual = q_actual + (cfg.error.kappa * round(dq / cfg.error.kappa) - dq);
    end
end
