function b = adaptiveBudget(model, q0, target)
%adaptiveBudget 按场景复杂度自适应采样预算（简单省时、复杂自动加大）
%
%   b = adaptiveBudget(model, q0, target)
%
%   启发式（基于第一性原理：预算只对"样本不足型"失败有效，精修卡死型无效）：
%     - 无障碍 + 短距离 → 1500（简单场景 1500 成功率已饱和，省时）
%     - 障碍 ≤ 2      → 3000（中等）
%     - 障碍 > 2 或多障碍密集 → 6000（复杂，自动加大）
%   结果不超过 model.cfg.rrt_max_samples（用户上限）。
%
%   失败升档（样本不足型窄通道）由调用方处理（见 method_auto：×2 重试一次）。

    cfg = model.cfg;
    n_obs = size(cfg.obstacles.circles,1) + size(cfg.obstacles.rects,1);
    [~, pe] = planarFK_L(q0, model.DH, cfg.rod_offset_arr);
    d = norm(pe - target(1:2));
    if n_obs == 0 && d < 2
        b = 1500;
    elseif n_obs <= 2
        b = 3000;
    else
        b = 6000;
    end
    b = min(b, cfg.rrt_max_samples);
end
