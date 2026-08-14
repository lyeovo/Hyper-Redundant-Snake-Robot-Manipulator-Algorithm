function rep = assessRobustness(model, q, target, n_mc)
%assessRobustness 蒙特卡洛鲁棒性评估（方案 §6.3）
%   rep = assessRobustness(model, q, target, n_mc)
%   q    : 待评估关节角（如 simulateMotion 的 q_final）
%   target: [x, y, θ]
%   n_mc : 采样数（默认 1000）
%
%   返回 rep：
%     .pos_err  [mean, p95, max] 末端位置误差分布（m）
%     .ang_err  [mean, p95, max] 末端角度误差分布（rad）
%     .collision_prob             碰撞概率 P(d<rho0)
%     .worst_q                    最坏样本（末端误差最大者）
%     .worst_err                  最坏末端误差
    cfg = model.cfg;
    if nargin < 4 || isempty(n_mc), n_mc = 1000; end
    if ~cfg.error.on
        % 误差模型关闭：评估"标定/数值残差"意义有限，直接报告单次结果
        [pe, th] = fkErr(model, q, target);
        rep = struct('pos_err', [pe pe pe], 'ang_err', [th th th], ...
            'collision_prob', 0, 'worst_q', q, 'worst_err', pe);
        return;
    end
    pos = zeros(1, n_mc);  ang = zeros(1, n_mc);  col = 0;
    q_prev = q;
    for k = 1:n_mc
        qa = errorModel(model, q, q_prev);
        q_prev = qa;
        [pe, th] = fkErr(model, qa, target);
        pos(k) = pe;  ang(k) = th;
        g = obsDistAll(model, qa);
        if ~isempty(g) && min(g) < cfg.rho0, col = col + 1; end
    end
    [worst_err, wi] = max(pos);
    rep = struct();
    rep.pos_err = [mean(pos), prctile(pos,95), max(pos)];
    rep.ang_err = [mean(ang), prctile(ang,95), max(ang)];
    rep.collision_prob = col / n_mc;
    rep.worst_q = [];
    rep.worst_err = worst_err;
end

function [pe, th] = fkErr(model, q, target)
    [~, p_end] = planarFK_L(q, model.DH, model.cfg.rod_offset_arr);
    theta = getEndEffectorAngle_L(q, model.DH, model.cfg.rod_offset_arr);
    pe = norm(target(1:2) - p_end);
    th = abs(wrapAngle(target(3) - theta));
end
