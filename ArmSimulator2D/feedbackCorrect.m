function dq = feedbackCorrect(model, q, X_measured, X_target, Kp)
%feedbackCorrect 末端实测位姿 → 增量修正（雅可比阻尼最小二乘，方案 §6.2.3）
%   dq = feedbackCorrect(model, q, X_measured, X_target, Kp)
%   X_measured: 实测末端位置 [x,y]（视觉/编码器反馈）
%   X_target  : 期望末端位置 [x,y]
%   Kp        : 增益（默认 0.5，防过冲）
%
%   用途：闭环——真实电机执行若干步后，用实测末端位姿补偿累积误差。
%   q 更新：q = q + dq'，再继续运动。
    if nargin < 5 || isempty(Kp), Kp = 0.5; end
    cfg = model.cfg;
    J = planarJac_L(q, model.DH, cfg.rod_offset_arr);
    e = X_target(:) - X_measured(:);
    JJT = J * J';
    % 相对阻尼（防病态 JJT 放大，但不过度压制修正）：λ 取 JJT 迹的 2%
    lam = 0.02 * max(1, trace(JJT) / 2);
    dq = (J' / (JJT + lam * eye(2))) * e * Kp;
    dq = dq';
    % 步长钳制（对应电机单步最大增量）
    nrm = norm(dq);
    if nrm > cfg.dq_max, dq = dq * (cfg.dq_max / nrm); end
end
