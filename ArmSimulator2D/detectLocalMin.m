function [is_min, grad_norm, dist_end] = detectLocalMin(model, q, target, tol_grad)
%detectLocalMin 局部最优检测判据（方案 §5.6）
%   [is_min, grad_norm, dist_end] = detectLocalMin(model, q, target, tol_grad)
%   判据：‖∇V‖ < tol_grad 且 末端误差 > tol_pos ⇒ 困于势阱（局部最优）
%   触发后上层应执行逃逸（多起点 / SA 升温重启 / 升级到 RRT/PRM 采样层）
    cfg = model.cfg;
    if nargin < 4, tol_grad = cfg.localmin_tol_grad; end
    grad = armGradient(model, q);
    grad_norm = norm(grad);
    [~, p_end] = planarFK_L(q, model.DH, cfg.rod_offset_arr);
    dist_end = norm(target(1:2) - p_end);
    is_min = (grad_norm < tol_grad) && (dist_end > cfg.tol_pos);
end
