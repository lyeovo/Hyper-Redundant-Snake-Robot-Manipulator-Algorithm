function [g_all, dg_all] = obsDistGradAll(model, q)
%obsDistGradAll 汇总全部障碍（圆+矩形）在当前模式下的距离与梯度
%   [g_all, dg_all] = obsDistGradAll(model, q)
%   g_all : K×1，到各障碍边缘的带符号距离（侵入为负）
%   dg_all: K×N，各自对 q 的梯度
%   mode 'seg'  （默认）：检测全部杆段（含偏移小段），最安全
%   mode 'point'：仅电机节点（段端）+ 末端，快速模式（可能漏检穿段）
    cfg = model.cfg;
    N = cfg.N; DH = model.DH; rod = cfg.rod_offset_arr;
    obs = cfg.obstacles;
    n_cir = size(obs.circles, 1);
    n_rect = size(obs.rects, 1);
    g_all = []; dg_all = [];
    if n_cir == 0 && n_rect == 0, return; end
    if strcmp(cfg.obs_mode, 'point')
        p_nodes = planarFK_SimpleNode(q, DH, rod);      % (N+1)×2
        for k = 2:N+1                                    % 电机 M_{k-1} + 末端 M_N
            p = p_nodes(k,:);
            Jp = planarJacPoint_L(q, DH, 2*(k-1), rod);  % M_{k-1} 的 p_all 行号
            for ci = 1:n_cir
                c = obs.circles(ci,:);
                [g, dgp] = pointCircleDistGrad(p, c(1), c(2), c(3));
                g_all  = [g_all;  g];
                dg_all = [dg_all; dgp * Jp];
            end
            for ri = 1:n_rect
                rect = obs.rects(ri,:);
                [g, dgp] = pointRectSignedDist(p, rect);
                g_all  = [g_all;  g];
                dg_all = [dg_all; dgp * Jp];
            end
        end
    else
        [p_all, ~] = planarFK_L(q, DH, rod);
        n_seg = size(p_all,1) - 1;
        for seg = 1:n_seg
            p0 = p_all(seg,:); p1 = p_all(seg+1,:);
            for ci = 1:n_cir
                c = obs.circles(ci,:);
                [g, dg] = segCircleDistGrad(p0, p1, c(1), c(2), c(3), q, DH, seg, rod);
                g_all  = [g_all;  g];
                dg_all = [dg_all; dg];
            end
            for ri = 1:n_rect
                rect = obs.rects(ri,:);
                [g, dg] = segRectDistGrad(q, DH, seg, p0, p1, rect, rod);
                g_all  = [g_all;  g];
                dg_all = [dg_all; dg];
            end
        end
    end
end
