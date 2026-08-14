function g_all = obsDistAll(model, q)
%obsDistAll 轻量障碍距离（只算距离，不算梯度——碰撞检查用）
%   与 obsDistGradAll 的 g_all 输出完全一致（遍历顺序相同）
    cfg = model.cfg;
    N = cfg.N; DH = model.DH; rod = cfg.rod_offset_arr;
    obs = cfg.obstacles;
    n_cir = size(obs.circles, 1);
    n_rect = size(obs.rects, 1);
    g_all = [];
    if n_cir == 0 && n_rect == 0, return; end
    if strcmp(cfg.obs_mode, 'point')
        p_nodes = planarFK_SimpleNode(q, DH, rod);
        for k = 2:N+1
            p = p_nodes(k,:);
            for ci = 1:n_cir
                c = obs.circles(ci,:);
                [g, ~] = pointCircleDistGrad(p, c(1), c(2), c(3));
                g_all = [g_all; g]; %#ok<AGROW>
            end
            for ri = 1:n_rect
                rect = obs.rects(ri,:);
                [g, ~] = pointRectSignedDist(p, rect);
                g_all = [g_all; g]; %#ok<AGROW>
            end
        end
    else
        [p_all, ~] = planarFK_L(q, DH, rod);
        n_seg = size(p_all,1) - 1;
        for seg = 1:n_seg
            p0 = p_all(seg,:); p1 = p_all(seg+1,:);
            for ci = 1:n_cir
                c = obs.circles(ci,:);
                [g, ~] = segCircleDistGrad(p0, p1, c(1), c(2), c(3), q, DH, seg, rod);
                g_all = [g_all; g]; %#ok<AGROW>
            end
            for ri = 1:n_rect
                rect = obs.rects(ri,:);
                [g, ~] = segRectDistGrad(q, DH, seg, p0, p1, rect, rod);
                g_all = [g_all; g]; %#ok<AGROW>
            end
        end
    end
end
