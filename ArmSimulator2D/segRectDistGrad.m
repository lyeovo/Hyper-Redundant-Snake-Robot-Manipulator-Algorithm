function [g, dg] = segRectDistGrad(q, DH, seg_idx, p0, p1, rect, rod_offset_arr)
%segRectDistGrad 机械臂段 [p0,p1] 到旋转矩形的带符号距离与对 q 的梯度
%   g : 外部为正（4 边最短距离），段侵入矩形内为负（采样点最深侵入深度）
%   dg: 1×N；外部取最近边梯度，内部取最深采样点的矩形梯度
%   rect = [cx, cy, th, w, h]
    n = length(q);
    cx = rect(1); cy = rect(2); th = rect(3);
    w = rect(4);  h = rect(5);
    hw = w/2; hh = h/2;
    c = cos(th); s = sin(th);
    % 4 顶点（局部 ±hw, ±hh 旋到全局）
    corners = [cx - hw*c + hh*s, cy - hw*s - hh*c;
               cx + hw*c + hh*s, cy + hw*s - hh*c;
               cx + hw*c - hh*s, cy + hw*s + hh*c;
               cx - hw*c - hh*s, cy - hw*s + hh*c];
    % 外部：4 边最短距离
    g = Inf; dg = zeros(1, n);
    for e = 1:4
        e1 = corners(e,:);
        e2 = corners(mod(e,4)+1,:);
        [de, dge] = segSegDistGrad(q, DH, seg_idx, p0, p1, e1, e2, rod_offset_arr);
        if de < g, g = de; dg = dge; end
    end
    % 内部判定：自适应加密采样（间距 ≤ min(w,h)/2，保证穿越细长矩形必被命中）
    n_samp = max(6, ceil(norm(p1 - p0) / max(0.5 * min(w, h), 1e-9)));
    g_min_ins = Inf;  t_best = 0;  dgpi_best = [0, 0];
    for tii = 0:n_samp
        ti = tii / n_samp;
        pt = (1-ti)*p0 + ti*p1;
        [gi, dgpi] = pointRectSignedDist(pt, rect);
        if gi < 0 && gi < g_min_ins
            g_min_ins = gi;  t_best = ti;  dgpi_best = dgpi;
        end
    end
    if ~isinf(g_min_ins)
        g  = g_min_ins;
        % 采样点雅可比 = 端点雅可比线性插值（避免 round 端点近似误差）
        J0 = planarJacPoint_L(q, DH, seg_idx,   rod_offset_arr);
        J1 = planarJacPoint_L(q, DH, seg_idx+1, rod_offset_arr);
        dg = dgpi_best * ((1-t_best)*J0 + t_best*J1);
    end
end
