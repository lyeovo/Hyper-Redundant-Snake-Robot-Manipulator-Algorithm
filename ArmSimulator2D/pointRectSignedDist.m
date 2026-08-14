function [g, dg_p] = pointRectSignedDist(p, rect)
%pointRectSignedDist 点到旋转矩形的带符号距离与对 p 的梯度
%   rect = [cx, cy, th, w, h]：中心 (cx,cy)，绕垂直轴旋转 th（rad），宽 w 高 h
%   g    : 外部为正（到最近边/角），内部为负（到最近边的负距离），边上为 0
%   dg_p : 1×2，g 对 p 的梯度（内部指向最近边外侧 = 推出方向）
    cx = rect(1); cy = rect(2); th = rect(3);
    w = rect(4);  h = rect(5);
    c = cos(th); s = sin(th);
    R  = [c, -s; s, c];        % 全局 → 局部
    Rt = [c, s; -s, c];        % 局部 → 全局
    pl = R * (p(:) - [cx; cy]);
    a = abs(pl(1)); b = abs(pl(2));
    hw = w/2; hh = h/2;
    dx = a - hw; dy = b - hh;
    if dx > 0 && dy > 0
        % 外部角点扇形区
        g = norm([dx, dy]);
        ga = dx/g; gb = dy/g;
    elseif dx > 0
        g = dx;  ga = 1;  gb = 0;
    elseif dy > 0
        g = dy;  ga = 0;  gb = 1;
    else
        % 内部：最近边 = max(dx, dy)（dx,dy ≤ 0，越大越近）
        if dx >= dy, g = dx; ga = 1; gb = 0;
        else,        g = dy; ga = 0; gb = 1; end
    end
    dg_l = [ga * sign(pl(1)); gb * sign(pl(2))];
    dg_p = (Rt * dg_l)';
end
