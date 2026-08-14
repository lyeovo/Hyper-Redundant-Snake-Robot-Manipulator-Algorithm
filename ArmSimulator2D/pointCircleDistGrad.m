function [g, dg_p] = pointCircleDistGrad(p, cx, cy, r)
%pointCircleDistGrad 点到圆的带符号距离（到边缘，侵入为负）与对 p 的梯度
%   g    = |p-c| - r
%   dg_p = (p-c)/|p-c|（1×2；|p-c|≈0 时取 0）
    dx = p(1) - cx; dy = p(2) - cy;
    dc = sqrt(dx^2 + dy^2);
    g = dc - r;
    if dc < 1e-8
        dg_p = [0, 0];
    else
        dg_p = [dx/dc, dy/dc];
    end
end
