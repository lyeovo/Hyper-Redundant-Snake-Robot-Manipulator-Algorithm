function [g, dg] = segCircleDistGrad(p0, p1, cx, cy, r, q, DH, seg_idx, rod_offset_arr)
%segCircleDistGrad 机械臂段 [p0,p1] 到圆的带符号距离与对 q 的梯度
%   g : 到圆边缘距离（段侵入圆内为负；最近点 = 圆心在段上的投影点）
%   dg: 1×N，对关节角 q 的梯度（几何雅可比链式，off=0 时精确）
%   修复（相对旧代码）：真正减去半径 r；段退化时退化为点-圆，防除零
    dx_seg = p1(1) - p0(1); dy_seg = p1(2) - p0(2);
    len2 = dx_seg^2 + dy_seg^2;
    n = length(q);
    if len2 < 1e-12
        % 段退化为点
        dx = p0(1) - cx; dy = p0(2) - cy;
        dc = sqrt(dx^2 + dy^2);
        g = dc - r;
        J0 = planarJacPoint_L(q, DH, seg_idx, rod_offset_arr);
        if dc < 1e-8, dg = zeros(1,n);
        else,         dg = [dx/dc, dy/dc] * J0; end
        return;
    end
    t = clampVal(((cx - p0(1))*dx_seg + (cy - p0(2))*dy_seg) / len2, 0, 1);
    p_near = p0 + t*[dx_seg, dy_seg];
    dx = p_near(1) - cx; dy = p_near(2) - cy;
    dc = sqrt(dx^2 + dy^2);
    g = dc - r;
    J0 = planarJacPoint_L(q, DH, seg_idx,   rod_offset_arr);
    J1 = planarJacPoint_L(q, DH, seg_idx+1, rod_offset_arr);
    if dc < 1e-8
        dg = zeros(1, n);
    else
        dg = (1-t)*([dx/dc, dy/dc]*J0) + t*([dx/dc, dy/dc]*J1);
    end
end
