function [dist, dg] = segSegDistGrad(q, DH, seg_idx, p0, p1, l0, l1, rod_offset_arr)
%segSegDistGrad 机械臂段 [p0,p1] 到障碍线段 [l0,l1] 的最短距离与对 q 的梯度
%   基于双参数 (ta,tb) 最近点算法；退化情形（点/平行）均有分支，防除零
    dx_line = l1(1)-l0(1); dy_line = l1(2)-l0(2);
    len2_line = dx_line^2 + dy_line^2;
    dx_seg = p1(1)-p0(1); dy_seg = p1(2)-p0(2);
    len2_seg = dx_seg^2 + dy_seg^2;
    n = length(q);
    if len2_line < 1e-12 && len2_seg < 1e-12
        dist = norm(p0 - l0);
        dg = zeros(1, n); return;
    end
    if len2_line < 1e-12
        % 障碍退化为点 → 点-圆（r=0）
        [dist, dg] = segCircleDistGrad(p0, p1, l0(1), l0(2), 0, q, DH, seg_idx, rod_offset_arr);
        return;
    end
    if len2_seg < 1e-12
        % 机械臂段退化为点 → 点到线
        t = clampVal(((p0(1)-l0(1))*dx_line + (p0(2)-l0(2))*dy_line)/len2_line, 0, 1);
        nearest = l0 + t*[dx_line, dy_line];
        dx = nearest(1)-p0(1); dy = nearest(2)-p0(2);
        dist = sqrt(dx^2+dy^2);
        J0 = planarJacPoint_L(q, DH, seg_idx, rod_offset_arr);
        if dist < 1e-8, dg = zeros(1,n);
        else,            dg = [-dx/dist, -dy/dist]*J0; end
        return;
    end
    dp = l0 - p0;
    ATA = len2_seg;  BTB = len2_line;
    ATB = dx_seg*dx_line + dy_seg*dy_line;
    ATdp = dx_seg*dp(1) + dy_seg*dp(2);
    BTdp = dx_line*dp(1) + dy_line*dp(2);
    det = ATA*BTB - ATB^2;
    if abs(det) < 1e-12
        % 平行：ta 取中点，tb 取垂直投影 tb = (ta·ATB − BTdp)/BTB（由 (l−u)·l_vec = 0 解出）
        ta0 = 0.5;  tb0 = clampVal((ta0*ATB - BTdp)/BTB, 0, 1);
    else
        ta0 = (BTB*ATdp - ATB*BTdp)/det;
        tb0 = (ATB*ATdp - ATA*BTdp)/det;
    end
    in_ta = (ta0 > 0) && (ta0 < 1);         % 无约束解是否在段内
    in_tb = (tb0 > 0) && (tb0 < 1);         % 无约束解是否在线内
    J0 = planarJacPoint_L(q, DH, seg_idx,   rod_offset_arr);
    J1 = planarJacPoint_L(q, DH, seg_idx+1, rod_offset_arr);
    if in_ta && in_tb
        % ---- 双边内点：∂g/∂ta = ∂g/∂tb = 0，解析精确 ----
        u = p0 + ta0*[dx_seg, dy_seg];
        l = l0 + tb0*[dx_line, dy_line];
        dv = l - u;
        dist = sqrt(dv(1)^2 + dv(2)^2);
        if dist < 1e-8
            dg = zeros(1, n); return;
        end
        dhat = dv / dist;                   % 臂段指向障碍线的单位向量
        Jm = (1-ta0)*J0 + ta0*J1;
        dg = -dhat * Jm;                    % ∂g/∂q = −dhat·∂u/∂q
        return;
    end

    % ---- 边界情形（Eberly）：最近点在端点组合上，4 候选取最小 ----
    %   候选 A/B：段端点 p_a 到障碍线 [l0,l1]（tbp 内点或 clamp，梯度均为 −dhat·J_a）
    %   候选 C/D：障碍端点 l_b 到段 [p0,p1]（ta 内点需隐式修正，clamp 用端点雅可比）
    best = Inf;
    s_vec = [dx_seg, dy_seg];
    for a = 0:1
        pa = p0 + a*s_vec;
        Ja = planarJacPoint_L(q, DH, seg_idx + a, rod_offset_arr);
        tbp = clampVal(((pa - l0)*[dx_line; dy_line]) / len2_line, 0, 1);
        lb = l0 + tbp*[dx_line, dy_line];
        dv = lb - pa;
        d = sqrt(dv(1)^2 + dv(2)^2);
        if d < best
            best = d;
            dhat = dv / max(d, 1e-12);
            dg = -dhat * Ja;
        end
    end
    for b = 0:1
        lb = l0 + b*[dx_line, dy_line];
        ta_p = clampVal(((lb - p0)*[dx_seg; dy_seg]) / len2_seg, 0, 1);
        u = p0 + ta_p*s_vec;
        dv = lb - u;
        d = sqrt(dv(1)^2 + dv(2)^2);
        if d < best
            best = d;
            dhat = dv / max(d, 1e-12);
            if ta_p > 0 && ta_p < 1
                % ta 内点：∂g/∂ta ≠ 0，由 (u−lb)·s = 0 隐式求 ∂ta/∂q
                Jm = (1-ta_p)*J0 + ta_p*J1;
                dta = -( s_vec*Jm + (u - lb)*(J1 - J0) ) / len2_seg;
                dg = -dhat * Jm - (dhat * s_vec') * dta;    % (dhat*s_vec') 标量
            elseif ta_p == 0
                dg = -dhat * J0;
            else
                dg = -dhat * J1;
            end
        end
    end
    dist = best;
end
