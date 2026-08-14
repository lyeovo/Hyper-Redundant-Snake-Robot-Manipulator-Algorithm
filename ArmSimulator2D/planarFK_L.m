function [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr)
%planarFK_L 前向运动学（段端 + 垂直偏移点交替序列）
%   [p_all, p_end] = planarFK_L(q, DH, rod_offset_arr)
%   p_all: (2N+1)×2 节点序列：行1=基座；段k: 行2k=段端M_k, 行2k+1=偏移点P_k'
%   p_end: 末端段端 M_N 坐标 (1×2)
%   DH(i,3) = 第i段杆长；rod_offset_arr(i) = 第i段末端垂直偏移（可为0）
    n = length(q);
    p_all = zeros(2*n+1, 2);
    P_curr = [0, 0];
    p_all(1,:) = P_curr;
    idx = 2;
    M = [0, 0];
    th_sum = 0;
    for i = 1:n
        th_sum = th_sum + q(i);
        L = DH(i,3);
        off = rod_offset_arr(i);
        Mx = P_curr(1) + L*cos(th_sum);
        My = P_curr(2) + L*sin(th_sum);
        M = [Mx, My];
        p_all(idx,:) = M;
        idx = idx + 1;
        P_next = [Mx - off*sin(th_sum), My + off*cos(th_sum)];
        p_all(idx,:) = P_next;
        idx = idx + 1;
        P_curr = P_next;
    end
    p_end = M;
end
