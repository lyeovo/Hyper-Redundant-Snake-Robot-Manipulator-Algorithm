function p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr)
%planarFK_SimpleNode 段端节点序列（仅杆端，不含偏移点）
%   p_nodes: (N+1)×2，行1=基座，行k+1=段端M_k（即 p_all(2k)）
    n = length(q);
    p_nodes = zeros(n+1, 2);
    P_curr = [0, 0];
    p_nodes(1,:) = P_curr;
    th_sum = 0;
    for i = 1:n
        th_sum = th_sum + q(i);
        L = DH(i,3);
        off = rod_offset_arr(i);
        Mx = P_curr(1) + L*cos(th_sum);
        My = P_curr(2) + L*sin(th_sum);
        P_curr = [Mx - off*sin(th_sum), My + off*cos(th_sum)];
        p_nodes(i+1,:) = [Mx, My];
    end
end
