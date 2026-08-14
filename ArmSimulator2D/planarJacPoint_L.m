function Jp = planarJacPoint_L(q, DH, idx, rod_offset_arr)
%planarJacPoint_L p_all 第 idx 行节点的位置雅可比（2×N）
%   idx: p_all 行号。偶数 2k = 段端 M_k（节点索引 k+1）；奇数 2k+1 = 偏移点 P_k'
%   旋转中心 = 节点 p_nodes(j)，j=1..该点依赖的关节数（几何近似，off=0 时精确）
    n = length(q);
    p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr);
    [p_all, ~] = planarFK_L(q, DH, rod_offset_arr);
    pt = p_all(idx,:);
    xk = pt(1); yk = pt(2);
    % 该点由前 kc 个关节决定：段端 M_k 与偏移点 P_k' 均由 q_1..q_k 决定
    %   M_k  = P_{k-1}' + L·dir(T_k)，T_k = q_1+..+q_k
    %   P_k' = M_k + off·perp(T_k)
    if mod(idx,2) == 0
        kc = idx/2;              % M_k：旋转中心 = p_nodes(1..k)
    else
        kc = (idx-1)/2;          % P_k'：旋转中心 = p_nodes(1..k)
    end
    kc = min(kc, n);
    Jp = zeros(2, n);
    for i = 1:kc
        xi = p_nodes(i,1); yi = p_nodes(i,2);
        Jp(1,i) = -(yk - yi);
        Jp(2,i) =  xk - xi;
    end
end
