function J = planarJac_L(q, DH, rod_offset_arr)
%planarJac_L 末端位置雅可比（2×N）
%   几何雅可比：旋转中心 = 段端节点 p_nodes(i)（忽略垂直偏移的二阶耦合，off=0 时精确）
    n = length(q);
    [~, p_end] = planarFK_L(q, DH, rod_offset_arr);
    p_nodes = planarFK_SimpleNode(q, DH, rod_offset_arr);
    J = zeros(2, n);
    xn = p_end(1); yn = p_end(2);
    for i = 1:n
        xi = p_nodes(i,1); yi = p_nodes(i,2);
        J(1,i) = -(yn - yi);
        J(2,i) =  xn - xi;
    end
end
