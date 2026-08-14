function J_ang = jacEndAngle_L(q, DH, rod_offset_arr)
%jacEndAngle_L 末端朝向角雅可比（1×N）
%   θ = atan2(M_N − P_{N-1}')；∂θ/∂q = dθ/dM_N · (J(M_N) − J(P_{N-1}'))
    n = length(q);
    [p_all, ~] = planarFK_L(q, DH, rod_offset_arr);
    xk = p_all(end-1,1); yk = p_all(end-1,2);   % M_N
    xe = p_all(end-2,1); ye = p_all(end-2,2);   % P_{N-1}'
    dx = xk - xe; dy = yk - ye;                 % 最后一段方向
    L2 = dx^2 + dy^2;
    if L2 < 1e-12
        J_ang = zeros(1, n); return;
    end
    Jk = planarJacPoint_L(q, DH, 2*n,   rod_offset_arr);   % J(M_N)
    Je = planarJacPoint_L(q, DH, 2*n-1, rod_offset_arr);   % J(P_{N-1}')
    dtheta_dp = 1/L2 * [-dy, dx];
    J_ang = dtheta_dp * (Jk - Je);
end
