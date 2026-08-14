function theta_end = getEndEffectorAngle_L(q, DH, rod_offset_arr)
%getEndEffectorAngle_L 末端朝向角（最后一段的方向，rad，[-pi,pi]）
%   修正（相对旧代码）：θ = atan2(M_N − P_{N-1}')（最后一段方向）
%   旧代码取反向 (P_{N-1}' − M_N)，导致全零构型（臂沿 +X）时 θ=π 而非 0
    [p_all, ~] = planarFK_L(q, DH, rod_offset_arr);
    x0 = p_all(end-1,1); y0 = p_all(end-1,2);   % M_N（末端）
    x1 = p_all(end-2,1); y1 = p_all(end-2,2);   % P_{N-1}'
    theta_end = atan2(y0 - y1, x0 - x1);
end
