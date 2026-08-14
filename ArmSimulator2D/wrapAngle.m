function a = wrapAngle(a)
%wrapAngle 角度差连续化：映射到 [-pi, pi)（替代 Mapping Toolbox 的 wrapToPi）
%   用途：末端角度误差 e_ang = wrapAngle(theta_t - theta)，消除 ±pi 跳变
    a = mod(a + pi, 2*pi) - pi;
end
