function kf = kalmanTargetUpdate(kf, z)
%kalmanTargetUpdate Kalman 观测更新（z = [x, y, yaw]，首帧直接赋值，yaw 自动 wrap）
    H = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0];
    z = z(:);
    if ~kf.init
        kf.x(1:3) = z;
        kf.init = true;
        return;
    end
    y = z - H * kf.x;
    y(3) = wrapAngle(y(3));
    S = H * kf.P * H' + kf.R;
    K = kf.P * H' / S;
    kf.x = kf.x + K * y;
    kf.x(3) = wrapAngle(kf.x(3));
    kf.P = (eye(5) - K * H) * kf.P;
end

function a = wrapAngle(a)
    a = mod(a + pi, 2*pi) - pi;
end
