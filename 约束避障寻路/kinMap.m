function [phi,Xall,x,y,a,b,S] = kinMap(theta)
% 6自由度平面机械臂矩阵运动学映射 theta -> 所有几何量
% theta: 6×1 相对关节角 [θ1;θ2;θ3;θ4;θ5;θ6]
% Output:
%   phi:6×1 各杆全局绝对倾角 φk = sum_{i=1}^k θi
%   Xall:7×1 全部端点横坐标 [x0;x1;x2;x3;x4;x5;x6]
%   x,y:6×1 杆1~6末端坐标
%   a,b:6×1 每根杆有效区间左右端点 a_k,b_k
%   S:6×6 上三角累加矩阵

%% 1. 构造累加矩阵S
S = tril(ones(6,6)); % 下三角 = 前缀和矩阵
phi = S * theta;    % φ = Sθ

%% 2. 杆位移分量
cPhi = cos(phi);
sPhi = sin(phi);

%% 3. 端点坐标递推（矩阵前缀和）
x0 = -2*pi;
x = x0 * ones(6,1) + S * cPhi;
y = zeros(6,1) + S * sPhi;

%% 4. 全部7个端点横坐标 Xall = [x0, x1,x2,x3,x4,x5,x6]^T
Xall = [x0; x];

%% 5. 逐杆计算 a_k,b_k (k=1~6)
a = zeros(6,1);
b = zeros(6,1);
for k = 1:6
    Xleft  = Xall(k);
    Xright = Xall(k+1);
    Xkmin  = min(Xleft, Xright);
    Xkmax  = max(Xleft, Xright);
    a(k) = max(Xkmin, -pi);
    b(k) = min(Xkmax,  pi);
end

end