clear; clc; close all;

%% 测试一组关节角
theta = [0.2; -0.3; 0.15; -0.4; 0.25; -0.1]; % 6×1
% theta = pi*rand(6,1) - pi/2; % 随机采样θ∈(-π,π)

%% 运动学正向映射
[phi,Xall,x,y,a,b,S] = kinMap(theta);
disp('=== 累加矩阵 S ==='); disp(S);
disp('=== 各杆绝对倾角 φ ==='); disp(phi);
disp('=== 7个端点横坐标 Xall=[x0,x1~x6] ==='); disp(Xall);
disp('=== 每杆有效区间 a_k, b_k ==='); disp([a,b]);

%% 校验12条几何约束
[C_all, feasible] = constraintCheck(theta);
disp('=== 12条约束数值 [C1+,C1-,C2+,C2-,...,C6+,C6-] ==='); disp(C_all);
disp(['构型是否可行(无杆穿越y=sinx±0.5)：',num2str(feasible)]);

%% 绘制机械臂简图验证
figure; hold on; grid on;
% 上下边界曲线
xx = linspace(-pi,pi,500);
y_up = sin(xx)+0.5;
y_down = sin(xx)-0.5;
plot(xx,y_up,'r--','LineWidth',1.2);
plot(xx,y_down,'b--','LineWidth',1.2);

% 机械臂7个点
Xall_y = [0; y];
plot(Xall, Xall_y, 'ko-','LineWidth',1.5,'MarkerSize',6);
xlabel('X'); ylabel('Y');
title('6臂平面机械臂 + 禁带边界 y=sinx±0.5');
legend('上边界y=sinx+0.5','下边界y=sinx-0.5','机械臂连杆');
xlim([-4,4]); % 聚焦(-π,π)区间
hold off;