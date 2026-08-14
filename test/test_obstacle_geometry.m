function test_obstacle_geometry()
%test_obstacle_geometry 障碍原语对照手算解析解
%   圆：点-圆、段-圆（外部/侵入/退化）
%   矩形：点-矩形（外部角/外部边/内部/旋转），段-矩形
    fprintf('== test_obstacle_geometry ==\n');
    fails = 0;

    % ---- 点-圆 ----
    [g, dgp] = pointCircleDistGrad([0,0], 5, 0, 2);
    fails = fails + assertNear(g, 3, '点-圆外部 g');
    fails = fails + assertNear(norm(dgp), 1, '点-圆梯度模');
    [g2, ~] = pointCircleDistGrad([4.5,0], 5, 0, 2);
    fails = fails + assertNear(g2, -1.5, '点-圆内部 g');

    % ---- 段-圆：段穿过圆（侵入） ----
    q = zeros(1,4); DH = testDH(4);
    [g3, ~] = segCircleDistGrad([0,0], [10,0], 5, 0, 2, q, DH, 1, zeros(1,4));
    fails = fails + assertNear(g3, -2, '段-圆穿过 g=-2');
    % 外部
    [g4, ~] = segCircleDistGrad([0,0], [10,10], 5, 0, 2, q, DH, 1, zeros(1,4));
    % 最近点 t=0.25 → (2.5,2.5)，dc=sqrt(12.5)=3.5355 → g=1.5355
    fails = fails + assertNear(g4, sqrt(12.5)-2, '段-圆外部 g');
    % 段退化
    [g5, ~] = segCircleDistGrad([3,0], [3,0], 5, 0, 2, q, DH, 1, zeros(1,4));
    fails = fails + assertNear(g5, 0, '段-圆退化 g=0');

    % ---- 点-矩形 [0,0,0,4,2]（w=4,h=2） ----
    rect = [0,0,0,4,2];
    [g6, ~] = pointRectSignedDist([3,1], rect);
    fails = fails + assertNear(g6, 1, '矩形外部角 g=1');       % 到角点(2,1)
    [g7, ~] = pointRectSignedDist([3,0], rect);
    fails = fails + assertNear(g7, 1, '矩形外部边 g=1');       % 到边 x=2
    [g8, ~] = pointRectSignedDist([0,0], rect);
    fails = fails + assertNear(g8, -1, '矩形内部 g=-1');       % 最近边 y=1
    [g9, dg9] = pointRectSignedDist([0.5,0.8], rect);
    fails = fails + assertNear(g9, -0.2, '矩形内部 g=-0.2');   % 最近边 y=1
    fails = fails + assertNear(dg9(2), 1, '矩形内部梯度 y 分量'); % 指向 +y（推出）
    % 旋转矩形 [0,0,pi/2,4,2] = 等价 w=2,h=4
    [g10, ~] = pointRectSignedDist([0,3], [0,0,pi/2,4,2]);
    fails = fails + assertNear(g10, 1, '旋转矩形外部边 g=1');

    % ---- 段-矩形 ----
    q4 = zeros(1,4); DH4 = testDH(4);
    % 段 (0,0)→(0,5) 与矩形 [2,2,0,2,2]（x∈[1,3],y∈[1,3]）：最近距离 = 1（到边 x=1 或 y=1）
    [g11, ~] = segRectDistGrad(q4, DH4, 1, [0,0], [0,5], [2,2,0,2,2], zeros(1,4));
    fails = fails + assertNear(g11, 1, '段-矩形外部 g=1');
    % 段穿过矩形：(-1,2)→(5,2) 穿过 x∈[1,3] → 侵入
    [g12, ~] = segRectDistGrad(q4, DH4, 1, [-1,2], [5,2], [2,2,0,2,2], zeros(1,4));
    fails = fails + assertNear(g12, -1, '段-矩形穿过 g=-1');  % 最深：段中点(2,2)=矩形中心，深度1

    fprintf('  结果: %d 项断言失败\n\n', fails);
    if fails > 0, error('test_obstacle_geometry FAILED'); end
end

function DH = testDH(N)
    DH = zeros(N,4); DH(:,3) = 1.0;
end

function f = assertNear(actual, expected, name, tol)
    if nargin < 4, tol = 1e-9; end
    if abs(actual - expected) > tol
        fprintf('  [FAIL] %s: got %.6f, expected %.6f\n', name, actual, expected);
        f = 1;
    else
        f = 0;
    end
end
