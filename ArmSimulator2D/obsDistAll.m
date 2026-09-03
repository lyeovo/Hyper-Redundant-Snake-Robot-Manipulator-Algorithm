function g_all = obsDistAll(model, q)
%obsDistAll 障碍距离（只返回距离，不返回梯度——碰撞检查用）
%   g_all = obsDistAll(model, q)
%   g_all : K×1，到各障碍边缘的带符号距离（侵入为负），遍历顺序与 obsDistGradAll 一致
%
%   实现说明：本函数原先是 obsDistGradAll 的【逐行复制后删掉梯度部分】（43 行），
%   两处逻辑一旦走偏就会造成"价值函数认为安全、碰撞检查认为侵入"的不一致。
%   现改为薄封装，距离计算只有一份实现。
%
%   性能：实测（6 关节 / 2 圆 + 1 矩形 / 20000 次）薄封装与原实现耗时相同（1.00x）。
%   原因是底层 segCircleDistGrad / segRectDistGrad 无论调用方是否需要都会算出梯度，
%   原实现只是用 [g,~] 丢弃了结果——所谓"只算距离"的轻量性从未真正生效。
    [g_all, ~] = obsDistGradAll(model, q);
end
