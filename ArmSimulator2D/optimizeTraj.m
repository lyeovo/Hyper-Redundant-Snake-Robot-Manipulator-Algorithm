function [traj, stats] = optimizeTraj(model, traj, opts)
%optimizeTraj 路径后处理：随机短切（缩短）+ 滑动平滑（去折角），保持端点与无碰撞
%
%   [traj, stats] = optimizeTraj(model, traj, opts)
%
%   输入：
%     model    臂模型（createArmModel 输出）
%     traj     [T×N] 关节路径（RRT* 等采样规划输出）
%     opts     .n_shortcut  随机短切次数（默认 800）
%              .smooth_win  滑动平滑窗口（默认 3；1=不平滑）
%              .rho_clear   期望离障碍净距（默认 model.cfg.rho0）
%              .seed        随机种子（可复现）
%   输出：
%     traj     [T'×N] 优化后路径（端点不变，无碰撞）
%     stats    .n_shortcut 实际短切次数
%              .len_before / .len_after  C-space 路径长度
%
%   说明：
%   - 短切：随机取路径两点，直线连接若无碰撞则移除中间段（OMPL
%     simplifySolution 同款，对采样路径缩短 10-30%）
%   - 平滑：滑动均值去折角，逐点碰撞校验，穿障点回退原始值
%   - 维度无关：路径是 [T×N] 关节序列，碰撞检查走 obsDistAll

    if nargin < 3, opts = struct(); end
    n_shortcut = optget(opts, 'n_shortcut', 800);
    win        = optget(opts, 'smooth_win', 3);
    seed       = optget(opts, 'seed', []);
    if ~isempty(seed), rng(seed); end

    if size(traj, 1) < 3
        stats = struct('n_shortcut', 0, 'len_before', 0, 'len_after', 0);
        return;
    end
    len0 = sum(vecnorm(diff(traj, 1, 1), 2, 2));
    stats.len_before = len0;

    % ---- 1. 随机短切（缩短） ----
    ns = 0;
    for k = 1:n_shortcut
        T = size(traj, 1);
        if T < 3, break; end
        i = randi([1 T-1]);
        j = randi([i+1 T]);
        if j > i+1 && edgeFree(model, traj(i,:), traj(j,:))
            traj = [traj(1:i, :); traj(j:end, :)];
            ns = ns + 1;
        end
    end
    stats.n_shortcut = ns;

    % ---- 2. 滑动平滑（去折角，端点不动，逐点碰撞回退） ----
    if win >= 3
        T = size(traj, 1);
        if T > win
            raw = traj;
            sm = movmean(traj, win, 1);
            for i = 2:T-1
                cand = sm(i, :);
                % 平滑点必须无碰撞且与两端连线无碰撞，否则保留原始点
                if isFreeP(model, cand) && edgeFree(model, traj(i-1,:), cand) ...
                        && edgeFree(model, cand, traj(i+1,:))
                    traj(i, :) = cand;
                else
                    traj(i, :) = raw(i, :);
                end
            end
        end
    end

    stats.len_after = sum(vecnorm(diff(traj, 1, 1), 2, 2));
end

% ---- 局部工具 ----
function ok = isFreeP(model, q)
    g = obsDistAll(model, q);
    ok = isempty(g) || min(g) >= model.cfg.rho0;
end

function ok = edgeFree(model, qa, qb)
    dq = qb - qa;
    n_chk = max(4, ceil(norm(dq) / 0.25));
    ok = true;
    for s = 0:n_chk
        if ~isFreeP(model, qa + (s/n_chk) * dq), ok = false; return; end
    end
end
