function out = cvaePolicyDeploy(mat_file, model, q0, target)
%cvaePolicyDeploy 部署闭环策略：CVAE 快生成 + 安全层（穿障回退 RRT*）
%   out = cvaePolicyDeploy(mat_file, model, q0, target)
%
%   部署语义（方案 §5.7.6）：
%   1. CVAE 潜空间优化生成粗轨迹（毫秒~数百 ms）
%   2. 逐帧碰撞检查（obsDistAll）——穿障则回退 RRT*（可靠性兜底）
%   3. 终点无梯度精修达 tol
%   out : struct('traj', [T×N], 'q_final', [1×N], 'via', 'cvae'|'rrtstar')
    out = cvaePolicyFromFile(mat_file, model, q0, target, 3, 20);
    out.via = 'cvae';
    % 安全层：轨迹全程无碰撞 + 终态达标才采用，否则回退 RRT*（可靠性兜底）
    [~, pe] = planarFK_L(out.q_final, model.DH, model.cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(out.q_final, model.DH, model.cfg.rod_offset_arr);
    err_end = norm(pe - target(1:2)) + 0.1*abs(wrapAngle(target(3) - th));
    if ~isFreeTraj(model, out.traj) || err_end > 0.08   % 直出质量差（pos+0.1·ang>0.08）即回退，防"粗直出"误报收敛
        % 回退 auto 链（RRT*→graph→PRM→SA，多障碍难任务成功率高于单 RRT*）
        info = method_auto(model, q0, target, struct('max_samples', 2500));
        out = struct('traj', info.q_snapshot, 'q_final', info.q_final, 'via', 'rrtstar');
        out.via = info.method_used;
    end
    % 终态严格收敛精修（C 档目标：部署端 pos ≤ 0.01m；无梯度贪心绕开势阱）
    q_fin = out.q_final;
    for r = 1:3
        q_fin = refineRandomGreedy(model, q_fin, target, ...
            struct('layers', 3, 'steps_per_layer', 120, 'goal_eps', 0.008));
        [~, pe] = planarFK_L(q_fin, model.DH, model.cfg.rod_offset_arr);
        if norm(pe - target(1:2)) < 0.008, break; end
    end
    out.q_final = q_fin;
    if size(out.traj, 1) > 1, out.traj(end, :) = q_fin; end   % 轨迹终点与精修一致（回放对齐）
    [~, pe] = planarFK_L(q_fin, model.DH, model.cfg.rod_offset_arr);
    th = getEndEffectorAngle_L(q_fin, model.DH, model.cfg.rod_offset_arr);
    out.dist_end = norm(pe - target(1:2));
    out.ang_err  = abs(wrapAngle(target(3) - th));
end

function ok = isFreeTraj(model, traj)
    ok = true;
    for k = 1:size(traj, 1)
        g = obsDistAll(model, traj(k, :));
        if ~isempty(g) && min(g) < model.cfg.rho0
            ok = false; return;
        end
    end
end
