function res = evaluatePolicy(policy, opts)
%evaluatePolicy 策略评估框架（维度无关：任意策略函数 → 统一指标）
%   res = evaluatePolicy(policy, opts)
%
%   policy : 策略函数句柄 @(model, q0, target) → struct('traj',[T×D], 'q_final',[1×D])
%            （traj 可为空→用 q_final 单点；q_final 必填）
%            特殊值 'rrtstar'：调用 method_rrtstar 作基线（默认评估基线）
%   opts   : .n_test(默认 30) .N .L_seg .seed
%            .ref_max_samples  基线 RRT* 预算（默认 1500）
%            .goal_pos(默认 0.1) .goal_ang(默认 0.2)  成功率判据
%            .verbose(默认 true)
%
%   返回 res（统一指标，3D 同样适用）：
%     .success_rate   终态误差达标比例
%     .collision_rate 全程侵入障碍比例（应为 0）
%     .mean_end_pos / .mean_end_ang
%     .mean_gap       最优性差距 = mean((策略代价−基线代价)/基线代价)
%     .mean_gen_s     策略生成平均耗时（ms）
%     .per_case       [n_test×1] 逐例结果（诊断用）
%
%   用法：
%     res = evaluatePolicy('rrtstar');                        % 基线自评
%     res = evaluatePolicy(@(m,q0,t) myPolicy(m,q0,t), ...);  % 自定义策略
    if nargin < 2 || isempty(opts), opts = struct(); end
    n_test   = of(opts, 'n_test', 30);
    N = of(opts, 'N', 4);
    L = of(opts, 'L_seg', 1.04393);
    seed     = of(opts, 'seed', []);
    ref_ms   = of(opts, 'ref_max_samples', 1500);
    ref_opt  = of(opts, 'ref_optimize', true);   % 参考侧轨迹优化（近全局参考）
    goal_pos = of(opts, 'goal_pos', 0.1);
    goal_ang = of(opts, 'goal_ang', 0.2);
    verbose  = of(opts, 'verbose', true);
    if ~isempty(seed), rng(seed); end
    if ischar(policy) || isstring(policy)
        if strcmp(policy, 'rrtstar')
            use_ref = true;
        else
            error('evaluatePolicy:policy', '未知策略 %s', policy);
        end
    else
        use_ref = false;
    end

    % ---- 测试集（独立采样，不泄漏到示范训练集） ----
    tests = cell(1, n_test);
    for k = 1:n_test
        [m, q0, tgt] = sampleTask2D(struct('N', N, 'L_seg', L));
        tests{k} = struct('model', m, 'q0', q0, 'tgt', tgt);
    end

    n_succ = 0;  n_coll = 0;
    endpos = zeros(n_test, 1);  endang = zeros(n_test, 1);
    gaps = [];  gens = zeros(n_test, 1);

    for k = 1:n_test
        m = tests{k}.model;  q0 = tests{k}.q0;  tgt = tests{k}.tgt;
        % ---- 策略生成 ----
        t0 = tic;
        if use_ref
            info = method_rrtstar(m, q0, tgt, struct('max_samples', ref_ms));
            out = struct('traj', info.q_snapshot, 'q_final', info.q_final);
        else
            out = policy(m, q0, tgt);
        end
        gens(k) = toc(t0) * 1000;   % ms
        if isempty(out) || ~isstruct(out) || isempty(out.q_final)
            endpos(k) = inf;  endang(k) = inf;  continue;   % 策略失败
        end
        qf = out.q_final(:)';
        % ---- 终态误差 ----
        [~, pe] = planarFK_L(qf, m.DH, m.cfg.rod_offset_arr);
        th = getEndEffectorAngle_L(qf, m.DH, m.cfg.rod_offset_arr);
        endpos(k) = norm(pe - tgt(1:2));
        endang(k) = abs(wrapAngle(tgt(3) - th));
        if endpos(k) < goal_pos && endang(k) < goal_ang, n_succ = n_succ + 1; end
        % ---- 全程碰撞 ----
        if ~isempty(out.traj)
            if ~isFreeAll(m, out.traj), n_coll = n_coll + 1; end
        end
        % ---- 最优性差距（vs 基线 RRT*） ----
        if ~use_ref
            info_r = method_rrtstar(m, q0, tgt, struct('max_samples', ref_ms));
            if info_r.success
                % 参考侧也做轨迹优化（近全局参考 = 预算内最优 + 短切）
                if ref_opt && size(info_r.q_snapshot, 1) > 3
                    tr_r = optimizeTraj(m, info_r.q_snapshot, struct('n_shortcut', 600, 'smooth_win', 3));
                else
                    tr_r = info_r.q_snapshot;
                end
                % 空 traj（策略只给 q_final）无法算路径代价，跳过 gap（避免误报 -100%）
                if ~isempty(out.traj) && size(out.traj, 1) >= 2
                    c_pol = sum(vecnorm(diff([q0; out.traj], 1, 1), 2, 2));
                    c_ref = sum(vecnorm(diff(tr_r, 1, 1), 2, 2));
                    gaps(end+1) = (c_pol - c_ref) / max(c_ref, 1e-9); %#ok<AGROW>
                end
            end
        end
    end

    res = struct(...
        'success_rate', n_succ / n_test, ...
        'collision_rate', n_coll / n_test, ...
        'mean_end_pos', mean(endpos), 'mean_end_ang', mean(endang), ...
        'mean_gap', mean(gaps), 'gap_std', std(gaps), ...
        'mean_gen_s', mean(gens) / 1000, ...
        'per_case', struct('endpos', endpos, 'endang', endang));
    if verbose
        fprintf('[evaluate] 成功率 %.0f%% (%d/%d) | 碰撞率 %.0f%% | 末端 pos=%.3f ang=%.3f', ...
            res.success_rate*100, n_succ, n_test, res.collision_rate*100, ...
            res.mean_end_pos, res.mean_end_ang);
        if ~use_ref && ~isempty(gaps)
            fprintf(' | 最优性差距 %+.1f%%±%.1f%%', res.mean_gap*100, res.gap_std*100);
        end
        fprintf(' | 生成 %.1f ms\n', res.mean_gen_s*1000);
    end
end

function ok = isFreeAll(model, traj)
    ok = true;
    for k = 1:size(traj, 1)
        g = obsDistAll(model, traj(k, :));
        if ~isempty(g) && min(g) < model.cfg.rho0
            ok = false; return;
        end
    end
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
