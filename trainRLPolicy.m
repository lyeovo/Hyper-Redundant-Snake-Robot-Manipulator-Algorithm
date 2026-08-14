function stats = trainRLPolicy(opts)
%trainRLPolicy RL 离线训练：任务族 OpenAI-ES（学会通用避障趋目标策略）
%   stats = trainRLPolicy(opts)
%
%   用途：GUI/命令行选择 'rl' 方法时，若存在 RLPolicy.mat 则自动加载推理
%   （不再每次现训练），否则回退为任务内现训练。本脚本用于离线训练并保存策略。
%
%   opts（可缺省）:
%     .n_gen          进化代数（默认 500；建议 1000-5000，看成功率曲线）
%     .n_pop          扰动种群数（默认 16）
%     .sigma_es       扰动标准差（默认 0.2）
%     .lambda         residual 权重（默认 0.3）
%     .max_rollout    rollout 步数（默认 50）
%     .tasks_per_gen  每代采样任务数（默认 4；越大策略越泛化、单代越慢）
%     .lr             学习率（默认 0.5）
%     .save_path      保存路径（默认 'RLPolicy.mat'）
%     .load_path      续训起点（默认 ''，从零开始；填上次保存文件即可续训）
%     .eval_every     每多少代评估一次（默认 50）
%     .verbose        进度打印（默认 true）
%
%   训练产出 RLPolicy.mat：theta（策略参数）+ 元数据（N、lam、max_roll、成功率曲线）。
%   Ctrl+C 中断时自动保存当前进度（onCleanup），重跑时传 load_path 续训。
%
%   示例：
%     trainRLPolicy();                              % 默认 500 代
%     trainRLPolicy(struct('n_gen',2000,'tasks_per_gen',8));   % 大训练
%     trainRLPolicy(struct('load_path','RLPolicy.mat'));       % 续训
    if nargin < 1 || isempty(opts), opts = struct(); end
    n_gen    = of(opts,'n_gen', 500);
    n_pop    = of(opts,'n_pop', 16);
    sig      = of(opts,'sigma_es', 0.2);
    lam      = of(opts,'lambda', 0.3);
    max_roll = of(opts,'max_rollout', 50);
    tpg      = of(opts,'tasks_per_gen', 4);
    lr       = of(opts,'lr', 0.5);
    save_path = of(opts,'save_path', 'RLPolicy.mat');
    load_path = of(opts,'load_path', '');
    eval_every = of(opts,'eval_every', 50);
    verbose  = of(opts,'verbose', true);

    % 固定训练模型（策略维度依赖 N；训练/推理须同 N）
    base = createArmModel(struct('N', 4, 'L_seg', 1.0));
    cfg = base.cfg;
    N = cfg.N;
    D = 2 + 1 + 1 + N;
    rng(42);   % 可复现

    % ---- 初始化 / 续训 ----
    if ~isempty(load_path) && exist(load_path, 'file') == 2
        S = load(load_path);
        if isfield(S, 'theta')
            theta = S.theta;
            if size(theta, 1) ~= N || size(theta, 2) ~= D
                error('trainRLPolicy:dim', '策略维度 %dx%d 与当前 N=%d (需 %dx%d) 不匹配，请重新训练', ...
                    size(theta,1), size(theta,2), N, N, D);
            end
            if verbose
                fprintf('续训: 载入 %s（theta %dx%d）\n', load_path, size(theta,1), size(theta,2));
            end
        else
            theta = zeros(N, D);
        end
    else
        theta = zeros(N, D);
    end

    % ---- 任务采样器（随机障碍布局 + 随机起始 + 可达目标） ----
    function [model, q0, tgt] = sampleTask()
        circles = zeros(0,3);  rects = zeros(0,5);
        n_c = randi([0, 2]);
        for k = 1:n_c
            cx = 1.2 + 1.6*rand;  cy = -0.6 + 1.6*rand;
            r  = 0.2 + 0.3*rand;
            circles(end+1,:) = [cx, cy, r]; %#ok<AGROW>
        end
        if rand < 0.4
            rects(end+1,:) = [1.0+1.5*rand, -0.5+1.3*rand, (rand-0.5)*1.2, 0.5+0.3*rand, 0.25+0.25*rand]; %#ok<AGROW>
        end
        model = createArmModel(struct('N', N, 'L_seg', 1.0, ...
            'obstacles', struct('circles', circles, 'rects', rects)));
        % 随机起始构型
        q0 = cfg.q_min + (cfg.q_max - cfg.q_min) .* rand(1, N);
        % 可达目标：随机构型的 FK 末端 ± 小扰动（可能被障碍隔开 → 学会绕行）
        qr = cfg.q_min + (cfg.q_max - cfg.q_min) .* rand(1, N);
        [~, pe] = planarFK_L(qr, model.DH, cfg.rod_offset_arr);
        tgt = [pe(1) + 0.25*randn, pe(2) + 0.25*randn, (rand-0.5)*pi];
    end

    % ---- 评估（固定测试集 20 个任务） ----
    n_eval = 20;
    ev = cell(1, n_eval);
    for k = 1:n_eval
        [ev{k}.model, ev{k}.q0, ev{k}.tgt] = sampleTask();
    end
    function [succ, dmean] = evaluate(th)
        nsucc = 0;  dsum = 0;
        for k = 1:n_eval
            [d, ~, ~, ~] = rlRollout(ev{k}.model, ev{k}.q0, ev{k}.tgt, th, lam, max_roll);
            dsum = dsum + d;
            if d < cfg.rrt_goal_eps, nsucc = nsucc + 1; end
        end
        succ = nsucc / n_eval;
        dmean = dsum / n_eval;
    end

    % ---- 进度记录 ----
    succ_hist = [];  dmean_hist = [];
    best_succ = -1;  best_theta = theta;
    t_start = tic;

    % Ctrl+C 中断时保存进度（断点续训）
    cleanup = onCleanup(@() saveCheckpoint());
    function saveCheckpoint()
        S = struct('theta', theta, 'lam', lam, 'N', N, 'max_roll', max_roll, ...
            'n_gen', n_gen, 'succ_hist', succ_hist, 'dmean_hist', dmean_hist, ...
            'best_succ', best_succ, 'best_theta', best_theta, ...
            'date', datestr(now), 'type', 'RLPolicy');
        save(save_path, '-struct', 'S');
        if verbose
            fprintf('\n[checkpoint] 已保存 %s（当前成功率 %.1f%%）\n', save_path, best_succ*100);
        end
    end

    % ---- ES 主循环 ----
    for gen = 1:n_gen
        eps = sig * randn(n_pop, numel(theta));
        R = zeros(n_pop, 1);
        for p = 1:n_pop
            th_p = theta + reshape(eps(p,:), size(theta));
            rp = 0;
            for t = 1:tpg
                [m, q0i, tgti] = sampleTask();
                [d, a, ~, ~] = rlRollout(m, q0i, tgti, th_p, lam, max_roll);
                rp = rp - (d + 0.1*a);
            end
            R(p) = rp / tpg;
        end
        R = (R - mean(R)) / max(std(R), 1e-6);
        theta = theta + lr * reshape(eps' * R, size(theta)) / (n_pop * sig);

        % 定期评估 + 进度
        if mod(gen, eval_every) == 0 || gen == n_gen
            [succ, dmean] = evaluate(theta);
            succ_hist(end+1) = succ; %#ok<AGROW>
            dmean_hist(end+1) = dmean; %#ok<AGROW>
            if succ >= best_succ
                best_succ = succ;  best_theta = theta;
            end
            if verbose
                fprintf('[gen %4d/%d] 评估: 成功率 %.0f%% | 平均末端距离 %.3f m | 累计 %.1fs\n', ...
                    gen, n_gen, succ*100, dmean, toc(t_start));
            end
            if succ >= 0.9 && mod(gen, eval_every) == 0
                % 早停：测试集成功率 ≥90% 且保持一个评估周期
                if isfield(opts, 'early_stop') && opts.early_stop
                    if verbose, fprintf('早停：成功率 %.0f%% ≥ 90%%\n', succ*100); end
                    break;
                end
            end
        end
    end

    % ---- 收尾：保存最优策略 ----
    theta = best_theta;
    [final_succ, final_dmean] = evaluate(theta);
    saveCheckpoint();
    stats = struct('theta', theta, 'lam', lam, 'N', N, 'max_roll', max_roll, ...
        'n_gen', n_gen, 'succ_hist', succ_hist, 'dmean_hist', dmean_hist, ...
        'final_succ', final_succ, 'final_dmean', final_dmean, ...
        'save_path', save_path, 'time_s', toc(t_start));
    if verbose
        fprintf('训练完成: 最终成功率 %.1f%% | 平均末端距离 %.3f m | 策略已存 %s\n', ...
            final_succ*100, final_dmean, save_path);
    end
end

function v = of(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
