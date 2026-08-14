function [model, q0, target, obs_desc] = sampleTask2D(opts)
%sampleTask2D 随机 2D 任务采样器（示范生成/策略评估共用）
%   [model, q0, target, obs_desc] = sampleTask2D(opts)
%   opts: .N(默认 4) .L_seg(默认 1.0)
%         .n_circle_max(默认 2) .n_rect_max(默认 1)
%         .target_sigma(默认 0.25，可达目标扰动，相对绝对距离)
%         .difficulty(1-3，默认 2：控制障碍数量/大小/目标扰动，1 最易)
%         .seed(可选，可复现)
%
%   几何自适应：障碍分布在可达域半径 R=N·L 的 [0.35R, 0.9R] 环带
%   （避开中心起始区与边缘不可达区），尺寸/数量随 difficulty 缩放——
%   N/L 改变时分布自动适配（3D 演进同理替换采样器本体）。
%
%   返回：
%     model     createArmModel 输出（含随机障碍布局）
%     q0        [1×N] 随机起始构型
%     target    [1×3] 目标位姿 [x, y, θ]
%     obs_desc  struct('circles',[n×3],'rects',[n×5])
    if nargin < 1 || isempty(opts), opts = struct(); end
    if isfield(opts, 'seed') && ~isempty(opts.seed), rng(opts.seed); end
    N = of(opts, 'N', 4);
    L = of(opts, 'L_seg', 1.04393);
    ncm = of(opts, 'n_circle_max', 2);
    nrm = of(opts, 'n_rect_max', 1);
    tsig = of(opts, 'target_sigma', 0.25);
    diff = of(opts, 'difficulty', 2);

    R = N * L;   % 可达域半径（几何自适应基准）

    % 难度控制障碍数量/密度（difficulty=1 稀疏 … 3 密集多障碍）
    % diff 从 opts 读；0 = 随机 1-3（混合难度训练用）
    if diff == 0, diff = randi([1, 3]); end
    switch diff
        case 1, ncm = 1;  nrm = 0;
        case 2, ncm = 2;  nrm = 1;
        case 3, ncm = 4;  nrm = 2;
        otherwise, ncm = min(4, max(1, ncm));  nrm = min(2, max(0, nrm));
    end

    % 障碍布局：环带 [0.35R, 0.9R]，难度控制数量/大小
    circles = zeros(0, 3);  rects = zeros(0, 5);
    n_c = randi([0, ncm]);
    for k = 1:n_c
        rr = R * (0.35 + 0.55*rand);
        a = 2*pi*rand;
        r0 = R * (0.10 + 0.12*rand) * (1 + 0.25*(diff-2));   % 半径 0.10R-0.22R
        circles(end+1, :) = [rr*cos(a), rr*sin(a), r0]; %#ok<AGROW>
    end
    p_rect = [0, 0.4, 0.7];
    if rand < p_rect(diff) && nrm > 0
        n_r = randi([1, nrm]);
        for k = 1:n_r
            rr = R * (0.4 + 0.5*rand);
            a = 2*pi*rand;
            rects(end+1, :) = [rr*cos(a), rr*sin(a), (rand-0.5)*1.2, ...
                R*(0.16+0.10*rand), R*(0.08+0.08*rand)]; %#ok<AGROW>
        end
    end

    model = createArmModel(struct('N', N, 'L_seg', L, ...
        'obstacles', struct('circles', circles, 'rects', rects)));
    cfg = model.cfg;
    % 随机起始构型（必须自由：侵入障碍安全区则重采样，最多 50 次）
    q0 = [];
    for tries = 1:50
        q0c = cfg.q_min + (cfg.q_max - cfg.q_min) .* rand(1, N);
        g = obsDistAll(model, q0c);
        % 间隙要求 rho0+0.12：贴障碍(0.05)的起点可扩展方向锥极窄，RRT 无法生长
        if isempty(g) || min(g) >= cfg.rho0 + 0.12
            q0 = q0c; break;
        end
    end
    if isempty(q0), q0 = zeros(1, N); end
    % 可达目标：随机构型 FK 末端 ± 扰动（目标点必须在障碍外，否则不可达）
    qr = cfg.q_min + (cfg.q_max - cfg.q_min) .* rand(1, N);
    [~, pe] = planarFK_L(qr, model.DH, cfg.rod_offset_arr);
    for tries = 1:50
        target = [pe(1) + tsig*randn, pe(2) + tsig*randn, (rand-0.5)*pi];
        if isTargetFree(model, target(1:2)), break; end
    end

    obs_desc = struct('circles', circles, 'rects', rects);
end

function ok = isTargetFree(model, p)
    % 目标点在所有障碍之外（含安全边距）
    cfg = model.cfg;
    ok = true;
    for k = 1:size(cfg.obstacles.circles, 1)
        c = cfg.obstacles.circles(k, :);
        if norm(p - c(1:2)) < c(3) + cfg.rho0 + 0.12, ok = false; return; end
    end
    for k = 1:size(cfg.obstacles.rects, 1)
        r = cfg.obstacles.rects(k, :);
        ct = cos(r(3)); st = sin(r(3));
        lx = (p(1)-r(1))*ct + (p(2)-r(2))*st;
        ly = -(p(1)-r(1))*st + (p(2)-r(2))*ct;
        if abs(lx) <= r(4)/2 + cfg.rho0 && abs(ly) <= r(5)/2 + cfg.rho0
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
