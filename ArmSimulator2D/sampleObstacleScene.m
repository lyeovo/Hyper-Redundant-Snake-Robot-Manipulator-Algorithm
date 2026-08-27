function scene = sampleObstacleScene(opts)
%sampleObstacleScene 生成带 2-5 个随机障碍的工作空间场景（连通图验证用）
%   scene = sampleObstacleScene(opts)
%   opts: .N(默认 6) .L_seg(默认 1.04393) .n_obs(默认 2-5 随机) .seed(可复现)
%
%   返回 scene:
%     .model     createArmModel 输出（含随机障碍布局）
%     .q0        [1×N] 随机起点构型（干扰用）
%     .start     [1×2] 起点末端工作空间位置
%     .goal      [1×2] 目标工作空间位置（可达且障碍外）
%     .obs_desc  struct('circles',[n×3],'rects',[n×5])
%
%   障碍约束：总数 2~5；圆/可旋转矩形随机混合；分布在工作空间环带，
%   并避开起点与目标（保证连通图有实际意义）。
    if nargin < 1 || isempty(opts), opts = struct(); end
    if isfield(opts,'seed') && ~isempty(opts.seed), rng(opts.seed); end
    N = of(opts,'N',6);
    L = of(opts,'L_seg',1.04393);
    nobs = of(opts,'n_obs',0);
    if nobs < 2 || nobs > 5, nobs = randi([2,5]); end
    R = N * L;

    % ---- 随机 2-5 个障碍（圆 / 可旋转矩形，环带 0.4R-0.85R，避开中心区）----
    circles = zeros(0,3);  rects = zeros(0,5);
    guard = 0;  ncirc = 0;  nrect = 0;
    while (ncirc + nrect) < nobs && guard < 12*nobs
        guard = guard + 1;
        rr = R*(0.40 + 0.45*rand);  a = 2*pi*rand;
        x = rr*cos(a);  y = rr*sin(a);
        if rand < 0.5 && ncirc < nobs-1        % 圆
            r0 = R*(0.10 + 0.12*rand);
            if x^2 + y^2 > (r0+0.15*R)^2       % 别把起点(基座)圈住
                circles(end+1,:) = [x, y, r0]; ncirc = ncirc+1; %#ok<AGROW>
            end
        elseif nrect < nobs                    % 可旋转矩形
            th = (rand-0.5)*1.2;
            w = R*(0.12+0.10*rand);  h = R*(0.06+0.06*rand);
            rects(end+1,:) = [x, y, th, w, h]; nrect = nrect+1; %#ok<AGROW>
        end
    end

    model = createArmModel(struct('N',N,'L_seg',L, ...
        'obstacles', struct('circles',circles,'rects',rects)));
    cfg = model.cfg;

    % ---- 随机自由起点构型（避开障碍安全区）→ start（末端位置）----
    q0 = sampleFreeQ(model);
    [~, pe0] = planarFK_L(q0, model.DH, cfg.rod_offset_arr);
    start = pe0(1:2);

    % ---- 可达且障碍外的目标工作空间点 ----
    goal = sampleFreeGoal(model, start, R);

    scene = struct('model',model,'q0',q0,'start',start,'goal',goal, ...
        'obs_desc', struct('circles',circles,'rects',rects));
end

%% ---------- 工具 ----------
function q0 = sampleFreeQ(model)
    % 当前 q_min/q_max 内采样碰撞自由的起点构型（最多 50 次）
    cfg = model.cfg;
    q0 = zeros(1, cfg.N);
    for t = 1:50
        qc = cfg.q_min + (cfg.q_max - cfg.q_min).*rand(1,cfg.N);
        g = obsDistAll(model, qc);
        if isempty(g) || min(g) >= cfg.rho0 + 0.12, q0 = qc; return; end
    end
end

function p = sampleFreeGoal(model, start, R)
    % 在可达域(半径 R)内采样障碍外、离起点不太近的目标工作空间点
    cfg = model.cfg;
    for t = 1:100
        ang = 2*pi*rand;  rr = R*(0.3 + 0.65*rand);
        p = [rr*cos(ang), rr*sin(ang)];
        if norm(p - start) < 0.4*R, continue; end
        if isPtFree(model, p), return; end
    end
    % 兜底：起终点（绕开障碍的方向上取一个自由点）
    for ang = 0:0.2:2*pi
        p = start + 0.5*R*[cos(ang), sin(ang)];
        if isPtFree(model, p) && norm(p-start) > 0.3*R, return; end
    end
end

function ok = isPtFree(model, p)
    cfg = model.cfg;  ok = true;
    for ci = 1:size(cfg.obstacles.circles,1)
        c = cfg.obstacles.circles(ci,:);
        if norm(p - c(1:2)) < c(3) + cfg.rho0 + 0.02, ok = false; return; end
    end
    for ri = 1:size(cfg.obstacles.rects,1)
        r = cfg.obstacles.rects(ri,:);  ct = cos(r(3)); st = sin(r(3));
        lx = (p(1)-r(1))*ct + (p(2)-r(2))*st;
        ly = -(p(1)-r(1))*st + (p(2)-r(2))*ct;
        if abs(lx) <= r(4)/2 + cfg.rho0 && abs(ly) <= r(5)/2 + cfg.rho0, ok = false; return; end
    end
end

function v = of(s, f, d)
    if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
