function rep = tcpDecimateCheck(model, q, varargin)
%tcpDecimateCheck 评估抽稀因子 n 的末端路径误差，并给出建议 n（选项2）
%   rep = tcpDecimateCheck(model, q, opts)
%   model: createArmModel 输出（用 planarFK_L 算末端位置）
%   q    : [K×N] 绝对关节角(rad) 的仿真轨迹
%   opts : .step_per_rev(40000)  .ee_tol(1e-3 m)  .n_scan([1 2 3 4 5 8 10 12 15 20 25 33])
%
%   误差模型：抽稀到每 n 步取点后，两下发点之间假设电控做【线性关节插值】，
%   把该线性近似路径与原始仿真轨迹逐点比较，取【末端位置(max)误差】。
%   rep: .n_candidates .max_ee_err(每 n, m) .best_n(满足 ee_tol 的最大 n) .step_deg
    p = inputParser;
    addParameter(p,'step_per_rev', 40000);
    addParameter(p,'ee_tol', 1e-3);
    addParameter(p,'n_scan', [1 2 3 4 5 8 10 12 15 20 25 33]);
    parse(p,varargin{:});
    tol = p.Results.ee_tol;  ns = p.Results.n_scan;
    K = size(q,1);
    cfg = model.cfg;
    ee0 = zeros(K,2);
    for k = 1:K
        [~, pe] = planarFK_L(q(k,:), model.DH, cfg.rod_offset_arr);
        ee0(k,:) = pe;
    end
    rep.n_candidates = ns;
    rep.max_ee_err = zeros(1, numel(ns));
    for i = 1:numel(ns)
        n = ns(i);
        idx = [1:n:K];  if idx(end) ~= K, idx = [idx K]; end
        mx = 0;
        for j = 1:numel(idx)-1
            a = idx(j);  b = idx(j+1);
            for k = a:b
                w = (k-a)/max(1,(b-a));
                qq = q(a,:) + w*(q(b,:) - q(a,:));            % 线性关节插值
                [~, pe] = planarFK_L(qq, model.DH, cfg.rod_offset_arr);
                mx = max(mx, norm(pe - ee0(k,:)));
            end
        end
        rep.max_ee_err(i) = mx;
    end
    ok = find(rep.max_ee_err <= tol, 1, 'last');
    if isempty(ok), rep.best_n = ns(1); else, rep.best_n = ns(ok); end
    rep.step_deg = 360/p.Results.step_per_rev;
end
