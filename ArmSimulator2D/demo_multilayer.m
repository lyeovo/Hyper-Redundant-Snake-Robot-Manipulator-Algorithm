function demo_multilayer(seeds)
%demo_multilayer 调用 method_multilayer（骨架图→候选路径→逐段求解）验证
%   demo_multilayer(seeds)
if nargin<1||isempty(seeds), seeds=1:6; end
for s=seeds(:)'
    sc=sampleObstacleScene(struct('seed',s));
    tgt=[sc.goal, 0];
    t=tic;
    info=simulateMotion(sc.model, 'multilayer', sc.q0, tgt, 'Snapshot', 4);
    dt=toc(t);
    if isfield(info,'stats') && isfield(info.stats,'n_nodes')
        fprintf('s%d: success=%d err=%.3f ang=%.3f method=%s nodes=%d edges=%d cand=%d tried=%d t=%.1fs\n', ...
            s, info.success, info.dist_end, info.err_ang, info.method_used, ...
            info.stats.n_nodes, info.stats.n_edges, info.stats.n_candidates, info.stats.tried, dt);
    else
        fprintf('s%d: success=%d err=%.3f ang=%.3f method=%s t=%.1fs (fallback/other)\n', ...
            s, info.success, info.dist_end, info.err_ang, info.method_used, dt);
    end
end
end
