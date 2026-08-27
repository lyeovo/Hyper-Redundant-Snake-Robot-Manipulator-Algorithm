function measureGraph(seeds)
%measureGraph 输出图规模/最短路径段数/搜索耗时（性能估算用）
%   measureGraph(seeds) ; seeds 默认 1:5
if nargin<1||isempty(seeds), seeds=1:5; end
for s=seeds(:)'
    sc=sampleObstacleScene(struct('seed',s));
    [g,info]=buildConnectivityGraph(sc.obs_desc,sc.start,sc.goal,struct('GRID',64,'model',sc.model));
    n=size(g.nodes,1);
    adj=cell(1,n);
    for e=1:size(g.edges,1)
        adj{g.edges(e,1)}(end+1)=g.edges(e,2); adj{g.edges(e,2)}(end+1)=g.edges(e,1);
    end
    % BFS 最短路径段数 + 耗时
    t=tic;
    par=zeros(1,n); vis=false(1,n); Q=g.start_i; vis(g.start_i)=true;
    while ~isempty(Q)
        c=Q(1); Q(1)=[];
        if c==g.goal_i, break; end
        for nb=adj{c}
            if ~vis(nb), vis(nb)=true; par(nb)=c; Q(end+1)=nb; end
        end
    end
    if ~vis(g.goal_i), fprintf('s%d NOT reachable\n',s); continue; end
    plen=0; c=g.goal_i;
    while c~=g.start_i, c=par(c); plen=plen+1; end
    tA=toc(t);
    fprintf('s%d: nodes=%d edges=%d reach=%d shortestSegments=%d bfsMs=%.2f\n', ...
        s, info.n_nodes, info.n_edges, info.start_goal_reachable, plen, tA*1000);
end
end
