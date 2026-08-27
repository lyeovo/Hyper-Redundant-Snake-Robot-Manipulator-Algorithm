function paths = graphPaths(graph, K, maxSeg)
%graphPaths 骨架图 start→goal 候选路径（简单路径枚举，按代价升序返回前 K）
%   paths = graphPaths(graph, K, maxSeg)
%   graph  : buildConnectivityGraph 输出
%   K      : 最多返回候选数（默认 6）
%   maxSeg : 路径段数上限（默认 9，防枚举爆炸）
    if nargin < 2 || isempty(K), K = 6; end
    if nargin < 3 || isempty(maxSeg), maxSeg = 9; end
    n = size(graph.nodes,1);
    adj = cell(1,n);
    for e = 1:size(graph.edges,1)
        adj{graph.edges(e,1)}(end+1) = graph.edges(e,2); %#ok<AGROW>
        adj{graph.edges(e,2)}(end+1) = graph.edges(e,1); %#ok<AGROW>
    end
    paths = {};  explored = 0;  MAXEXP = 4000;
    si = graph.start_i;  gi = graph.goal_i;
    collect(si, si);   % 递归收集（见下）
    cost = zeros(1, numel(paths));
    for i = 1:numel(paths)
        pp = paths{i};  c = 0;
        for j = 2:numel(pp)
            c = c + norm(graph.nodes(pp(j),:) - graph.nodes(pp(j-1),:));
        end
        cost(i) = c;
    end
    [~,ord] = sort(cost);
    paths = paths(ord);
    if numel(paths) > K, paths = paths(1:K); end

    function collect(cur, p)
        explored = explored + 1;
        if explored > MAXEXP, return; end
        if numel(p) > maxSeg+1, return; end
        if cur == gi
            paths{end+1} = p; %#ok<AGROW>
            return;
        end
        for nb = adj{cur}
            if ~ismember(nb, p), collect(nb, [p nb]); end
        end
    end
end
