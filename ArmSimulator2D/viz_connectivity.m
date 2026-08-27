function viz_connectivity(seeds, outdir)
%viz_connectivity 把可行场景的连通图（节点/边/障碍/起终点）画成 PNG 供查看
%   viz_connectivity(seeds, outdir)
%   seeds : 场景种子数组（默认 [1 2 3 4 5]）
%   outdir: 输出目录（默认 'viz'）
%   依赖：sampleObstacleScene / buildConnectivityGraph（ArmSimulator2D)
    if nargin < 1 || isempty(seeds), seeds = [1 2 3 4 5]; end
    if nargin < 2, outdir = 'viz'; end
    if ~exist(outdir,'dir'), mkdir(outdir); end
    for s = seeds(:)'
        sc = sampleObstacleScene(struct('seed', s));
        [graph, info] = buildConnectivityGraph(sc.obs_desc, sc.start, sc.goal, ...
            struct('GRID', 64, 'model', sc.model));
        fig = figure('Visible','off','Position',[80 80 950 950]);
        clf; hold on;
        R = sc.model.cfg.N * sc.model.cfg.L_seg(1);
        th = linspace(0,2*pi,120);
        % 可达参考圆
        plot(R*cos(th), R*sin(th), ':', 'Color',[0.65 0.65 0.7], 'LineWidth',0.8);
        % 障碍：圆 + 可旋转矩形（红）
        for ci = 1:size(sc.obs_desc.circles,1)
            c = sc.obs_desc.circles(ci,:);
            fill(c(1)+c(3)*cos(th), c(2)+c(3)*sin(th), [1 0.35 0.35], ...
                'FaceAlpha',0.30, 'EdgeColor',[0.85 0.15 0.15], 'LineWidth',1.4);
        end
        for ri = 1:size(sc.obs_desc.rects,1)
            r = sc.obs_desc.rects(ri,:);  ct=cos(r(3)); st=sin(r(3)); hw=r(4)/2; hh=r(5)/2;
            corners = [r(1)-hw*ct+hh*st, r(2)-hw*st-hh*ct;
                       r(1)+hw*ct+hh*st, r(2)+hw*st-hh*ct;
                       r(1)+hw*ct-hh*st, r(2)+hw*st+hh*ct;
                       r(1)-hw*ct-hh*st, r(2)-hw*st+hh*ct];
            patch(corners(:,1),corners(:,2),[1 0.35 0.35],'FaceAlpha',0.30, ...
                'EdgeColor',[0.85 0.15 0.15],'LineWidth',1.4);
        end
        % 边（可见性，浅蓝）
        nodes = graph.nodes;
        for e = 1:size(graph.edges,1)
            a = graph.edges(e,1);  b = graph.edges(e,2);
            plot(nodes([a b],1), nodes([a b],2), '-', 'Color',[0.72 0.86 1], 'LineWidth',0.9);
        end
        % 节点（取出的走廊中心点，蓝）
        plot(nodes(:,1), nodes(:,2), 'o', 'MarkerSize',4.5, ...
            'MarkerFaceColor',[0.15 0.45 1], 'MarkerEdgeColor',[0.1 0.35 0.9]);
        % 起终点
        plot(sc.start(1), sc.start(2), 'g^', 'MarkerSize',11, 'MarkerFaceColor',[0 0.8 0]);
        plot(sc.goal(1),  sc.goal(2),  'mx', 'MarkerSize',12, 'LineWidth',2.2, 'MarkerSize',11);
        hold off; axis equal; grid on;
        title(sprintf('scene %d | %d nodes, %d edges | %d circ, %d rect | start→goal reachable=%d', ...
            s, info.n_nodes, info.n_edges, size(sc.obs_desc.circles,1), size(sc.obs_desc.rects,1), info.start_goal_reachable), ...
            'FontSize', 9);
        xlabel('x (m)'); ylabel('y (m)');
        fname = fullfile(outdir, sprintf('scene_%02d.png', s));
        print(fig, '-dpng', '-r70', fname);
        close(fig);
    end
    fprintf('已保存 %d 个场景图到 %s\n', numel(seeds), outdir);
end
