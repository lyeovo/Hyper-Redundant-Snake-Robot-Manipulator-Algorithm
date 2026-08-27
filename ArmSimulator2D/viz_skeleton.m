function viz_skeleton(seeds, outdir)
%viz_skeleton 骨架(中轴线)取点参考图例：障碍 + 骨架中轴线(边) + 节点(交叉/端点) + 起终点
%   对照：当前 buildConnectivityGraph 是"PRM 式均匀"；本图是"骨架走廊式"取点参考。
%   viz_skeleton(seeds, outdir)
    if nargin < 1 || isempty(seeds), seeds = [1 2 3]; end
    if nargin < 2, outdir = 'viz'; end
    if ~exist(outdir,'dir'), mkdir(outdir); end
    for s = seeds(:)'
        sc = sampleObstacleScene(struct('seed', s));
        [~, info] = buildConnectivityGraph(sc.obs_desc, sc.start, sc.goal, ...
            struct('GRID', 128, 'model', sc.model));   % 用 128 分辨率取精细骨架
        free = info.occ;  gx = info.gx;  gy = info.gy;  dx = info.dx;
        skel = bwskel(free, 'MinBranchLength', 4);      % 骨架中轴线（去短毛刺）
        neigh = conv2(double(skel), ones(3,3), 'same') - double(skel);
        junc  = skel & neigh >= 3;                       % 交叉点（走廊分叉）
        ends  = skel & neigh == 1;                       % 端点（走廊尽头）
        [jR,jC] = find(junc);  [eR,eC] = find(ends);
        jn = [gx(jC).', gy(jR)];  en = [gx(eC).', gy(eR)];   % N×2 [x,y]（行向量索引→转置成列）
        % 沿骨架均匀取一些"走廊中点"作其余节点（蓝）
        [sR,sC] = find(skel);
        allSk = [gx(sC).', gy(sR)];
        rej = zeros(0,2);
        for k = 1:size(jn,1), rej(end+1,:) = jn(k,:); end %#ok<AGROW>
        for k = 1:size(en,1), rej(end+1,:) = en(k,:); end %#ok<AGROW>
        field = allSk;  picked = zeros(0,2);
        dmin = 0.20 * sc.model.cfg.N * sc.model.cfg.L_seg(1);   % 0.20R 间距
        for k = 1:size(field,1)
            p = field(k,:);
            if min([sqrt(sum((p-jn).^2,2)); sqrt(sum((p-en).^2,2)); sqrt(sum((p-picked).^2,2))]) >= dmin
                picked(end+1,:) = p; %#ok<AGROW>
            end
        end
        % ---- 绘图 ----
        fig = figure('Visible','off','Position',[80 80 950 950]); clf; hold on;
        Rl = sc.model.cfg.N * sc.model.cfg.L_seg(1);  th = linspace(0,2*pi,120);
        plot(Rl*cos(th), Rl*sin(th), ':', 'Color',[0.65 0.65 0.7], 'LineWidth',0.8);
        for ci = 1:size(sc.obs_desc.circles,1)
            c = sc.obs_desc.circles(ci,:);
            fill(c(1)+c(3)*cos(th), c(2)+c(3)*sin(th), [1 0.35 0.35], ...
                'FaceAlpha',0.30,'EdgeColor',[0.85 0.15 0.15],'LineWidth',1.4);
        end
        for ri = 1:size(sc.obs_desc.rects,1)
            r = sc.obs_desc.rects(ri,:); ct=cos(r(3)); st=sin(r(3)); hw=r(4)/2; hh=r(5)/2;
            corners=[r(1)-hw*ct+hh*st,r(2)-hw*st-hh*ct; r(1)+hw*ct+hh*st,r(2)+hw*st-hh*ct; ...
                     r(1)+hw*ct-hh*st,r(2)+hw*st+hh*ct; r(1)-hw*ct-hh*st,r(2)-hw*st+hh*ct];
            patch(corners(:,1),corners(:,2),[1 0.35 0.35],'FaceAlpha',0.30, ...
                'EdgeColor',[0.85 0.15 0.15],'LineWidth',1.4);
        end
        % 骨架(边)
        [skR,skC]=find(skel);  plot(gx(skC), gy(skR), '.', 'Color',[0.45 0.9 0.9], 'MarkerSize',3);
        % 走廊中点节点(蓝)
        if ~isempty(picked), plot(picked(:,1),picked(:,2),'o','MarkerSize',4.5, ...
            'MarkerFaceColor',[0.15 0.45 1],'MarkerEdgeColor',[0.1 0.35 0.9]); end
        % 交叉点(大红)/端点(绿方)
        if ~isempty(jn), plot(jn(:,1),jn(:,2),'s','MarkerSize',7,'MarkerFaceColor',[0.9 0.15 0.15], ...
            'MarkerEdgeColor',[0.6 0.05 0.05]); end
        if ~isempty(en), plot(en(:,1),en(:,2),'^','MarkerSize',7,'MarkerFaceColor',[0.2 0.8 0.2], ...
            'MarkerEdgeColor',[0.1 0.5 0.1]); end
        plot(sc.start(1),sc.start(2),'g^','MarkerSize',11,'MarkerFaceColor',[0 0.8 0]);
        plot(sc.goal(1), sc.goal(2), 'mx', 'MarkerSize',11, 'LineWidth',2.2);
        hold off; axis equal; grid on;
        title(sprintf('SKELETON scene %d | %d junctions, %d endpoints, %d mid-nodes | reach=%d', ...
            s, size(jn,1), size(en,1), size(picked,1), info.start_goal_reachable), 'FontSize',9);
        xlabel('x (m)'); ylabel('y (m)');
        print(fig, '-dpng', '-r70', fullfile(outdir, sprintf('skeleton_%02d.png', s)));
        close(fig);
    end
    fprintf('已保存骨架参考图到 %s\n', outdir);
end
