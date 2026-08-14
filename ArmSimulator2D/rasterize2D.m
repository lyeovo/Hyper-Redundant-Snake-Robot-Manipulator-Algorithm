function img = rasterize2D(circles, rects, GRID, EXTENT)
%rasterize2D 障碍 → 占据栅格（与 train_policy.py/train_cvae.py rasterize_obs 严格一致）
%   2D 特征编码；3D 演进替换为体素/点云编码（策略加载器唯一改动点）
    xmin = EXTENT(1); xmax = EXTENT(2); ymin = EXTENT(3); ymax = EXTENT(4);
    img = zeros(GRID, GRID);
    [yy, xx] = meshgrid(1:GRID, 1:GRID);
    gx = xmin + (xx - 0.5) * (xmax - xmin) / GRID;
    gy = ymin + (yy - 0.5) * (ymax - ymin) / GRID;
    for k = 1:size(circles, 1)
        c = circles(k, :);
        img((gx - c(1)).^2 + (gy - c(2)).^2 <= c(3)^2) = 1;
    end
    for k = 1:size(rects, 1)
        r = rects(k, :);
        ct = cos(r(3)); st = sin(r(3));
        lx = (gx - r(1)) * ct + (gy - r(2)) * st;
        ly = -(gx - r(1)) * st + (gy - r(2)) * ct;
        img(abs(lx) <= r(4)/2 & abs(ly) <= r(5)/2) = 1;
    end
end
