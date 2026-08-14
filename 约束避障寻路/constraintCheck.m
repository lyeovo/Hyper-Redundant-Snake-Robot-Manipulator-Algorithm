function [C_all, feasible] = constraintCheck(theta)
% C_all:12×1 约束值 [C1+;C1-;C2+;C2-;...;C6+;C6-]
% feasible: bool, 1=全部约束满足，0=存在相交

[phi,Xall,~,~,a,b] = kinMap(theta);

C_all = zeros(12,1);
idx = 1;

for k = 1:6
    phik = phi(k);
    xkm1 = Xall(k);
    ak = a(k);
    bk = b(k);

    sinPk = sin(phik);
    cosPk = cos(phik);

    % ===== H_k^+ 上边界 =====
    Hap = sinPk*(ak - xkm1) - cosPk*(sin(ak) + 0.5);
    Hbp = sinPk*(bk - xkm1) - cosPk*(sin(bk) + 0.5);
    Ck_plus = Hap * Hbp;
    C_all(idx) = Ck_plus;
    idx = idx + 1;

    % ===== H_k^- 下边界 =====
    Ham = sinPk*(ak - xkm1) - cosPk*(sin(ak) - 0.5);
    Hbm = sinPk*(bk - xkm1) - cosPk*(sin(bk) - 0.5);
    Ck_minus = Ham * Hbm;
    C_all(idx) = Ck_minus;
    idx = idx + 1;
end

% 所有约束 > 0 才可行
feasible = all(C_all > 1e-10); % 微小阈值规避浮点误差
end