function val = clampVal(x, low, high)
%clampVal 将 x 截断到 [low, high]
    val = min(max(x, low), high);
end
