function v = optget(s, field, default)
%optget 从 struct 取字段，缺失/空则回退默认值（全局统一 helper）
%   v = optget(s, field, default)
%   s      : struct（也可为 [] 或非 struct，此时一律返回 default）
%   field  : 字段名
%   default: 缺省值
%
%   语义：字段存在且非空 → 取其值；否则（不存在 / 为空 / s 非 struct）→ default。
%
%   历史：本函数此前以 of / getopt / getopt2 / getf / gp / ifs 六种名字在 26 个文件里
%   各自定义了一遍（函数体完全相同）。现统一为 optget，各文件不再保留本地副本。
%   注意 MATLAB R2016b+ 自带 inputParser / arguments 块，但本项目大量使用
%   "轻量 struct 选项"传递，optget 与之配套，保持最小依赖。
    if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
