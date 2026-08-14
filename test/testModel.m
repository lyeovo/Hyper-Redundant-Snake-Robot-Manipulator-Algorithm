function model = testModel(varargin)
%testModel 测试用模型构造器（createArmModel 的键值对包装）
%   model = testModel('Field', value, ...)  缺省字段走 modelDefaults
    p = struct();
    for i = 1:2:length(varargin)
        p.(varargin{i}) = varargin{i+1};
    end
    model = createArmModel(p);
end
