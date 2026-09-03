function cfg = motorConfig()
%motorConfig 电控 8 通道电机描述（6 关节电机 + 1 夹爪朝向电机 + 1 夹爪开合电机）
%   cfg = motorConfig()
%   cfg.motors(i): .id(1..8) .name .type .unit .min .max
%   id 1..6 = 臂关节；7 = 夹爪朝向(GripperOri)；8 = 夹爪开合(GripperGap)
    names = {'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','GripperOri','GripperGap'};
    types = {'joint','joint','joint','joint','joint','joint','gripper_ori','gripper_gap'};
    unit  = {'rad','rad','rad','rad','rad','rad','rad','ratio'};
    mn    = { -pi, -pi, -pi, -pi, -pi, -pi, -pi, 0 };
    mx    = {  pi,  pi,  pi,  pi,  pi,  pi,  pi, 1 };
    cfg = struct('count', 8, 'motors', struct('id',{},'name',{},'type',{},'unit',{},'min',{},'max',{}));
    for i = 1:numel(names)
        cfg.motors(i) = struct('id',i,'name',names{i},'type',types{i},'unit',unit{i},'min',mn{i},'max',mx{i});
    end
end
