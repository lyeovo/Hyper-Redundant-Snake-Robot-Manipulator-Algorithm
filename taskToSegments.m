function segs = taskToSegments(cmd, model, approach_dist)
%taskToSegments command_type → 运动段展开（参数与视觉接口统一）
%   segs = taskToSegments(cmd, model, approach_dist)
%   cmd : TaskCommand（不改结构；参数从 cmd.params 读，缺失回退 cmd.motion_params；键遵循
%          TASK_COMMAND_INTERFACE v2.0：角度用度(theta_deg/alpha_deg)、距离用米(distance_m)、
%          关节 1 索引(joint_index)、位置用 x/y；旧键 theta/alpha/d/n(弧度)作向后兼容回退）
%   model : createArmModel 输出（joint_index 按 model.cfg.N 截断）
%   segs: struct 数组，字段：
%           .target [x,y,θ]    末端目标（kind='ee'）
%           .kind   'ee'|'ee_relative'|'ee_rotate'|'joint_delta'|'joint_abs'
%           .gripper 0保持 1开 2合   .name 当前步名
%           以及相对/关节字段：.dir,.dist,.alpha,.joint,.delta,.angle（按 kind 取）
%
%   指令类型 → 展开（参数由 params 提供，缺省用当前末端/目标/默认）：
%     move_to       : ee   → [x,y,θ]（θ 取 selected_target 朝向，缺省 0）
%     move_along    : ee_relative → 当前末端沿 theta_deg 方向 distance_m 米
%     move_for_pick : ee   → selected_target（实时物体位）
%     move_for_place: ee   → destination 硬编码放置位
%     rotate        : ee_rotate → 末端位置不动，θ 转 alpha_deg
%     rotate_arm    : joint_delta → 第 joint_index 关节转 alpha_deg
%     facing_arm    : joint_abs  → 第 joint_index 关节连杆朝 theta_deg 方向
%     pick          : [朝向物体] + [抓取(合)] + [后撤]
%     place         : [开爪]
%     withdraw      : [沿 -target 方向后撤 approach]
%     reset         : [回 [0,0,0]]
%   （emergency_stop / cancel_task 属控制类指令，由 taskExecute 提前处理，不走此展开）
    if nargin < 3 || isempty(approach_dist), approach_dist = 0.15; end
    ct = cmd.command_type;
    % params 为主、motion_params 为向后兼容镜像；角度统一度→弧度（求解器/段用弧度）
    prm = optget(cmd, 'params', optget(cmd, 'motion_params', struct()));
    if isfield(cmd,'selected_target') && isstruct(cmd.selected_target) && ~isempty(cmd.selected_target) ...
            && isfield(cmd.selected_target,'pose_camera') && ~isempty(cmd.selected_target.pose_camera)
        tgt = projectTo2D(cmd.selected_target.pose_camera);   % [x,y,θ] rad
    else
        tgt = [0, 0, 0];   % 无目标（move_to 等）：位置/朝向由 params 提供
    end
    A  = deg2rad(optget(prm,'alpha_deg', rad2deg(optget(prm,'alpha', 0.5))));   % 度→弧度
    n  = max(1, round(optget(prm,'joint_index', optget(prm,'n', 1))));
    if isstruct(model) && isfield(model,'cfg') && isfield(model.cfg,'N') && ~isempty(model.cfg.N)
        n = min(n, model.cfg.N);   % 文档含 16 关节，本地按实际关节数截断
    end
    D  = optget(prm,'distance_m', optget(prm,'d', approach_dist));
    Th = deg2rad(optget(prm,'theta_deg', rad2deg(optget(prm,'theta', tgt(3)))));
    x  = optget(prm,'x', tgt(1));   y = optget(prm,'y', tgt(2));

    switch ct
        % ---- 末端位置类 ----
        case 'move_to'
            segs = mk('ee', [x, y, tgt(3)], 0, 'MOVING_TO');   % x/y 取 params（接口键）

        case 'move_along'
            segs = mk('ee_relative', [], 0, 'MOVING_ALONG', 'dir', Th, 'dist', D);

        case 'move_for_pick'
            segs = mk('ee', [tgt(1), tgt(2), tgt(3)], 0, 'MOVING_TO_OBJECT');

        case 'move_for_place'
            if isfield(cmd,'destination') && isfield(cmd.destination,'pose_base') && ~isempty(cmd.destination.pose_base)
                dest = projectTo2D(cmd.destination.pose_base);
            else
                dest = tgt;   % 缺省：原目标位（硬编码时由调用方给 destination）
            end
            segs = mk('ee', [dest(1), dest(2), dest(3)], 0, 'MOVING_TO_PLACE');

        % ---- 旋转类 ----
        case 'rotate'
            segs = mk('ee_rotate', [], 0, 'ROTATING', 'alpha', A);

        case 'rotate_arm'
            segs = mk('joint_delta', [], 0, 'ROTATING_JOINT', 'joint', n, 'delta', A);

        case 'facing_arm'
            segs = mk('joint_abs', [], 0, 'FACING_LINK', 'joint', n, 'angle', Th);

        % ---- 夹爪 / 回退 ----
        case 'pick'
            segs = mk('ee', [tgt(1), tgt(2), tgt(3)], 2, 'GRASPING');
        case 'place'
            segs = mk('ee', [tgt(1), tgt(2), tgt(3)], 1, 'RELEASING');
        case 'withdraw'
            b = tgt - approach_dist*[cos(tgt(3)), sin(tgt(3)), 0];
            segs = mk('ee', b, 0, 'WITHDRAWING');
        case 'reset'
            segs = mk('ee', [0, 0, 0], 0, 'RESETTING');

        otherwise
            error('taskToSegments:type', '未知 command_type: %s', ct);
    end
end

function s = mk(kind, target, gripper, name, varargin)
% mk 生成一段（kind, target, gripper, name + 可选 joint/delta/angle/dir/dist/alpha）
    s = struct('target', target, 'gripper', gripper, 'name', name, 'kind', kind, ...
        'joint', 0, 'delta', 0, 'angle', 0, 'dir', 0, 'dist', 0, 'alpha', 0);
    for i = 1:2:numel(varargin), s.(varargin{i}) = varargin{i+1}; end
end
