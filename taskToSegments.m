function segs = taskToSegments(cmd, ~, approach_dist)
%taskToSegments command_type → 运动段展开（修正版任务类型集）
%   segs = taskToSegments(cmd, model, approach_dist)
%   cmd : TaskCommand（不改结构，参数从 existing 字段读：
%           move_along/move_for_place/rotate/rotate_arm/facing_arm 的 θ,d,α,n 取 cmd.motion_params
%           （键：theta/d/alpha/n）；位置类取 selected_target/destination.pose_camera/pose_base）
%   segs: struct 数组，字段：
%           .target [x,y,θ]    末端目标（kind='ee'）
%           .kind   'ee'|'ee_relative'|'ee_rotate'|'joint_delta'|'joint_abs'
%           .gripper 0保持 1开 2合   .name 当前步名
%           以及相对/关节字段：.dir,.dist,.alpha,.joint,.delta,.angle（按 kind 取）
%
%   类型 → 展开（参数由 motion_params 提供，缺省用目标/默认）：
%     move_to       : ee   → [x,y,θ]
%     move_along    : ee_relative → 当前末端沿 θ 方向 d 米
%     move_for_pick : ee   → selected_target（实时物体位）
%     move_for_place: ee   → destination 硬编码放置位
%     rotate        : ee_rotate → 末端位置不动，θ 转 α
%     rotate_arm    : joint_delta → 第 n 关节转 α
%     facing_arm    : joint_abs  → 第 n 关节连杆朝 θ 方向
%     pick          : [朝向物体] + [抓取(合)] + [后撤]
%     place         : [开爪]
%     withdraw      : [沿 -target 方向后撤 approach]
%     reset         : [回 [0,0,0]]
    if nargin < 3 || isempty(approach_dist), approach_dist = 0.15; end
    ct = cmd.command_type;
    prm = optget(cmd, 'motion_params', struct());
    tgt = projectTo2D(cmd.selected_target.pose_camera);   % [x,y,θ]
    A  = optget(prm,'alpha', 0.5);   n = max(1, round(optget(prm,'n', 1)));
    D  = optget(prm,'d', approach_dist);
    Th = optget(prm,'theta', tgt(3));   % θ：move_along 方向 / facing_arm 目标朝向 共用 'theta'

    switch ct
        % ---- 末端位置类 ----
        case 'move_to'
            segs = mk('ee', [tgt(1), tgt(2), tgt(3)], 0, 'MOVING_TO');

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
