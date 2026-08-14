function segs = taskToSegments(cmd, model, approach_dist)
%taskToSegments command_type → 运动序列展开（方案 §1.1 展开表）
%   segs = taskToSegments(cmd, model, approach_dist)
%   cmd: TaskCommand struct（视觉 UI 文件桥格式）
%   segs: struct 数组，每段 .target=[x,y,θ] .gripper（0=保持 1=开 2=合） .name（当前步名）
%
%   展开规则：
%     move_near_target : [approach]
%     pick_target      : [approach] → [grasp+close] → [lift]
%     pick_and_place   : [approach] → [grasp+close] → [lift] → [approach_place] → [place+open]
%     dock_to_interface: [pre] → [dock（θ 高精度）]
%     home             : [零位]
    if nargin < 3 || isempty(approach_dist), approach_dist = 0.15; end
    ct = cmd.command_type;
    tgt = projectTo2D(cmd.selected_target.pose_camera);
    % approach 点：沿目标朝向反方向退 approach_dist（预抓取点）
    lift_h = 0.10;   % 抬升高度（平面内沿 -Z 不可行 → 沿末端朝向反方向为抬离；此处用垂直方向分量 0，
                     % 实际抬离由电控夹爪/垂直轴处理，平面内仅做"后撤"）
    lift_pose = tgt - lift_h * [cos(tgt(3)), sin(tgt(3)), 0];

    switch ct
        case 'move_near_target'
            segs = mksegs(struct('target', tgt - approach_dist*[cos(tgt(3)), sin(tgt(3)), 0], ...
                'gripper', 0, 'name', 'MOVING_TO_TARGET'));

        case 'pick_target'
            segs = mksegs( ...
                struct('target', tgt - approach_dist*[cos(tgt(3)), sin(tgt(3)), 0], 'gripper', 0, 'name', 'MOVING_TO_PREGRASP'), ...
                struct('target', tgt, 'gripper', 2, 'name', 'GRASPING'), ...
                struct('target', lift_pose, 'gripper', 0, 'name', 'LIFTING'));

        case 'pick_and_place'
            if isfield(cmd, 'destination') && ~isempty(cmd.destination) && ...
               isfield(cmd.destination, 'pose_base') && ~isempty(cmd.destination.pose_base)
                dest = projectTo2D(cmd.destination.pose_base);
            else
                dest = tgt + [0.6, 0.0, 0];   % 仿真默认放置点（目标旁 0.6m）
            end
            segs = mksegs( ...
                struct('target', tgt - approach_dist*[cos(tgt(3)), sin(tgt(3)), 0], 'gripper', 0, 'name', 'MOVING_TO_PREGRASP'), ...
                struct('target', tgt, 'gripper', 2, 'name', 'GRASPING'), ...
                struct('target', lift_pose, 'gripper', 0, 'name', 'LIFTING'), ...
                struct('target', dest - approach_dist*[cos(dest(3)), sin(dest(3)), 0], 'gripper', 0, 'name', 'MOVING_TO_PLACE'), ...
                struct('target', dest, 'gripper', 1, 'name', 'RELEASING'));

        case 'dock_to_interface'
            segs = mksegs( ...
                struct('target', tgt - approach_dist*[cos(tgt(3)), sin(tgt(3)), 0], 'gripper', 0, 'name', 'MOVING_TO_PRE_DOCK'), ...
                struct('target', tgt, 'gripper', 0, 'name', 'DOCKING'));

        case 'home'
            segs = mksegs(struct('target', [0, 0, 0], 'gripper', 0, 'name', 'MOVING_HOME'));

        otherwise
            error('taskToSegments:type', '未知 command_type: %s', ct);
    end
end

function s = mksegs(varargin)
    s = struct('target', cell(1, nargin), 'gripper', cell(1, nargin), 'name', cell(1, nargin));
    for k = 1:nargin
        s(k).target = varargin{k}.target;
        s(k).gripper = varargin{k}.gripper;
        s(k).name = varargin{k}.name;
    end
end
