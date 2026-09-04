function segs = taskToSegments(cmd, model, approach_dist)
%taskToSegments command_type -> 运动段展开（参数与视觉接口统一）
%   segs = taskToSegments(cmd, model, approach_dist)
%   cmd : TaskCommand（遵循 TASK_COMMAND_INTERFACE v2.0）
%   model : createArmModel 输出
%   segs: struct 数组

    if nargin < 3 || isempty(approach_dist), approach_dist = 0.15; end
    ct = cmd.command_type;
    prm = optget(cmd, 'params', optget(cmd, 'motion_params', struct()));

    % 解析目标位姿 tgt：优先 pose_base（绝对物理位姿），若为 pose_camera 则通过手眼变换 objRelToAbs 转为基座绝对物理位姿
    if isfield(cmd,'selected_target') && isstruct(cmd.selected_target) && ~isempty(cmd.selected_target)
        if isfield(cmd.selected_target, 'pose_base') && ~isempty(cmd.selected_target.pose_base) ...
                && isfield(cmd.selected_target.pose_base, 'position')
            pb = cmd.selected_target.pose_base;
            tgt_base = projectTo2D(pb);
            if isfield(pb, 'frame_id') && (strcmpi(pb.frame_id, 'radar') || strcmpi(pb.frame_id, 'robot_base_ui'))
                tgt = [tgt_base(2), -tgt_base(1), tgt_base(3)];
            else
                tgt = tgt_base;
            end
        elseif isfield(cmd.selected_target, 'pose_camera') && ~isempty(cmd.selected_target.pose_camera)
            tgt_cam = projectTo2D(cmd.selected_target.pose_camera);   % [x,y,θ] rad（相对相机）
            % 利用当前机械臂几何模型与末端位姿调用 objRelToAbs 转换为空间基座绝对物理位姿
            if isstruct(model) && isfield(model, 'cfg') && isfield(model, 'DH')
                q_cur = optget(model, 'q', zeros(1, model.cfg.N));
                [~, pe] = planarFK_L(q_cur, model.DH, model.cfg.rod_offset_arr);
                th_cur = getEndEffectorAngle_L(q_cur, model.DH, model.cfg.rod_offset_arr);
                cam_pose = [pe(1), pe(2), th_cur];
                tgt = objRelToAbs(tgt_cam, cam_pose);   % [x,y,θ] 空间绝对物理位置
            else
                tgt = tgt_cam;
            end
        else
            tgt = [0, 0, 0];
        end
    else
        tgt = [0, 0, 0];   % 无目标（move_to 等）：位置朝向由 params 提供
    end

    A  = deg2rad(optget(prm,'alpha_deg', rad2deg(optget(prm,'alpha', 0.5))));   % 度→弧度
    n  = max(1, round(optget(prm,'joint_index', optget(prm,'n', 1))));
    if isstruct(model) && isfield(model,'cfg') && isfield(model.cfg,'N') && ~isempty(model.cfg.N)
        n = min(n, model.cfg.N);   % 本地按实际关节数截断
    end
    D  = optget(prm,'distance_m', optget(prm,'d', approach_dist));
    Th = deg2rad(optget(prm,'theta_deg', rad2deg(optget(prm,'theta', tgt(3)))));
    x  = optget(prm,'x', tgt(1));   y = optget(prm,'y', tgt(2));

    switch ct
        % ---- 末端位置段 ----
        case 'move_to'
            segs = mk('ee', [y, -x, tgt(3)], 0, 'MOVING_TO');  % 对齐坐标系：X_arm=y(前), Y_arm=-x(侧)

        case 'move_along'
            segs = mk('ee_relative', [], 0, 'MOVING_ALONG', 'dir', Th, 'dist', D);

        case 'move_for_pick'
            segs = mk('ee', [tgt(1), tgt(2), tgt(3)], 0, 'MOVING_TO_OBJECT');

        case 'move_for_place'
            if isfield(cmd,'destination') && isfield(cmd.destination,'pose_base') && ~isempty(cmd.destination.pose_base)
                dest = projectTo2D(cmd.destination.pose_base);
            else
                dest = tgt;   % 缺省：原目标位
            end
            segs = mk('ee', [dest(1), dest(2), dest(3)], 0, 'MOVING_TO_PLACE');

        % ---- 旋转段 ----
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
            segs = mk('reset', [], 0, 'RESETTING');

        otherwise
            error('taskToSegments:type', '未知 command_type: %s', ct);
    end
end

function s = mk(kind, target, gripper, name, varargin)
    s = struct('target', target, 'gripper', gripper, 'name', name, 'kind', kind, ...
        'joint', 0, 'delta', 0, 'angle', 0, 'dir', 0, 'dist', 0, 'alpha', 0);
    for i = 1:2:numel(varargin), s.(varargin{i}) = varargin{i+1}; end
end