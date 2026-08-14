function pose2d = projectTo2D(pose3d)
%projectTo2D 3D 位姿 → 2D 平面位姿（方案 D13）
%   pose2d = projectTo2D(pose3d)
%   pose3d: 视觉模块输出（TaskCommand.selected_target.pose_camera 结构）
%           .position.{x,y,z}（米）+ .orientation_quat.{x,y,z,w} 或 .orientation_euler.{roll,pitch,yaw}
%   工作平面 = 水平面（robot_base X-Y，Z 垂直），θ = yaw（绕垂直轴 Z 的四元数偏航角）
%   手眼标定（T_base_camera）前，相机系坐标直接作为仿真基座系使用（仿真模式）
    if isfield(pose3d, 'position') && ~isempty(pose3d.position)
        x = pose3d.position.x;  y = pose3d.position.y;
    elseif isfield(pose3d, 'position_array')
        p = pose3d.position_array;  x = p(1);  y = p(2);
    else
        error('projectTo2D:pose', '无法解析位置字段');
    end
    if isfield(pose3d, 'orientation_euler') && ~isempty(pose3d.orientation_euler)
        yaw = pose3d.orientation_euler.yaw;
    elseif isfield(pose3d, 'orientation_quat') && ~isempty(pose3d.orientation_quat)
        q = pose3d.orientation_quat;
        yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
    else
        yaw = 0;
    end
    pose2d = [x, y, yaw];
end
