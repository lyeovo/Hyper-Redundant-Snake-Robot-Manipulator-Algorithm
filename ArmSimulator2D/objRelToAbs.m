function P = objRelToAbs(obj_rel, cam_pose)
%objRelToAbs 相机系相对位姿 → 基座系绝对位姿（平面 2D 刚性变换）
%   P = objRelToAbs(obj_rel, cam_pose)
%   obj_rel : [x,y,θ] 物体相对相机的位姿（米 / rad）
%   cam_pose: [x,y,θ] 相机在基座系的位姿（米 / rad，来自手眼标定/已知相机安装）
%   P       : [x,y,θ] 物体在基座系的绝对位姿
%   p_base = R(θ_cam)·p_c + t_cam；θ_base = θ_c + θ_cam
    th = cam_pose(3);
    x = cam_pose(1) + obj_rel(1)*cos(th) - obj_rel(2)*sin(th);
    y = cam_pose(2) + obj_rel(1)*sin(th) + obj_rel(2)*cos(th);
    P = [x, y, wrapAngle(obj_rel(3) + th)];
end
