function q = inverseKinPose(model, pose, opts)
%inverseKinPose 末端位姿 → 关节角反解（内部用动量法）
%   q = inverseKinPose(model, pose)
%   q = inverseKinPose(model, pose, opts)    % opts 同 method_momentum（max_iter 等）
%   pose: [x, y, θ]；失败返回 []（调用方转 error_code=5）
%   用途：simulateMotion 的 q0 若为末端位姿，先反解得到关节角
    if nargin < 3 || isempty(opts), opts = struct(); end
    opts = setDefault(opts, 'max_iter', model.cfg.max_iter);
    opts = setDefault(opts, 'snapshot_m', inf);
    m2 = model;
    m2.cfg.X_target = pose(1:2);
    m2.cfg.theta_target = pose(3);
    info = method_momentum(m2, model.cfg.q_init, pose, opts);
    if info.success
        q = info.q_final;
    else
        q = [];
    end
end

function o = setDefault(o, field, val)
    if ~isfield(o, field) || isempty(o.(field)), o.(field) = val; end
end
