%% runLArmIK_2D_Simple.m -- 【已废弃】薄包装：转调 ArmSimulator2D 新接口
%  ======================================================================
%  新核心：ArmSimulator2D/（createArmModel + simulateMotion，5 种方法 + auto 调度）
%  本文件仅保留旧调用兼容，新代码请使用：
%      model = createArmModel(params);
%      info  = simulateMotion(model, method, q0, target, ...);
%  完整设计见 文档/实现方案.md；旧完整版 runLArmIK_2D.m 亦已废弃（供 GUI 迁移过渡）。
%  ======================================================================
function q_final = runLArmIK_2D_Simple(params, print_step)
    if nargin < 2 || isempty(print_step), print_step = 50; end
    % 旧字段兼容：theta_end_target → theta_target
    if isfield(params, 'theta_end_target')
        params.theta_target = params.theta_end_target;
    end
    model = createArmModel(params);
    target = [model.cfg.X_target, model.cfg.theta_target];
    if isfield(params, 'q_init') && ~isempty(params.q_init)
        q0 = params.q_init;
    else
        q0 = model.cfg.q_init;
    end
    info = simulateMotion(model, 'auto', q0, target, 'Snapshot', print_step);
    q_final = info.q_final;
    if ~info.success
        warning('runLArmIK_2D_Simple:deprecated', ...
            '未收敛 error_code=%d（pos=%.3f ang=%.3f）——新接口 simulateMotion 返回完整 info', ...
            info.error_code, info.dist_end, info.err_ang);
    end
end
