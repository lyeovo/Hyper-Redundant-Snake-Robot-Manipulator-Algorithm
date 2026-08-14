%% QA analysis: runLArmIK_2D_Simple.m vs runLArmIK_2D.m
% NOTE: run with cwd = project dir. ASCII only.
close all force;
results = {};

try
    n1 = nargin('runLArmIK_2D_Simple');
    results{end+1} = sprintf('Simple parse OK, nargin=%d', n1);
catch e
    results{end+1} = ['Simple PARSE/LOAD ERROR: ' e.message];
end

try
    n2 = nargin('runLArmIK_2D');
    results{end+1} = sprintf('Full parse OK, nargin=%d', n2);
catch e
    results{end+1} = ['Full PARSE/LOAD ERROR: ' e.message];
end

% Which solveIK wins the name conflict?
try
    p = which('solveIK');
    results{end+1} = ['which solveIK -> ' char(p)];
catch e
    results{end+1} = ['which solveIK ERROR: ' e.message];
end

try
    w = which('maybeFillS');
    if isempty(w), results{end+1} = 'maybeFillS NOT DEFINED (used at line 725-727)';
    else, results{end+1} = ['maybeFillS -> ' char(w)]; end
catch e
    results{end+1} = ['maybeFillS check ERROR: ' e.message];
end

try
    w = which('maybeFill_s');
    if isempty(w), results{end+1} = 'maybeFill_s NOT DEFINED (used at lines 2161-2164)';
    else, results{end+1} = ['maybeFill_s -> ' char(w)]; end
catch e
    results{end+1} = ['maybeFill_s check ERROR: ' e.message];
end

try
    w = which('calcCostSimple');
    if isempty(w), results{end+1} = 'calcCostSimple NOT FOUND';
    else, results{end+1} = ['calcCostSimple -> ' char(w)]; end
catch e
    results{end+1} = ['calcCostSimple check ERROR: ' e.message];
end

% ---- Test Simple runLArmIK_2D_Simple (fast, no figures) ----
try
    p = struct();
    p.N = 4; p.L_seg = 1.0;
    p.q_min = -pi; p.q_max = pi;
    p.X_target = [1.5, 0.5]; p.theta_end_target = 0;
    p.q_init = zeros(1,4); p.rod_offset_arr = zeros(1,4);
    p.max_iter = 200; p.lambda_m = 1e-5; p.dq_step_max = 0.15;
    p.obs = []; p.obs_lines = {}; p.rho0 = 0.05; p.safe_margin = 0.01;
    p.w_pos = 1.0; p.w_ang = 0.3; p.w_obs = 0.5; p.w_var = 0.01; p.w_acc = 0.1;
    p.momentum_beta = 0.85; p.barrier_eps = 0.001;
    p.m_arr = zeros(1,4); p.sig0_arr = [1 0.06 0.06 0.06]; p.tau_arr = [1 0.7 0.7 0.7];
    p.use_global_search = 0;
    q = runLArmIK_2D_Simple(p, 100);
    results{end+1} = ['Simple solver ran OK, q=' mat2str(q,3)];
catch e
    results{end+1} = ['Simple solver ERROR: ' e.message];
end

% ---- Test full runLArmIK_2D with small max_iter, mode 0 ----
try
    p2 = struct();
    p2.N = 4; p2.L_seg = 1.0;
    p2.q_min = -pi; p2.q_max = pi;
    p2.X_target = [1.5, 0.5]; p2.theta_end_target = 0;
    p2.q_init = zeros(1,4); p2.rod_offset_arr = zeros(1,4);
    p2.max_iter = 5; p2.lambda_m = 1e-5; p2.dq_step_max = 0.15; p2.dq_step_min = -0.15;
    p2.kappa = 3.2e-5;
    p2.lambda_damp = 0.01; p2.gamma_soft = 1000;
    p2.gamma_ang_base = 0.01; p2.gamma_ang_peak = 10; p2.sigma_weight = 0.1;
    p2.m_arr = zeros(1,4); p2.sig0_arr = [1 0.06 0.06 0.06];
    p2.tau_arr = [1 0.7 0.7 0.7]; p2.sig_min2 = 0.5;
    p2.lambda_part = 1e-3; p2.w_part = ones(1,4);
    p2.lambda_activate = 5e-3; p2.lambda_motor = 1e-4;
    p2.obs = []; p2.obs_lines = {}; p2.rho0 = 0.05; p2.safe_margin = 0.01;
    p2.use_momentum = true; p2.momentum_beta = 0.9;
    p2.barrier_C = 200; p2.barrier_eps = 0.001;
    p2.dt_base = 0.08; p2.rho_critical = 0.15;
    p2.lambdaM = 0.1; p2.plot_pad = 0.15; p2.bottom_pad = 0.15;
    p2.use_global_search = 0;
    p2.dq_stall_thresh = 1e-5; p2.stall_count_max = 8;
    q2 = runLArmIK_2D(p2, 100);
    results{end+1} = ['Full solver ran OK, q=' mat2str(q2,3)];
catch e
    results{end+1} = ['Full solver ERROR: ' e.message];
end

% ---- Test missing-lambdaM path: refinePath called with solveIK params ----
try
    results{end+1} = 'Checking if solveIK(params) contains lambdaM field...';
    gp = GlobalParams();
    results{end+1} = sprintf('GlobalParams has lambdaM=%g', gp.lambdaM);
    % emulate solveIK param build: does it set lambdaM? -> inspect file text
catch e
    results{end+1} = ['lambdaM check ERROR: ' e.message];
end

fprintf('================ QA RESULTS ================\n');
for i = 1:numel(results)
    fprintf('[%02d] %s\n', i, results{i});
end
fprintf('============================================\n');
