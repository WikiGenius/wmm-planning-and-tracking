function mm_test_main_controller()
%MM_TEST_MAIN_CONTROLLER
%  Integration + sanity tests for:
%     - mm_scenario_basic
%     - mm_controller_init
%     - mm_build_eta_reference
%     - mm_interp_eta_ref
%     - mm_closed_loop_dynamics
%     - mm_forward_dynamics_reduced
%
%  This script does NOT plot. It just prints status + uses assert().
%
%  Expected state structure:
%     X = [q; eta_dot]  (13x1)
%     q       = [x y theta_m theta_r theta_l q1 q2 q3]
%     eta_dot = [theta_r_dot theta_l_dot q1dot q2dot q3dot]

    clc;
    fprintf('===============================================\n');
    fprintf('   MAIN CONTROLLER INTEGRATION TESTS\n');
    fprintf('===============================================\n');

    %% --------------------------------------------------------------
    %  0) Paths
    %% --------------------------------------------------------------
    test_root = fileparts(mfilename('fullpath'));
    addpath(fullfile(test_root, 'dynamics_library'));
    addpath(fullfile(test_root, 'controllers'));
    addpath(fullfile(test_root, 'data'));

    %% --------------------------------------------------------------
    %  1) Load physical parameters
    %% --------------------------------------------------------------
    [phys, ~] = mm_params_build_save_load('load', 'mm_params');
    fprintf('Loaded: mm_params.mat\n');

    %% --------------------------------------------------------------
    %  2) Build scenario + controller + η reference
    %% --------------------------------------------------------------
    traj = mm_scenario_basic();               % all task settings
    ctrl = mm_controller_init(phys, traj);    % gains only

    % Offline reference for plotting & feedforward
    dt_ref = 0.01;
    [ref, base_ref_all, arm_ref_all] = mm_build_eta_reference( ...
        phys, traj, traj.Tf, dt_ref);
    ctrl.ref = ref;                           % attach to ctrl for reuse

    %% --------------------------------------------------------------
    %  3) Test: mm_interp_eta_ref correctness
    %% --------------------------------------------------------------
    fprintf('[REF] Testing mm_interp_eta_ref...\n');

    % 3.1 Exact at grid points
    k_mid = floor(numel(ref.t)/2);
    t_mid = ref.t(k_mid);
    [eta_mid, eta_dot_mid, eta_ddot_mid] = mm_interp_eta_ref(ref, t_mid);

    assert( max(abs(eta_mid      - ref.eta(k_mid,:).'))      < 1e-10, ...
        'mm_interp_eta_ref: eta mismatch at grid point.');
    assert( max(abs(eta_dot_mid  - ref.eta_dot(k_mid,:).'))  < 1e-10, ...
        'mm_interp_eta_ref: eta_dot mismatch at grid point.');
    assert( max(abs(eta_ddot_mid - ref.eta_ddot(k_mid,:).')) < 1e-8, ...
        'mm_interp_eta_ref: eta_ddot mismatch at grid point.');

    fprintf('  ✔ mm_interp_eta_ref exact at grid points.\n');

    % 3.2 Clamp behaviour outside [t0, Tf]
    t_before = ref.t(1)  - 1.0;
    t_after  = ref.t(end)+ 1.0;

    [eta_b, eta_dot_b, eta_ddot_b] = mm_interp_eta_ref(ref, t_before);
    [eta_a, eta_dot_a, eta_ddot_a] = mm_interp_eta_ref(ref, t_after);

    assert( max(abs(eta_b      - ref.eta(1,:).'))      < 1e-10, ...
        'mm_interp_eta_ref: eta should clamp at start.');
    assert( max(abs(eta_dot_b  - ref.eta_dot(1,:).'))  < 1e-10, ...
        'mm_interp_eta_ref: eta_dot should clamp at start.');
    assert( max(abs(eta_ddot_b - ref.eta_ddot(1,:).')) < 1e-8, ...
        'mm_interp_eta_ref: eta_ddot should clamp at start.');

    assert( max(abs(eta_a      - ref.eta(end,:).'))      < 1e-10, ...
        'mm_interp_eta_ref: eta should clamp at end.');
    assert( max(abs(eta_dot_a  - ref.eta_dot(end,:).'))  < 1e-10, ...
        'mm_interp_eta_ref: eta_dot should clamp at end.');
    assert( max(abs(eta_ddot_a - ref.eta_ddot(end,:).')) < 1e-8, ...
        'mm_interp_eta_ref: eta_ddot should clamp at end.');

    fprintf('  ✔ mm_interp_eta_ref clamps outside time interval correctly.\n\n');

    %% --------------------------------------------------------------
    %  4) Closed-loop ODE run (NO plotting)
    %% --------------------------------------------------------------
    fprintf('[MAIN] Basic closed-loop run with ODE45...\n');

    % Initial state: use scenario base + arm, wheels at 0, eta_dot=0
    q0 = [
        traj.p0(:);          % [x0; y0; theta0]
        0; 0;                % theta_r, theta_l
        traj.q0_arm(:)       % arm joints
    ];
    eta_dot0 = zeros(5,1);
    X0       = [q0; eta_dot0];

    % Pack simdata exactly as in mm_main_controller
    simdata.ctrl      = ctrl;
    simdata.gains_eta = ctrl.gains.eta;
    simdata.phys      = phys;

    odefun = @(t,X) mm_closed_loop_dynamics(t, X, simdata);
    opts   = odeset('RelTol',1e-6, 'AbsTol',1e-8);

    [t_sol, X_sol] = ode45(odefun, [0 traj.Tf], X0, opts);

    % 4.1 No runtime failure already implicit. Check dimensions & NaNs:
    assert(size(X_sol,2) == 13, ...
        'State X_sol must have 13 columns ([q; eta_dot]).');
    assert(all(all(~isnan(X_sol))), 'X_sol contains NaNs.');
    assert(all(all(~isinf(X_sol))), 'X_sol contains Infs.');

    fprintf('  ✔ mm_closed_loop_dynamics + ODE45 ran without error.\n');
    fprintf('  ✔ state dimension & NaN/Inf checks passed.\n');

    % 4.2 Final time consistency
    Tf     = traj.Tf;
    t_end  = t_sol(end);
    assert(abs(t_end - Tf) < 1e-6, ...
        'Final time inconsistency: t_end=%.6f, Tf=%.6f', t_end, Tf);
    fprintf('  ✔ final time consistency passed (Tf=%.3f).\n', Tf);

    %% --------------------------------------------------------------
    %  5) Base & arm goal convergence
    %% --------------------------------------------------------------
    q_final = X_sol(end,1:8).';     % final q

    % Base pose error (x,y,theta)
    x_end  = q_final(1);
    y_end  = q_final(2);
    th_end = q_final(3);

    base_pos_err   = norm([x_end; y_end] - traj.pf(1:2));
    th_err         = wrapToPi(th_end - traj.pf(3));
    base_pose_err  = norm([base_pos_err; th_err]);

    % Arm pose error (joint positions only)
    q_arm_end  = q_final(6:8);
    arm_err    = norm(q_arm_end - traj.qf_arm);

    % Tolerances (you can tighten later)
    base_tol = 5e-2;    % 5 cm + ~0.05 rad combined
    arm_tol  = 5e-2;    % 0.05 rad per joint

    fprintf('[MAIN] Final base pose error:  %.3f (tol %.3f)\n', base_pose_err, base_tol);
    fprintf('[MAIN] Final arm pose  error:  %.3f (tol %.3f)\n', arm_err, arm_tol);

    assert(base_pose_err < base_tol, ...
        'Final base pose error too large (%.3f > %.3f).', base_pose_err, base_tol);
    assert(arm_err < arm_tol, ...
        'Final arm pose error too large  (%.3f > %.3f).', arm_err, arm_tol);

    fprintf('  ✔ base & arm goal convergence passed.\n');

    %% --------------------------------------------------------------
    %  6) Basic tracking quality checks
    %% --------------------------------------------------------------
    fprintf('[MAIN] Tracking quality checks...\n');

    % Rebuild reference on solver grid for base pose for comparison
    t_ref = ref.t(:);
    x_ref = interp1(t_ref, base_ref_all(:,1), t_sol, 'pchip');
    y_ref = interp1(t_ref, base_ref_all(:,2), t_sol, 'pchip');

    x_traj = X_sol(:,1);
    y_traj = X_sol(:,2);

    % Mean absolute tracking error for base XY
    mean_xy_err = mean( sqrt((x_traj - x_ref).^2 + (y_traj - y_ref).^2) );
    assert(mean_xy_err < 0.15, ...
        'Mean XY tracking error too high (%.3f > 0.15).', mean_xy_err);

    fprintf('  ✔ mean XY tracking error OK (%.3f).\n', mean_xy_err);

    %% --------------------------------------------------------------
    %  7) Robustness test: perturbed initial pose
    %% --------------------------------------------------------------
    fprintf('[ROBUST] Perturbed initial base pose test...\n');

    % Small perturbation in base pose (10 cm, 10 deg)
    delta_q0 = zeros(8,1);
    delta_q0(1:2) = [0.1; -0.1];      % 10 cm offset
    delta_q0(3)   = deg2rad(10);      % 10 degrees

    q0_pert   = q0 + delta_q0;
    X0_pert   = [q0_pert; eta_dot0];

    [t_sol2, X_sol2] = ode45(odefun, [0 traj.Tf], X0_pert, opts);

    q_final2 = X_sol2(end,1:8).';

    x_end2  = q_final2(1);
    y_end2  = q_final2(2);
    th_end2 = q_final2(3);

    base_pos_err2   = norm([x_end2; y_end2] - traj.pf(1:2));
    th_err2         = wrapToPi(th_end2 - traj.pf(3));
    base_pose_err2  = norm([base_pos_err2; th_err2]);

    fprintf('  [ROBUST] Final base pose error (perturbed init): %.3f\n', base_pose_err2);
    assert(base_pose_err2 < 0.10, ...
        'Perturbed initial condition: base pose did not converge (%.3f > 0.10).', ...
        base_pose_err2);

    fprintf('  ✔ robustness to small initial perturbations passed.\n');

    %% --------------------------------------------------------------
    %  DONE
    %% --------------------------------------------------------------
    fprintf('\n✓ ALL MAIN CONTROLLER TESTS PASSED ✓\n');
end

% =====================================================================
% Local helper (MATLAB < R2015 has no built-in wrapToPi; if you already
% have Mapping Toolbox, you can remove this and use wrapToPi directly).
% =====================================================================
function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
