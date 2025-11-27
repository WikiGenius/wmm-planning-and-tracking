function mm_main_controller()
%MM_MAIN_CONTROLLER
%  Top-level script for mobile manipulator closed-loop simulation.
%  Currently:
%    - Uses precomputed η-reference (no online geometric planner).
%    - Tracks η, η̇, η̈ using feedback linearization on reduced dynamics.
%
%  Ready for future extension:
%    - Online planner: mm_traj_planner_base_arm
%    - Kinematic layer: mm_kin_ctrl_base_kanayama, mm_kin_ctrl_arm_pd

    clc; close all;
    fprintf('▶ Starting mobile manipulator tracking simulation...\n');

    %% --------------------------------------------------------------
    % 1) Paths
    %% --------------------------------------------------------------
    project_root = fileparts(mfilename('fullpath'));
    addpath(fullfile(project_root, 'dynamics_library'));
    addpath(fullfile(project_root, 'controllers'));
    addpath(fullfile(project_root, 'data'));
    addpath(fullfile(project_root, 'transformations'));
    fprintf('💠 Paths added.\n');

    %% --------------------------------------------------------------
    % 2) Load physical parameters
    %% --------------------------------------------------------------
    [phys, ~] = mm_params_build_save_load('load', 'mm_params');
    fprintf('Loaded: mm_params.mat\n');

    %% --------------------------------------------------------------
    % 3) Scenario (task specification)
    %% --------------------------------------------------------------
    traj = mm_scenario_basic(phys);

    %% --------------------------------------------------------------
    % 4) Controller configuration (no planning inside)
    %% --------------------------------------------------------------
    ctrl = mm_controller_init(phys, traj);

    %% --------------------------------------------------------------
    % 5) PRECOMPUTE η-REFERENCE (offline planner, for FL / plotting)
    %% --------------------------------------------------------------
    dt_ref = 0.01;
    [ctrl.ref, base_ref_all, arm_ref_all] = mm_build_eta_reference( ...
        phys, traj, traj.Tf, dt_ref);

    %% --------------------------------------------------------------
    % 6) Initial state [q ; η̇]
    %% --------------------------------------------------------------
    q0 = [
        traj.p0_B;          % [x_b; y_b; theta_m]
        0; 0;             % wheel angles [theta_r; theta_l]
        traj.q0_arm(:)    % arm joints [q1; q2; q3]
    ];

    eta_dot0 = zeros(5,1);   % [theta_r_dot; theta_l_dot; q1dot; q2dot; q3dot]
    X0       = [q0; eta_dot0];

    %% --------------------------------------------------------------
    % 7) Pack simulation data
    %% --------------------------------------------------------------
    simdata.ctrl = ctrl;
    simdata.phys = phys;

    %% --------------------------------------------------------------
    % 8) Closed-loop ODE integration
    %% --------------------------------------------------------------
    odefun = @(t,X) mm_closed_loop_dynamics(t, X, simdata);
    opts   = odeset('RelTol',1e-6,'AbsTol',1e-8);

    fprintf('▶ Starting ODE45 integration...\n');
    [t_sol, X_sol] = ode45(odefun, [0 traj.Tf], X0, opts);

    %% --------------------------------------------------------------
    % 9) Visualization
    %% --------------------------------------------------------------
    mm_plot_results_controller( ...
        t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys);

    fprintf('✅ Simulation finished.\n');
end
