function result = mm_main_controller(showPlots)
%MM_MAIN_CONTROLLER
%  Top-level script for mobile manipulator closed-loop simulation.
%
%  Pipeline:
%    1) Add paths
%    2) Load physical parameters
%    3) Define scenario (task level)
%    4) Initialize controller (gains only, no planning)
%    5) Build η-reference offline (planner)
%    6) Simulate closed-loop dynamics (feedback linearization)
%    7) Post-process: control efforts + control energy
%    8) Plot all results

    if nargin == 0, showPlots = true; end
    fprintf('▶ Starting mobile manipulator tracking simulation...\n');

    %% --------------------------------------------------------------
    % 1) Paths
    %% --------------------------------------------------------------
    controllers = fileparts(mfilename('fullpath'));
    framework = fileparts(fileparts(controllers));
    addpath(genpath(controllers));
    addpath(fullfile(framework,'models','data'), fullfile(framework,'models','dynamics_library'));

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
    % 4) Controller configuration (gains only, no planning here)
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
        traj.p0_B;        % [x_b; y_b; theta_m] (mount B)
        0; 0;             % wheel angles [theta_r; theta_l]
        traj.q0_arm(:)    % arm joints [q1; q2; q3]
    ];

    eta_dot0 = zeros(5,1);            % [theta_r_dot; theta_l_dot; q1dot; q2dot; q3dot]
    X0       = [q0; eta_dot0];        % 13 x 1

    %% --------------------------------------------------------------
    % 7) Pack simulation data for the ODE
    %% --------------------------------------------------------------
    simdata.ctrl = ctrl;
    simdata.phys = phys;
    % No disturbance  → old behavior
    simdata.dist_fun = [];  % or just don't set it
    
    % % Example 1: constant bias on joint 2
    % simdata.dist_fun = @(t, q, eta_dot) [0; 0; 0.5; 0; 0];
    simdata.dist_fun = @(t, q, eta_dot) [0.5; 0.5; 0.05; 0.05; 0.05];

    % % Example 2: sinusoidal torque on wheel 1
    % simdata.dist_fun = @(t, q, eta_dot) [0.2*sin(0.5*t); 0; 0; 0; 0];
    % 
    % % Example 3: small random disturbance on all DOFs
    % simdata.dist_fun = @(t, q, eta_dot) 0.05*randn(5,1);

    %% --------------------------------------------------------------
    % 8) Closed-loop ODE integration
    %% --------------------------------------------------------------
    odefun = @(t,X) mm_closed_loop_dynamics(t, X, simdata);
    opts   = odeset('RelTol',1e-6,'AbsTol',1e-8);

    fprintf('▶ Starting ODE45 integration...\n');
    [t_sol, X_sol] = ode45(odefun, [0 traj.Tf], X0, opts);

    %% --------------------------------------------------------------
    % 9) Post-process: control efforts and control energy
    %% --------------------------------------------------------------
    [tau_sol, E_sol] = mm_postprocess_control_energy(t_sol, X_sol, ctrl, phys);

    %% --------------------------------------------------------------
    % 10) Visualization
    %% --------------------------------------------------------------
    result = struct('time',t_sol,'state',X_sol,'torque',tau_sol,'work',E_sol);
    assert(abs(t_sol(end)-traj.Tf)<1e-10 && all(isfinite([X_sol tau_sol E_sol]),'all'), ...
        'Legacy integration did not produce a complete finite trajectory.');
    if showPlots
        mm_plot_results_controller(t_sol,X_sol,ctrl,base_ref_all,arm_ref_all,phys,tau_sol,E_sol,'light');
    end

    fprintf('✅ Simulation finished.\n');
end
