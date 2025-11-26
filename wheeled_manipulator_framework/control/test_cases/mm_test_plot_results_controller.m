function mm_test_plot_results_controller()
%MM_TEST_PLOT_RESULTS_CONTROLLER
%  Robust tests for mm_plot_results_controller:
%
%    1) Perfect-tracking synthetic trajectory (no dynamics integration)
%    2) Perturbed trajectory (non-zero errors)
%    3) Subsampled time grid (tests interpolation robustness)
%
%  Assumes the following are available on the path:
%    - mm_params_build_save_load
%    - mm_scenario_basic
%    - mm_controller_init(phys, traj, dt_ref)
%    - mm_plot_results_controller
%    - mm_fb_lin_tracker, mm_R_fun, mm_M1_fun, mm_C1_fun, mm_G1_fun, mm_B1_fun
%    - mm_interp_eta_ref (as separate function or accessible helper)

    fprintf("\n===============================================\n");
    fprintf("   PLOTTING & PIPELINE VISUALIZATION TESTS\n");
    fprintf("===============================================\n\n");

    %% 0) Load physical parameters and build controller / refs
    [phys, move] = mm_params_build_save_load('load', 'mm_params'); %#ok<ASGLU>
    fprintf("Loaded: mm_params from mm_params_build_save_load\n");

    Tf     = 5.0;
    dt_ref = 0.01;

    traj = mm_scenario_basic(Tf);

    [ctrl, base_ref_all, arm_ref_all] = mm_controller_init(phys, traj, dt_ref);
    ref = ctrl.ref;

    %% 1) Test with perfect-tracking synthetic trajectory
    fprintf("[PLOT] Perfect-tracking synthetic trajectory...\n");
    test_plot_perfect_tracking(ctrl, base_ref_all, arm_ref_all, phys);
    fprintf("  ✔ plotting succeeded.\n");

    %% 2) Test with perturbed / noisy trajectory
    fprintf("[PLOT] Perturbed trajectory (non-zero tracking error)...\n");
    test_plot_perturbed(ctrl, base_ref_all, arm_ref_all, phys);
    fprintf("  ✔ plotting succeeded.\n");

    %% 3) Test with subsampled time grid
    fprintf("[PLOT] Subsampled time grid (interpolation stress test)...\n");
    test_plot_subsampled(ctrl, base_ref_all, arm_ref_all, phys);
    fprintf("  ✔ plotting succeeded.\n");

    fprintf("\n✓ ALL PLOTTING TESTS PASSED (NO ERRORS THROWN) ✓\n\n");
end

%% ==================================================================
%% 1) Perfect-tracking synthetic trajectory
%%
%% Build X_sol directly from reference:
%%   - q_sol(:,1:3) from base_ref_all
%%   - q_sol(:,4:8) from ref.eta
%%   - eta_dot_sol  from ref.eta_dot
%% Then call mm_plot_results_controller and ensure it runs.
%% ==================================================================
function test_plot_perfect_tracking(ctrl, base_ref_all, arm_ref_all, phys)

    ref       = ctrl.ref;
    t_sol     = ref.t(:);                 % same grid as reference
    Nt        = numel(t_sol);

    % Build q_sol from reference data:
    % q = [x; y; theta_m; theta_r; theta_l; q1; q2; q3]
    q_sol = zeros(Nt, 8);
    q_sol(:,1) = base_ref_all(:,1);       % x
    q_sol(:,2) = base_ref_all(:,2);       % y
    q_sol(:,3) = base_ref_all(:,3);       % theta_m
    q_sol(:,4:8) = ref.eta;               % [theta_r theta_l q1 q2 q3]

    eta_dot_sol = ref.eta_dot;            % [theta_r_dot theta_l_dot q1dot q2dot q3dot]

    % Full state trajectory X_sol = [q_sol, eta_dot_sol]
    X_sol = [q_sol, eta_dot_sol];

    % Should run without error
    mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys);
end

%% ==================================================================
%% 2) Perturbed trajectory
%%
%% Same construction as above, but add small perturbations to q and eta_dot
%% to simulate tracking errors. Check that plotting still runs.
%% ==================================================================
function test_plot_perturbed(ctrl, base_ref_all, arm_ref_all, phys)

    ref   = ctrl.ref;
    t_sol = ref.t(:);
    Nt    = numel(t_sol);

    % Base state from reference
    q_sol = zeros(Nt, 8);
    q_sol(:,1) = base_ref_all(:,1);
    q_sol(:,2) = base_ref_all(:,2);
    q_sol(:,3) = base_ref_all(:,3);
    q_sol(:,4:8) = ref.eta;

    eta_dot_sol = ref.eta_dot;

    % Add small perturbations to simulate tracking errors
    rng(1);  % deterministic noise
    q_noise       = 0.01 * randn(size(q_sol));        % small pose noise
    eta_dot_noise = 0.01 * randn(size(eta_dot_sol));  % small velocity noise

    q_sol_pert       = q_sol       + q_noise;
    eta_dot_sol_pert = eta_dot_sol + eta_dot_noise;

    X_sol = [q_sol_pert, eta_dot_sol_pert];

    mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys);
end

%% ==================================================================
%% 3) Subsampled time grid
%%
%% Use a coarser time grid than ref.t to stress-test interpolation in
%% mm_plot_results_controller / mm_interp_eta_ref.
%% ==================================================================
function test_plot_subsampled(ctrl, base_ref_all, arm_ref_all, phys)

    ref     = ctrl.ref;
    t_ref   = ref.t(:);
    Nt_full = numel(t_ref);

    % Subsample every Nstep points
    Nstep = 5;   % coarse grid
    idx   = 1:Nstep:Nt_full;
    t_sol = t_ref(idx);
    Nt    = numel(t_sol);

    % Build q_sol & eta_dot_sol by sampling reference on subsampled grid
    base_ref_sub = base_ref_all(idx,:);   % [x y theta]
    eta_sub      = ref.eta(idx,:);        % [theta_r theta_l q1 q2 q3]
    eta_dot_sub  = ref.eta_dot(idx,:);

    q_sol = zeros(Nt, 8);
    q_sol(:,1) = base_ref_sub(:,1);       % x
    q_sol(:,2) = base_ref_sub(:,2);       % y
    q_sol(:,3) = base_ref_sub(:,3);       % theta_m
    q_sol(:,4:8) = eta_sub;

    eta_dot_sol = eta_dot_sub;

    X_sol = [q_sol, eta_dot_sol];

    mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys);
end
