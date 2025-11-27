function mm_test_closed_loop_tracking()
    clc; close all;
    fprintf('=== TEST 3: CLOSED-LOOP TRACKING (FL + PLANNER) ===\n');

    % Run the main controller (it will simulate and plot)
    mm_main_controller();

    % Re-run the core parts here to get data for error analysis
    [phys, ~] = mm_params_build_save_load('load', 'mm_params');
    traj      = mm_scenario_basic();
    ctrl      = mm_controller_init(phys, traj);

    dt_ref   = 0.01;
    [ctrl.ref, base_ref_all, arm_ref_all] = mm_build_eta_reference( ...
        phys, traj, traj.Tf, dt_ref);

    % Build initial state
    q0 = [traj.p0; 0; 0; traj.q0_arm(:)];
    eta_dot0 = zeros(5,1);
    X0 = [q0; eta_dot0];

    simdata.ctrl = ctrl;
    simdata.phys = phys;

    odefun = @(t,X) mm_closed_loop_dynamics(t, X, simdata);
    opts   = odeset('RelTol',1e-6,'AbsTol',1e-8);

    [t_sol, X_sol] = ode45(odefun, [0 traj.Tf], X0, opts);

    % Compute tracking error in η-space
    eta_traj     = zeros(5, numel(t_sol));
    eta_ref_traj = zeros(5, numel(t_sol));

    for k = 1:numel(t_sol)
        t   = t_sol(k);
        Xk  = X_sol(k,:)';
        qk  = Xk(1:8);
        eta_traj(:,k) = qk(4:8);

        [eta_ref_k, ~, ~] = mm_interp_eta_ref(ctrl.ref, t);
        eta_ref_traj(:,k) = eta_ref_k;
    end

    e_eta = eta_traj - eta_ref_traj;
    e_norm = vecnorm(e_eta, 2, 1);   % 2-norm over 5 DOFs

    figure;
    subplot(2,1,1);
    plot(t_sol, e_eta);
    grid on; title('\eta tracking error (each DOF)');
    legend('\theta_r','\theta_l','q_1','q_2','q_3');

    subplot(2,1,2);
    plot(t_sol, e_norm,'k-');
    grid on; title('||\eta - \eta_{ref}||_2');

    fprintf(' Max ||eta - eta_ref||_2 = %.4e\n', max(e_norm));
    fprintf('=== TEST 3 DONE ===\n');
end
