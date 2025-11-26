function mm_test_planner_suite(do_plots)
%MM_TEST_PLANNER_SUITE
%  Full test suite for the trajectory planner modules:
%
%    ✓ quintic_bc
%    ✓ mm_planner_arm_quintic
%    ✓ mm_planner_base_se2_geodesic
%    ✓ mm_base_vel_to_wheel_rates
%    ✓ mm_build_eta_reference
%    ✓ mm_controller_init
%
%  Optional:
%    do_plots : if true (default), shows diagnostic visualizations
%
%  Usage:
%    mm_test_planner_suite();
%    mm_test_planner_suite(false);  % no plots

    if nargin < 1
        do_plots = true;
    end

    fprintf("\n===============================================\n");
    fprintf("   MOBILE MANIPULATOR TRAJECTORY TEST SUITE\n");
    fprintf("===============================================\n\n");

    %% Load physical parameters
    S = load('mm_params.mat');   % should contain phys
    phys = S.phys;

    Tf   = 5.0;

    % Use external scenario definition:
    % mm_scenario_basic.m must be on the MATLAB path.
    traj = mm_scenario_basic();

    %% ----------------------------------------------------------
    % 1) Test quintic_bc boundary constraints
    %% ----------------------------------------------------------
    fprintf("[TEST] quintic_bc...\n");
    test_quintic_bc();
    fprintf("  ✔ quintic_bc PASSED.\n\n");

    %% ----------------------------------------------------------
    % 2) Test arm planner (boundary + continuity)
    %% ----------------------------------------------------------
    fprintf("[TEST] mm_planner_arm_quintic...\n");
    test_planner_arm_quintic(Tf, traj, do_plots);
    fprintf("  ✔ arm planner PASSED.\n\n");

    %% ----------------------------------------------------------
    % 3) Test base SE(2) planner (pose + smoothness)
    %% ----------------------------------------------------------
    fprintf("[TEST] mm_planner_base_se2_geodesic...\n");
    test_planner_base_se2_geodesic(Tf, traj, do_plots);
    fprintf("  ✔ SE(2) base planner PASSED.\n\n");

    %% ----------------------------------------------------------
    % 4) Test wheel velocity converter
    %% ----------------------------------------------------------
    fprintf("[TEST] mm_base_vel_to_wheel_rates...\n");
    test_wheel_converter(phys);
    fprintf("  ✔ wheel converter PASSED.\n\n");

    %% ----------------------------------------------------------
    % 5) Test η reference builder (consistency with planner)
    %% ----------------------------------------------------------
    fprintf("[TEST] mm_build_eta_reference...\n");
    test_eta_reference(phys, traj, Tf, do_plots);
    fprintf("  ✔ η reference builder PASSED.\n\n");

    %% ----------------------------------------------------------
    % 6) Test controller init consistency
    %% ----------------------------------------------------------
    fprintf("[TEST] mm_controller_init...\n");
    test_controller_init(phys, Tf);
    fprintf("  ✔ controller init PASSED.\n\n");

    fprintf("✓ ALL TESTS PASSED SUCCESSFULLY ✓\n\n");
end

%% ==================================================================
%% 1) Test quintic_bc (scalar trajectory)
%% ==================================================================
function test_quintic_bc()
    q0   = 1;
    qf   = 3;
    qd0  = 0.1;
    qdf  = -0.2;
    qdd0 = 0.0;
    qddf = 0.05;
    Tf   = 5;

    % Check both t = 0 and t = Tf
    [q0e, qd0e, qdd0e] = quintic_bc(0, Tf, q0, qf, qd0, qdf, qdd0, qddf);
    assert(abs(q0e   - q0)   < 1e-8);
    assert(abs(qd0e  - qd0)  < 1e-8);
    assert(abs(qdd0e - qdd0) < 1e-8);

    [qfe, qdfe, qddfe] = quintic_bc(Tf, Tf, q0, qf, qd0, qdf, qdd0, qddf);
    assert(abs(qfe   - qf)   < 1e-8);
    assert(abs(qdfe  - qdf)  < 1e-8);
    assert(abs(qddfe - qddf) < 1e-8);
end

%% ==================================================================
%% 2) Arm planner test (q, qdot, qddot)
%% ==================================================================
function test_planner_arm_quintic(Tf, traj, do_plots)
    % Check boundary at t = 0
    ref0 = mm_planner_arm_quintic(0, Tf, traj);
    assert(norm(ref0.q    - traj.q0_arm)  < 1e-8);
    assert(norm(ref0.qdot - traj.qd0_arm) < 1e-8);

    % And at t = Tf
    refF = mm_planner_arm_quintic(Tf, Tf, traj);
    assert(norm(refF.q    - traj.qf_arm)  < 1e-8);
    assert(norm(refF.qdot - traj.qdf_arm) < 1e-8);

    % Optional visualization: joint trajectories
    if do_plots
        Nt = 200;
        t_grid = linspace(0, Tf, Nt);
        Q    = zeros(Nt, 3);
        Qd   = zeros(Nt, 3);
        Qdd  = zeros(Nt, 3);

        for k = 1:Nt
            ref = mm_planner_arm_quintic(t_grid(k), Tf, traj);
            Q(k,:)   = ref.q(:).';
            Qd(k,:)  = ref.qdot(:).';
            Qdd(k,:) = ref.qddot(:).';
        end

        figure;
        subplot(3,1,1);
        plot(t_grid, Q, 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('q_i [rad]');
        title('Arm joint angles (planner)');
        legend('q_1','q_2','q_3','Location','best');

        subplot(3,1,2);
        plot(t_grid, Qd, 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('$\dot{q}_i [rad/s]$', Interpreter='latex');
        title('Arm joint velocities (planner)');
        legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','Location','best');

        subplot(3,1,3);
        plot(t_grid, Qdd, 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('$\ddot{q}_i [rad/s^2]$', Interpreter='latex');
        title('Arm joint accelerations (planner)');
        legend('\ddot{q}_1','\ddot{q}_2','\ddot{q}_3','Location','best');
    end
end

%% ==================================================================
%% 3) SE(2) base planner test (pose, v, w, and accelerations)
%% ==================================================================
function test_planner_base_se2_geodesic(Tf, traj, do_plots)
    % Endpoints must match exactly p0, pf (by design)
    ref0 = mm_planner_base_se2_geodesic(0, traj);
    assert(norm([ref0.x; ref0.y; ref0.theta] - traj.p0) < 1e-6);

    refF = mm_planner_base_se2_geodesic(Tf, traj);
    assert(norm([refF.x; refF.y; refF.theta] - traj.pf) < 1e-6);

    % Sample the path to check smoothness & visualise planar trajectory
    Nt = 200;
    t_grid = linspace(0, Tf, Nt);
    xb   = zeros(Nt,1);
    yb   = zeros(Nt,1);
    thb  = zeros(Nt,1);
    v    = zeros(Nt,1);
    w    = zeros(Nt,1);
    xdd  = zeros(Nt,1);
    ydd  = zeros(Nt,1);
    thdd = zeros(Nt,1);

    for k = 1:Nt
        r = mm_planner_base_se2_geodesic(t_grid(k), traj);
        xb(k)   = r.x;
        yb(k)   = r.y;
        thb(k)  = r.theta;
        v(k)    = r.v_r;
        w(k)    = r.omega_r;
        xdd(k)  = r.xddot;
        ydd(k)  = r.yddot;
        thdd(k) = r.thetaddot;
    end

    % Numerical smoothness check: finite-difference derivative of v, w
    dv = diff(v) ./ diff(t_grid(:));
    dw = diff(w) ./ diff(t_grid(:));
    % Not strict, just to catch crazy spikes:
    assert(all(abs(dv) < 100));   % arbitrary but finite
    assert(all(abs(dw) < 200));

    if do_plots
        % XY planar path
        figure;
        plot(xb, yb, 'LineWidth', 1.8); hold on; grid on;
        plot(traj.p0(1), traj.p0(2), 'go', 'MarkerFaceColor','g');
        plot(traj.pf(1), traj.pf(2), 'ro', 'MarkerFaceColor','r');
        axis equal;
        xlabel('x_b [m]');
        ylabel('y_b [m]');
        title('Base planar trajectory (SE(2) geodesic)');
        legend('path','start','end','Location','best');

        % theta(t), v(t), omega(t)
        figure;
        subplot(3,1,1);
        plot(t_grid, thb, 'LineWidth', 1.5);
        grid on;
        ylabel('$\theta_b [rad]$', Interpreter='latex');
        title('Base heading');

        subplot(3,1,2);
        plot(t_grid, v, 'LineWidth', 1.5);
        grid on;
        ylabel('v_r [m/s]');
        title('Forward speed');

        subplot(3,1,3);
        plot(t_grid, w, 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('$\omega_r [rad/s]$', Interpreter='latex');
        title('Angular speed');

        % Accelerations
        figure;
        subplot(3,1,1);
        plot(t_grid, xdd, 'LineWidth', 1.5);
        grid on;
        ylabel('$\ddot{x}_b [m/s^2]$', Interpreter='latex');
        title('Base x-acceleration');

        subplot(3,1,2);
        plot(t_grid, ydd, 'LineWidth', 1.5);
        grid on;
        ylabel('$\ddot{y}_b [m/s^2]$', Interpreter='latex');
        title('Base y-acceleration');

        subplot(3,1,3);
        plot(t_grid, thdd, 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('$\ddot{\theta}_b [rad/s^2]$', Interpreter='latex');
        title('Base angular acceleration');
    end
end

%% ==================================================================
%% 4) Wheel velocity converter test
%% ==================================================================
function test_wheel_converter(phys)
    % Pure translation → w_r ≈ w_l
    v = 1; omega = 0;
    [wr, wl] = mm_base_vel_to_wheel_rates(v, omega, phys);
    assert(abs(wr - wl) < 1e-9);

    % Pure rotation → w_r ≈ -w_l
    v = 0; omega = 1;
    [wr2, wl2] = mm_base_vel_to_wheel_rates(v, omega, phys);
    assert(abs(wr2 + wl2) < 1e-9);

    % Vectorized input
    v_vec = [0.1; 0.2; 0.3];
    w_vec = [0.0; 0.1; -0.1];
    [wrv, wlv] = mm_base_vel_to_wheel_rates(v_vec, w_vec, phys);
    assert(all(size(wrv) == size(v_vec)));
    assert(all(size(wlv) == size(v_vec)));
end

%% ==================================================================
%% 5) η reference builder test
%% ==================================================================
function test_eta_reference(phys, traj, Tf, do_plots)
    dt = 0.01;
    [ref, base_ref_all, arm_ref_all] = mm_build_eta_reference(phys, traj, Tf, dt);

    % Dimensions
    assert(size(ref.eta,2)      == 5);       % [theta_r theta_l q1 q2 q3]
    assert(size(ref.eta_dot,2)  == 5);
    assert(size(ref.eta_ddot,2) == 5);
    assert(size(base_ref_all,1) == numel(ref.t));
    assert(size(arm_ref_all,1)  == numel(ref.t));

    % Check initial and final base pose are consistent with traj
    p0_eta = base_ref_all(1,:).';
    pf_eta = base_ref_all(end,:).';
    assert(norm(p0_eta - traj.p0) < 1e-6);
    assert(norm(pf_eta - traj.pf) < 1e-6);

    % Cross-check: base velocities from wheel rates vs SE(2) planner
    % Pick a mid time index
    k_mid = floor(numel(ref.t)/2);
    t_mid = ref.t(k_mid);

    % Wheel rates from η̇
    theta_r_dot = ref.eta_dot(k_mid,1);
    theta_l_dot = ref.eta_dot(k_mid,2);

    r = phys.r;
    L = phys.L;

    v_from_wheels     = r/2 * (theta_r_dot + theta_l_dot);
    omega_from_wheels = r/(2*L) * (theta_r_dot - theta_l_dot);

    % Base reference from planner at t_mid
    base_mid = mm_planner_base_se2_geodesic(t_mid, traj);

    assert(abs(v_from_wheels     - base_mid.v_r)     < 1e-3);
    assert(abs(omega_from_wheels - base_mid.omega_r) < 1e-3);

    if do_plots
        % Quick overlay: wheel angles vs time
        figure;
        plot(ref.t, ref.eta(:,1), 'LineWidth', 1.5); hold on;
        plot(ref.t, ref.eta(:,2), 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('$\theta_{r,l} [rad]$', Interpreter='latex');
        title('Wheel reference angles from η(t)');
        legend('\theta_r','\theta_l','Location','best');
    end
end

%% ==================================================================
%% 6) Controller init test
%% ==================================================================
function test_controller_init(phys, Tf)
    % Use the same basic scenario used in the main tests
    traj   = mm_scenario_basic(Tf);
    dt_ref = 0.01;

    [ctrl, base_ref_all, arm_ref_all] = mm_controller_init(phys, traj, dt_ref); %#ok<ASGLU>

    assert(isfield(ctrl, 'gains'));
    assert(isfield(ctrl, 'traj'));
    assert(isfield(ctrl, 'ref'));

    assert(isfield(ctrl.gains,'eta'));
    assert(isfield(ctrl.gains,'base_kin'));
    assert(isfield(ctrl.gains,'arm_kin'));

    assert(isfield(ctrl.ref,'t'));
    assert(isfield(ctrl.ref,'eta'));
    assert(isfield(ctrl.ref,'eta_dot'));
    assert(isfield(ctrl.ref,'eta_ddot'));
end
