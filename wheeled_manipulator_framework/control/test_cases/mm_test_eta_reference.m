function mm_test_eta_reference()
%MM_TEST_ETA_REFERENCE
%  Test the η-reference builder mm_build_eta_reference:
%    - Dimensions of eta, eta_dot, eta_ddot
%    - Consistency between base v,omega and wheel rates
%    - Arm boundary conditions

    clc; close all;
    fprintf('=== TEST 2: η-REFERENCE BUILDER ===\n');

    [phys, ~] = mm_params_build_save_load('load', 'mm_params');
    traj      = mm_scenario_basic();
    Tf        = traj.Tf;
    dt        = 0.01;

    [ref, base_ref_all, arm_ref_all] = mm_build_eta_reference( ...
                                phys, traj, Tf, dt);

    t       = ref.t;
    eta     = ref.eta;
    eta_dot = ref.eta_dot;
    eta_ddot= ref.eta_ddot;

    fprintf(' Ref grid: N = %d, t from %.3f to %.3f\n', numel(t), t(1), t(end));

    % 1) Dimensions
    assert(size(eta,1)      == 5, 'eta must be 5xN.');
    assert(size(eta_dot,1)  == 5, 'eta_dot must be 5xN.');
    assert(size(eta_ddot,1) == 5, 'eta_ddot must be 5xN.');

    % 2) Check wheel/base consistency: recompute v,omega from wheels
    r = phys.r;
    L = phys.L;

    theta_r_dot = eta_dot(1,:);
    theta_l_dot = eta_dot(2,:);

    v_from_wheels = 0.5 * r * (theta_r_dot + theta_l_dot);
    w_from_wheels = 0.5 * r/L * (theta_r_dot - theta_l_dot);

    figure;
    subplot(2,1,1);
    plot(t, base_ref_all.v_r, 'k-', t, v_from_wheels, 'r--');
    grid on; legend('v_r from planner','v from wheels');
    title('Linear velocity consistency');

    subplot(2,1,2);
    plot(t, base_ref_all.omega_r, 'k-', t, w_from_wheels, 'r--');
    grid on; legend('\omega_r from planner','\omega from wheels');
    title('Angular velocity consistency');

    % 3) Check start/end velocities near zero
    fprintf(' v_ref(0)=%.4f, v_ref(Tf)=%.4f\n', base_ref_all.v_r(1), base_ref_all.v_r(end));
    fprintf(' omega_ref(0)=%.4f, omega_ref(Tf)=%.4f\n', base_ref_all.omega_r(1), base_ref_all.omega_r(end));

    % 4) Quick check: arm boundary conditions
    fprintf(' Arm q(0)   = [% .3f % .3f % .3f]\n', arm_ref_all.q(:,1));
    fprintf(' Arm q(Tf)  = [% .3f % .3f % .3f]\n', arm_ref_all.q(:,end));
    fprintf(' q0_arm (traj) = [% .3f % .3f % .3f]\n', traj.q0_arm);
    fprintf(' qf_arm (traj) = [% .3f % .3f % .3f]\n', traj.qf_arm);

    fprintf('=== TEST 2 DONE ===\n');
end
