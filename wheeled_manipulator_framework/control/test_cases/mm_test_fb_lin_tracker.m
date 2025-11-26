function mm_test_fb_lin_tracker()
%MM_TEST_FB_LIN_TRACKER
%  Robust unit tests for the feedback linearization torque controller:
%    tau = mm_fb_lin_tracker(...)
%
%  Tests:
%    1) Equilibrium consistency (gravity/damping compensation)
%    2) Exact inverse dynamics consistency for arbitrary state
%    3) Gain formats (scalar / vector / full matrix)
%    4) Dimension mismatch handling
%
%  Usage:
%    mm_test_fb_lin_tracker;

    fprintf("\n===============================================\n");
    fprintf("   FEEDBACK-LINEARIZATION CONTROLLER TESTS\n");
    fprintf("===============================================\n\n");

    % Load physical parameters (even if not used directly, for interface)
    [phys, move] = mm_params_build_save_load('load', 'mm_params'); %#ok<ASGLU>
    fprintf("Loaded: mm_params from mm_params_build_save_load\n\n");

    %% ----------------------------------------------------------
    % 1) Equilibrium consistency test
    %% ----------------------------------------------------------
    fprintf("[FB-LIN] Equilibrium consistency (zero error, zero accel)...\n");
    test_fb_lin_equilibrium(phys);
    fprintf("  ✔ passed.\n");

    %% ----------------------------------------------------------
    % 2) Exact inverse dynamics consistency
    %% ----------------------------------------------------------
    fprintf("[FB-LIN] Exact inverse dynamics consistency (e=0, e_dot=0)...\n");
    test_fb_lin_inverse_dynamics(phys);
    fprintf("  ✔ passed.\n");

    %% ----------------------------------------------------------
    % 3) Gain format handling
    %% ----------------------------------------------------------
    fprintf("[FB-LIN] Gain formats (scalar / vector / matrix)...\n");
    test_fb_lin_gain_formats(phys);
    fprintf("  ✔ passed.\n");

    %% ----------------------------------------------------------
    % 4) Dimension mismatch handling
    %% ----------------------------------------------------------
    fprintf("[FB-LIN] Dimension mismatch handling...\n");
    test_fb_lin_dim_mismatch(phys);
    fprintf("  ✔ passed.\n");

    fprintf("\n✓ ALL FEEDBACK-LINEARIZATION TESTS PASSED ✓\n\n");
end

%% ==================================================================
%% 1) Equilibrium consistency:
%%    If e=0, e_dot=0, eta_ddot_d=0 and eta_dot=0, we should get
%%    M1*0 + (C1+D)*0 + G1 = B1*tau  ⇒ B1*tau - G1 ≈ 0.
%% ==================================================================
function test_fb_lin_equilibrium(phys)

    % Choose a simple pose (can be zero pose)
    q = zeros(8,1);
    % Just to make it slightly non-trivial, you can uncomment:
    % q = [0.0; 0.0; 0.0; 0.1; -0.2; 0.3; -0.1; 0.2];

    eta     = q(4:8);          % [θr θl θ1 θ2 θ3]
    eta_dot = zeros(5,1);

    % Construct full state X = [q; eta_dot]
    X = [q; eta_dot];

    % Desired equals actual, no desired acceleration
    eta_d      = eta;
    eta_dot_d  = eta_dot;
    eta_ddot_d = zeros(5,1);

    % Gains (do not matter here since e=e_dot=0)
    gains_eta.Kp = eye(5);
    gains_eta.Kd = eye(5);

    t = 0.0;

    % Compute torque from controller
    tau = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_eta, phys);

    % Retrieve dynamics and check consistency:
    M1 = mm_M1_fun(q);
    Rq = mm_R_fun(q);
    q_dot = Rq * eta_dot;
    C1 = mm_C1_fun(q, q_dot);
    G1 = mm_G1_fun(q);
    B1 = mm_B1_fun(q);

    data  = load('mm_dynamics_reduced.mat','D_eta');
    D_eta = data.D_eta;

    % In equilibrium:
    %   M1*0 + (C1 + D_eta)*0 + G1 - B1*tau ≈ 0
    residual = M1*zeros(5,1) + (C1 + D_eta)*eta_dot + G1 - B1*tau;

    assert(norm(residual) < 1e-8, ...
        'Equilibrium residual too large: ||M1*0 + (C1+D)*0 + G1 - B1*tau|| = %.3e', ...
         norm(residual));
end

%% ==================================================================
%% 2) Exact inverse dynamics consistency:
%%    For e=0, e_dot=0 ⇒ v = eta_ddot_d.
%%    We should have:
%%      M1*eta_ddot_d + (C1+D)*eta_dot + G1 = B1*tau  (up to numeric tol)
%% ==================================================================
function test_fb_lin_inverse_dynamics(phys)

    % Choose a generic, but moderate pose and eta_dot
    q = [0.1; 0.0; 0.05; 0.2; -0.1; 0.3; -0.2; 0.1];
    eta = q(4:8);
    eta_dot = [0.05; -0.03; 0.02; 0.01; -0.04];

    X = [q; eta_dot];

    % Desired equals actual state, but with some desired acceleration
    eta_d      = eta;
    eta_dot_d  = eta_dot;
    eta_ddot_d = [0.1; -0.05; 0.02; -0.01; 0.03];

    gains_eta.Kp = eye(5);     % any gains (e=0,e_dot=0 ⇒ v=eta_ddot_d)
    gains_eta.Kd = eye(5);

    t = 1.23;

    tau = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_eta, phys);

    M1 = mm_M1_fun(q);
    Rq = mm_R_fun(q);
    q_dot = Rq * eta_dot;
    C1 = mm_C1_fun(q, q_dot);
    G1 = mm_G1_fun(q);
    B1 = mm_B1_fun(q);

    data  = load('mm_dynamics_reduced.mat','D_eta');
    D_eta = data.D_eta;

    % Check: M1*eta_ddot_d + (C1 + D_eta)*eta_dot + G1 ≈ B1*tau
    lhs = M1*eta_ddot_d + (C1 + D_eta)*eta_dot + G1;
    rhs = B1 * tau;

    residual = lhs - rhs;

    assert(norm(residual) < 1e-7, ...
        'Inverse dynamics residual too large: ||lhs - rhs|| = %.3e', norm(residual));
end

%% ==================================================================
%% 3) Gain formats:
%%    Verify that scalar / vector / full-matrix Kp, Kd give identical
%%    behavior when they represent the same diagonal gains.
%% ==================================================================
function test_fb_lin_gain_formats(phys)

    % Choose arbitrary state
    q = [0.1; -0.2; 0.1; 0.15; -0.05; 0.2; -0.1; 0.05];
    eta = q(4:8);
    eta_dot = [0.02; 0.01; -0.03; 0.04; -0.02];

    X = [q; eta_dot];

    % Arbitrary desired trajectory differing from actual
    eta_d      = eta + [0.1; -0.1; 0.05; -0.02; 0.03];
    eta_dot_d  = eta_dot + [0.01; 0.0; -0.01; 0.02; -0.02];
    eta_ddot_d = [0.05; -0.03; 0.02; 0.0; -0.01];

    % Base scalar gain
    kp_scalar = 5.0;
    kd_scalar = 2.0;

    t = 0.5;

    % 1) Scalar gains
    gains_scalar.Kp = kp_scalar;
    gains_scalar.Kd = kd_scalar;
    tau_s = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_scalar, phys);

    % 2) Vector / diagonal gains
    kp_vec = kp_scalar * ones(5,1);
    kd_vec = kd_scalar * ones(5,1);
    gains_vec.Kp = kp_vec;
    gains_vec.Kd = kd_vec;
    tau_v = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_vec, phys);

    % 3) Full matrix gains
    KpM = kp_scalar * eye(5);
    KdM = kd_scalar * eye(5);
    gains_mat.Kp = KpM;
    gains_mat.Kd = KdM;
    tau_m = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_mat, phys);

    assert(norm(tau_s - tau_v) < 1e-10, ...
        'Scalar vs vector gain results differ: ||tau_s - tau_v|| = %.3e', norm(tau_s - tau_v));
    assert(norm(tau_s - tau_m) < 1e-10, ...
        'Scalar vs matrix gain results differ: ||tau_s - tau_m|| = %.3e', norm(tau_s - tau_m));
end

%% ==================================================================
%% 4) Dimension mismatch handling:
%%    If eta_d has wrong dimension, controller should error.
%% ==================================================================
function test_fb_lin_dim_mismatch(phys)

    % Simple state
    q = zeros(8,1);
    eta = q(4:8);
    eta_dot = zeros(5,1);
    X = [q; eta_dot];

    % WRONG dimension for eta_d (4 instead of 5)
    eta_d_wrong      = [0.1; 0.2; 0.3; 0.4];
    eta_dot_d_wrong  = zeros(4,1);
    eta_ddot_d_wrong = zeros(4,1);

    gains_eta.Kp = eye(5);
    gains_eta.Kd = eye(5);

    t = 0.0;

    did_error = false;
    try
        mm_fb_lin_tracker(t, X, eta_d_wrong, eta_dot_d_wrong, eta_ddot_d_wrong, gains_eta, phys);
    catch ME
        if contains(ME.identifier, 'mm_fb_lin_tracker') || ...
           contains(ME.message, 'eta_d, eta_dot_d, eta_ddot_d must all be 5x1')
            did_error = true;
        end
    end

    assert(did_error == true, ...
        'mm_fb_lin_tracker should error on wrong eta_d dimension, but it did not.');
end
