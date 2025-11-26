function mm_test_kin_ctrl_suite()
%MM_TEST_KIN_CTRL_SUITE
%  Robust unit tests for:
%    - mm_kin_ctrl_base_kanayama
%    - mm_kin_ctrl_arm_pd
%
%  Usage:
%    mm_test_kin_ctrl_suite;

    fprintf("\n===============================================\n");
    fprintf("   KINEMATIC CONTROLLERS TEST SUITE\n");
    fprintf("===============================================\n\n");

    % Load physical parameters using your canonical loader
    [phys, move] = mm_params_build_save_load('load', 'mm_params'); %#ok<ASGLU>

    % Optional: add velocity limits for base controller saturation tests
    phys.v_max     = 0.6;   % [m/s] (you can tune)
    phys.omega_max = 0.8;   % [rad/s]

    %% ----------------------------------------------------------
    % 1) Base controller tests
    %% ----------------------------------------------------------
    fprintf("[BASE] Zero-error consistency...\n");
    test_base_zero_error(phys);
    fprintf("  ✔ passed.\n");

    fprintf("[BASE] Straight-ahead position error...\n");
    test_base_front_error(phys);
    fprintf("  ✔ passed.\n");

    fprintf("[BASE] Lateral position error sign (turn direction)...\n");
    test_base_lateral_error(phys);
    fprintf("  ✔ passed.\n");

    fprintf("[BASE] Heading wrap-around invariance...\n");
    test_base_heading_wrap(phys);
    fprintf("  ✔ passed.\n");

    fprintf("[BASE] Optional saturation (phys.v_max, phys.omega_max)...\n");
    test_base_saturation(phys);
    fprintf("  ✔ passed.\n\n");

    %% ----------------------------------------------------------
    % 2) Arm controller tests
    %% ----------------------------------------------------------
    fprintf("[ARM] Zero-error consistency...\n");
    test_arm_zero_error();
    fprintf("  ✔ passed.\n");

    fprintf("[ARM] Position & velocity error response...\n");
    test_arm_step_error();
    fprintf("  ✔ passed.\n");

    fprintf("[ARM] Gain format handling (scalar / vector / matrix)...\n");
    test_arm_gain_formats();
    fprintf("  ✔ passed.\n");

    fprintf("[ARM] Dimension mismatch error handling...\n");
    test_arm_dim_mismatch();
    fprintf("  ✔ passed.\n");

    fprintf("\n✓ ALL KINEMATIC CONTROLLER TESTS PASSED ✓\n\n");
end

%% ==================================================================
%% BASE CONTROLLER TESTS
%% ==================================================================

function test_base_zero_error(phys)
% If robot exactly on reference pose, controller should output
% v_c ≈ v_r and omega_c ≈ omega_r (feedforward only).

    q = [1.0; 2.0; pi/3];   % arbitrary pose

    base_ref = struct();
    base_ref.x       = q(1);
    base_ref.y       = q(2);
    base_ref.theta   = q(3);
    base_ref.v_r     = 0.4;
    base_ref.omega_r = 0.3;

    gains = struct('Kx', 1.5, 'Ky', 2.0, 'Kth', 1.5);

    [v_c, omega_c, err] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys);

    % Errors should be ~0
    assert(abs(err.xe)      < 1e-10);
    assert(abs(err.ye)      < 1e-10);
    assert(abs(err.thetae)  < 1e-10);

    % Commands should match reference velocities
    assert(abs(v_c     - base_ref.v_r)     < 1e-10);
    assert(abs(omega_c - base_ref.omega_r) < 1e-10);
end

function test_base_front_error(phys)
% Robot behind the reference along x in its heading direction.
% Expect: positive v_c due to Kx * x_e.

    % Robot at origin, facing +x
    q = [0; 0; 0];

    % Reference is 1m ahead, same heading, zero feedforward velocities
    base_ref = struct();
    base_ref.x       = 1.0;
    base_ref.y       = 0.0;
    base_ref.theta   = 0.0;
    base_ref.v_r     = 0.0;
    base_ref.omega_r = 0.0;

    gains = struct('Kx', 1.0, 'Ky', 0.0, 'Kth', 0.0);

    [v_c, omega_c, err] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys);

    % In robot frame, x_e should be +1, y_e ~ 0
    assert(abs(err.xe - 1.0) < 1e-10);
    assert(abs(err.ye)       < 1e-10);

    % Pure Kx * x_e, so v_c should be positive, omega_c~0
    assert(v_c > 0);
    assert(abs(omega_c) < 1e-10);
end

function test_base_lateral_error(phys)
% Robot aligned with reference but shifted in +y in world.
% For th=0, that is left side of robot. With Ky>0, expect positive omega_c.

    % Robot at (0, 0), facing +x
    q = [0; 0; 0];

    % Reference at (0, 1), same heading
    base_ref = struct();
    base_ref.x       = 0.0;
    base_ref.y       = 1.0;
    base_ref.theta   = 0.0;
    base_ref.v_r     = 0.5;    % some forward speed
    base_ref.omega_r = 0.0;

    gains = struct('Kx', 0.0, 'Ky', 1.0, 'Kth', 0.0);

    [v_c, omega_c, err] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys); %#ok<ASGLU>

    % World error: dx=0, dy=1
    % Robot frame for th=0: x_e = 0, y_e = 1
    assert(abs(err.xe)       < 1e-10);
    assert(abs(err.ye - 1.0) < 1e-10);

    % omega_c = v_r*(Ky*y_e) = 0.5*1*1 = 0.5
    assert(omega_c > 0);  % means turn CCW to reduce lateral error
end

function test_base_heading_wrap(phys)
% If the robot heading differs by 2*pi but pose otherwise matches,
% the heading error should be ~0 (wrap invariance).

    q = [1.0; 2.0; 0.0];
    base_ref = struct();
    base_ref.x       = 1.0;
    base_ref.y       = 2.0;
    base_ref.theta   = 2*pi;   % same as 0, but wrapped
    base_ref.v_r     = 0.2;
    base_ref.omega_r = -0.1;

    gains = struct('Kx', 1.0, 'Ky', 1.0, 'Kth', 1.0);

    [~, ~, err] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys);

    % theta error should be wrapped to ~0
    assert(abs(err.thetae) < 1e-10);
end

function test_base_saturation(phys)
% If phys.v_max and phys.omega_max are given, controller outputs
% should be clipped to these bounds.

    q = [0; 0; 0];

    % Place ref far away with heading difference to push large commands
    base_ref = struct();
    base_ref.x       = 10.0;
    base_ref.y       = 5.0;
    base_ref.theta   = pi/2;
    base_ref.v_r     = 2.0;
    base_ref.omega_r = 1.0;

    gains = struct('Kx', 5.0, 'Ky', 5.0, 'Kth', 5.0);

    [v_c, omega_c, ~] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys);

    assert(abs(v_c)     <= phys.v_max   + 1e-12);
    assert(abs(omega_c) <= phys.omega_max + 1e-12);
end

%% ==================================================================
%% ARM CONTROLLER TESTS
%% ==================================================================

function test_arm_zero_error()
% If q = q_ref and qdot = qdot_ref, then qdot_cmd should equal qdot_ref.

    q_arm     = [0.1; -0.2; 0.3];
    qdot_arm  = [0.05; 0.0; -0.02];

    arm_ref = struct();
    arm_ref.q    = q_arm;
    arm_ref.qdot = qdot_arm;

    Kp_arm = diag([2 2 2]);
    Kd_arm = diag([1 1 1]);

    qdot_cmd = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, Kp_arm, Kd_arm);

    assert(norm(qdot_cmd - qdot_arm) < 1e-12);
end

function test_arm_step_error()
% Simple test: q below reference by 0.1 rad -> positive correction if Kp > 0.

    q_arm    = [0.0; 0.0; 0.0];
    qdot_arm = [0.0; 0.0; 0.0];

    arm_ref = struct();
    arm_ref.q    = [0.1; -0.2; 0.3];
    arm_ref.qdot = [0.0;  0.0; 0.0];

    % Use scalar gains
    Kp_arm = 5.0;
    Kd_arm = 1.0;

    qdot_cmd = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, Kp_arm, Kd_arm);

    % For joint 1: desired q=0.1, current=0 -> positive qdot_cmd(1)
    assert(qdot_cmd(1) > 0);
    % For joint 2: desired q=-0.2, current=0 -> negative qdot_cmd(2)
    assert(qdot_cmd(2) < 0);
end

function test_arm_gain_formats()
% Check that Kp_arm, Kd_arm can be:
%   - scalar
%   - 3-vector (diagonal)
%   - full 3x3

    q_arm    = [0.0; 0.0; 0.0];
    qdot_arm = [0.0; 0.0; 0.0];

    arm_ref = struct();
    arm_ref.q    = [0.1; 0.1; 0.1];
    arm_ref.qdot = [0.0; 0.0; 0.0];

    % 1) scalar
    qs = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, 2.0, 0.5);

    % 2) vector (diagonal)
    qv = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, [2 2 2], [0.5 0.5 0.5]);

    % 3) full matrix
    KpM = 2.0*eye(3);
    KdM = 0.5*eye(3);
    qm  = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, KpM, KdM);

    % They should all be identical for symmetric values
    assert(norm(qs - qv) < 1e-12);
    assert(norm(qs - qm) < 1e-12);
end

function test_arm_dim_mismatch()
% If arm_ref.q dimension doesn't match q_arm, the controller must throw an error.

    q_arm    = [0.0; 0.0; 0.0];
    qdot_arm = [0.0; 0.0; 0.0];

    % WRONG dimension (2 instead of 3)
    arm_ref = struct();
    arm_ref.q    = [0.1; 0.2];
    arm_ref.qdot = [0.0; 0.0];

    Kp_arm = 2.0;
    Kd_arm = 0.5;

    did_error = false;

    try
        mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, Kp_arm, Kd_arm);
    catch ME
        if contains(ME.identifier, 'mm_kin_ctrl_arm_pd') || ...
           contains(ME.message, 'dimension mismatch')
            did_error = true;
        end
    end

    assert(did_error == true, ...
        'mm_kin_ctrl_arm_pd should error on dimension mismatch, but it did not.');
end
