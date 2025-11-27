function dX = mm_closed_loop_dynamics(t, X, simdata)
%MM_CLOSED_LOOP_DYNAMICS
%  Three-layer architecture (conceptual):
%
%   Layer 3 (Planner)    : mm_traj_planner_base_arm
%   Layer 2 (Kinematics) : mm_kin_ctrl_base_kanayama + mm_kin_ctrl_arm_pd
%   Layer 1 (Dynamics)   : mm_fb_lin_tracker + mm_forward_dynamics_reduced
%
%  Current implementation:
%   - Uses ONLY the offline η-reference (no online planner).
%   - η_d, η̇_d, η̈_d from mm_interp_eta_ref(ctrl.ref, t).
%   - Torques from mm_fb_lin_tracker.
%
%  Matches mm_forward_dynamics_reduced signature:
%     dX = mm_forward_dynamics_reduced(t, X, u_fun)
%  where u_fun(t,q,eta_dot) returns τ.

    %% -----------------------------
    % 0) Unpack data
    %% -----------------------------
    ctrl = simdata.ctrl;
    phys = simdata.phys;

    gains_eta = ctrl.gains.eta;
    traj      = ctrl.traj;
    base_kin  = ctrl.gains.base_kin;  %#ok<NASGU> (reserved for future use)
    arm_kin   = ctrl.gains.arm_kin;   %#ok<NASGU>

    %% -----------------------------
    % 1) Unpack state
    %% -----------------------------
    X = X(:);
    if numel(X) < 13
        error('mm_closed_loop_dynamics: X must be 13x1: [q(8); eta_dot(5)].');
    end

    q       = X(1:8);       % [x; y; theta_m; theta_r; theta_l; q1; q2; q3]
    eta_dot = X(9:13);      % [theta_r_dot; theta_l_dot; q1dot; q2dot; q3dot]

    % Base pose
    % x_b  = q(1);
    % y_b  = q(2);
    % th_b = q(3);

    % Arm joints
    % q_arm  = q(6:8);        % [q1; q2; q3]
    % qd_arm = eta_dot(3:5);  % [q1dot; q2dot; q3dot]

    eta = q(4:8);           % η = [theta_r; theta_l; q1; q2; q3]

    %% -----------------------------
    % 2) Layer 3 – (future) ONLINE planner (task-space ref)
    %% -----------------------------
    % When you implement an online planner, you can do:
    %
    %   [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj);
    %
    % where:
    %   base_ref: struct with fields {.x, .y, .theta, .v_r, .omega_r}
    %   arm_ref : struct with fields {.q, .qdot, .qddot}
    %
    % For now, this is disabled (we use only precomputed η-ref).

    %% -----------------------------
    % 3) Layer 2 – (future) Kinematic controllers
    %% -----------------------------
    % Example usage (once planner is enabled):
    %
    % (a) Base kinematics: Kanayama controller
    %
    %   [v_cmd, omega_cmd, err_kin] = mm_kin_ctrl_base_kanayama( ...
    %         [x_b; y_b; th_b], base_ref, base_kin, phys);
    %
    % (b) Arm kinematics: joint-space velocity PD
    %
    %   qdot_cmd = mm_kin_ctrl_arm_pd( ...
    %         q_arm, qd_arm, arm_ref, arm_kin.Kp, arm_kin.Kd);
    %
    % (c) Map commanded base velocities → wheel rates
    %
    %   [theta_r_dot_cmd, theta_l_dot_cmd] = mm_base_vel_to_wheel_rates( ...
    %         v_cmd, omega_cmd, phys);

    %% -----------------------------
    % 4) Layer 3 (offline) – PRECOMPUTED η reference
    %% -----------------------------
    % Use the precomputed η, η̇, η̈ (from mm_build_eta_reference)
    % as position-level references for the dynamic layer.
    [eta_ref, eta_dot_ref, eta_ddot_ref] = mm_interp_eta_ref(ctrl.ref, t);
    % eta_ref      : [θr_ref θl_ref q1_ref q2_ref q3_ref]^T
    % eta_dot_ref  : feedforward velocities from planner
    % eta_ddot_ref : feedforward accelerations from planner

    %% -----------------------------
    % 5) Compose dynamic targets η_d, η̇_d, η̈_d
    %% -----------------------------
    eta_d      = eta_ref;       % position-level tracking follows planner
    eta_dot_d  = eta_dot_ref;   % start from planner velocities
    eta_ddot_d = eta_ddot_ref;  % planner accelerations

    % If you want to blend in kinematic controllers in the future:
    %
    %   % Wheel DOFs: overwrite or blend with Kanayama correction
    %   eta_dot_d(1) = theta_r_dot_cmd;
    %   eta_dot_d(2) = theta_l_dot_cmd;
    %
    %   % Arm DOFs: overwrite or blend with joint PD correction
    %   eta_dot_d(3:5) = qdot_cmd(:);

    %% -----------------------------
    % 6) Build torque generator u_fun(t, q, eta_dot)
    %% -----------------------------
    % mm_forward_dynamics_reduced will call:
    %
    %   u = u_fun(t_local, q_local, eta_dot_local)
    %
    % We close over (eta_d, eta_dot_d, eta_ddot_d) computed above,
    % so the torque is consistent with the three-layer logic.

    u_fun = @(t_local, q_local, eta_dot_local) ...
                mm_fb_lin_tracker( ...
                    t_local, ...
                    [q_local; eta_dot_local], ...     % X_local
                    eta_d, eta_dot_d, eta_ddot_d, ... % dynamic refs
                    gains_eta, phys);

    %% -----------------------------
    % 7) Call reduced dynamics with our torque generator
    %% -----------------------------
    dX = mm_forward_dynamics_reduced(t, X, u_fun);
end
