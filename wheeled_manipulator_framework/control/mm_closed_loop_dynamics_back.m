function dX = mm_closed_loop_dynamics(t, X, simdata)
%MM_CLOSED_LOOP_DYNAMICS
%  Hierarchical closed-loop dynamics for ODE45:
%
%    Layer 3: Planner
%       [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj);
%
%    Layer 2: Kinematic control (velocity-level)
%       [v_c, ω_c]  = mm_kin_ctrl_base_kanayama(q_base, base_ref, base_kin, phys);
%       [ω_r, ω_l]  = mm_base_vel_to_wheel_rates(v_c, ω_c, phys);
%       qdot_cmd    = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, Kp_arm, Kd_arm);
%       η̇_des      = [ω_r; ω_l; qdot_cmd];
%
%    Layer 1: Dynamic control (feedback linearization)
%       Inner loop: track η̇_des using mm_fb_lin_tracker at torque level.
%
%  Inputs:
%    t       : current time
%    X       : current state [q; eta_dot]
%    simdata : struct with fields:
%                - phys
%                - traj
%                - ctrl (contains gains)
%
%  Output:
%    dX      : state derivative [q_dot; eta_ddot]

    phys = simdata.phys;
    traj = simdata.traj;
    ctrl = simdata.ctrl;

    gains_eta   = ctrl.gains.eta;
    base_kin    = ctrl.gains.base_kin;
    arm_kin     = ctrl.gains.arm_kin;

    %% --- Unpack state ---
    nq = 8;
    ne = 5;

    q       = X(1:nq);         % [x; y; theta_m; theta_r; theta_l; q1; q2; q3]
    eta_dot = X(nq+1:nq+ne);   % [θ̇_r; θ̇_l; q̇1; q̇2; q̇3]

    % Convenience slices
    q_base    = q(1:3);        % [x; y; theta_m]
    q_arm     = q(6:8);        % [q1; q2; q3]
    qdot_arm  = eta_dot(3:5);  % joint velocities from η̇

    %% --- LAYER 3: High-level planner (geometric reference) ---
    [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj);
    % base_ref: x, y, theta, v_r, omega_r
    % arm_ref:  q, qdot, qddot

    %% --- LAYER 2: Kinematic controllers (velocity-level) ---
    % Base (Kanayama-style tracking of [x,y,theta])
    [v_c, omega_c, ~] = mm_kin_ctrl_base_kanayama(q_base, base_ref, base_kin, phys);

    % Convert (v_c, omega_c) → wheel rates
    [theta_r_dot_cmd, theta_l_dot_cmd] = mm_base_vel_to_wheel_rates(v_c, omega_c, phys);

    % Arm (joint-space PD at velocity level)
    qdot_cmd = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, ...
                                  arm_kin.Kp, arm_kin.Kd);

    % Desired η̇ for dynamic layer
    eta_dot_des = [
        theta_r_dot_cmd;
        theta_l_dot_cmd;
        qdot_cmd(:)
    ];

    %% --- LAYER 1: Dynamic tracking (feedback linearization) ---
    % Inner loop: track η̇_des (velocity-level). We set:
    %   η_d      = current η  → no position term in inner loop
    %   η̇_d     = η̇_des
    %   η̈_d     = 0
    %
    % Outer kinematic controllers (Layer 2) handle position error.

    eta        = q(4:8);             % current η
    eta_d      = eta;                % position target = current state ?
    eta_dot_d  = eta_dot_des;        % velocity target from kinematic layer
    eta_ddot_d = zeros(5,1);         % accel target (can be refined later)

    % Build torque generator with correct signature u_fun(t,q,eta_dot)
    u_fun = @(t_local, q_local, eta_dot_local) ...
                mm_fb_lin_tracker(t_local, ...
                                  [q_local; eta_dot_local], ...
                                  eta_d, eta_dot_d, eta_ddot_d, ...
                                  gains_eta, phys);

    % Call your existing reduced forward dynamics (unchanged)
    dX = mm_forward_dynamics_reduced(t, X, u_fun);
end
