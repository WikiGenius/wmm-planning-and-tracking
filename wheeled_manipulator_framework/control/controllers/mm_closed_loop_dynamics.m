function dX = mm_closed_loop_dynamics(t, X, simdata)
%MM_CLOSED_LOOP_DYNAMICS
%  Three-layer architecture:
%
%   Layer 3 (Planner)    : mm_traj_planner_base_arm
%   Layer 2 (Kinematics) : mm_kin_ctrl_base_kanayama + mm_kin_ctrl_arm_pd
%   Layer 1 (Dynamics)   : mm_fb_lin_tracker + mm_forward_dynamics_reduced
%
%  Flag-based behavior:
%     ctrl.flags.use_online_planner = false  → OFFLINE (default)
%     ctrl.flags.use_online_planner = true   → ONLINE planner + kinematics
%
%  Disturbance injection (for robustness tests):
%     - If simdata.dist_fun exists and is non-empty:
%          tau_dist = simdata.dist_fun(t, q, eta_dot)
%          u_total  = u_ctrl + tau_dist
%       Otherwise:
%          tau_dist = 0  (no disturbance)
%
%  Matches mm_forward_dynamics_reduced signature:
%     dX = mm_forward_dynamics_reduced(t, X, u_fun)
%  where u_fun(t,q,eta_dot) returns τ (control + disturbance).

    %% -----------------------------
    % 0) Unpack data
    %% -----------------------------
    ctrl = simdata.ctrl;
    phys = simdata.phys;

    gains_eta = ctrl.gains.eta;
    traj      = ctrl.traj;

    % 0.a) Master flag: offline vs online planner
    if ~isfield(ctrl, "flags") || ~isfield(ctrl.flags, "use_online_planner")
        ctrl.flags.use_online_planner = false;   % default: OFFLINE
    end
    use_online = ctrl.flags.use_online_planner;

    % 0.b) Optional kinematics gains
    base_kin = [];
    arm_kin  = [];

    if isfield(ctrl.gains, 'base_kin')
        base_kin = ctrl.gains.base_kin;
    end
    if isfield(ctrl.gains, 'arm_kin')
        arm_kin = ctrl.gains.arm_kin;
    end

    % 0.c) Disturbance function (torque-level)
    %      Expected: tau_dist = dist_fun(t, q, eta_dot)  ∈ R^5
    if isfield(simdata, 'dist_fun') && ~isempty(simdata.dist_fun)
        dist_fun = simdata.dist_fun;
    else
        % Default: no disturbance
        dist_fun = @(t_d, q_d, eta_dot_d) zeros(5,1);
    end

    %% -----------------------------
    % 1) Unpack state
    %% -----------------------------
    X = X(:);
    if numel(X) < 13
        error('mm_closed_loop_dynamics: X must be 13x1: [q(8); eta_dot(5)].');
    end

    q       = X(1:8);       % [x y theta_m theta_r theta_l q1 q2 q3]^T
    eta_dot = X(9:13);      % [theta_r_dot theta_l_dot q1dot q2dot q3dot]^T

    % Base
    x_b  = q(1);
    y_b  = q(2);
    th_b = q(3);

    % Arm
    q_arm  = q(6:8);        % [q1 q2 q3]^T
    qd_arm = eta_dot(3:5);  % [q1dot q2dot q3dot]^T

    %% -----------------------------
    % 2) ONLINE planner (if enabled)
    %% -----------------------------
    base_ref = [];
    arm_ref  = [];

    if use_online
        [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj, phys);
    end

    %% -----------------------------
    % 3) ONLINE kinematics (if enabled)
    %% -----------------------------
    theta_r_dot_cmd = [];
    theta_l_dot_cmd = [];
    qdot_cmd        = [];

    if use_online && ~isempty(base_ref) && ~isempty(base_kin)
        % (a) Base kinematics: Kanayama controller
        [v_cmd, omega_cmd, ~] = mm_kin_ctrl_base_kanayama( ...
                [x_b; y_b; th_b], base_ref, base_kin, phys);

        % Map base twist → wheel rates
        [theta_r_dot_cmd, theta_l_dot_cmd] = ...
                mm_base_vel_to_wheel_rates(v_cmd, omega_cmd, phys);
    end

    if use_online && ~isempty(arm_ref) && ~isempty(arm_kin)
        % (b) Arm kinematics: joint-space velocity PD
        qdot_cmd = mm_kin_ctrl_arm_pd( ...
                q_arm, qd_arm, arm_ref, arm_kin.Kp, arm_kin.Kd);
    end

    %% -----------------------------
    % 4) OFFLINE precomputed η-reference (default)
    %% -----------------------------
    [eta_ref, eta_dot_ref, eta_ddot_ref] = mm_interp_eta_ref(ctrl.ref, t);

    eta_d      = eta_ref;
    eta_dot_d  = eta_dot_ref;
    eta_ddot_d = eta_ddot_ref;

    %% -----------------------------
    % 5) If ONLINE → overwrite velocity layer
    %% -----------------------------
    if use_online
        % Wheel DOFs from Kanayama
        if ~isempty(theta_r_dot_cmd)
            eta_dot_d(1) = theta_r_dot_cmd;
            eta_dot_d(2) = theta_l_dot_cmd;
        end

        % Arm DOFs from joint-space velocity PD
        if ~isempty(qdot_cmd)
            qdot_cmd = qdot_cmd(:);
            if numel(qdot_cmd) >= 3
                eta_dot_d(3:5) = qdot_cmd(1:3);
            end
        end
    end

    %% -----------------------------
    % 6) Torque generator u_fun(t, q, eta_dot)
    %% -----------------------------
    % Control torque: mm_fb_lin_tracker(...)
    % Disturbance torque: dist_fun(t, q, eta_dot)
    %
    % Total torque:
    %   u_total = u_ctrl + u_dist

    u_fun = @(t_local, q_local, eta_dot_local) ...
        ( ...
            mm_fb_lin_tracker( ...
                t_local, ...
                [q_local; eta_dot_local], ...    % X_local
                eta_d, eta_dot_d, eta_ddot_d, ...% dynamic refs
                gains_eta, phys) ...
          + dist_fun(t_local, q_local, eta_dot_local) ...
        );

    %% -----------------------------
    % 7) Run reduced dynamics
    %% -----------------------------
    dX = mm_forward_dynamics_reduced(t, X, u_fun);

end
