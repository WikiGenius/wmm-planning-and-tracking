function [ref, base_ref_all, arm_ref_all] = mm_build_eta_reference(phys, traj, Tf, dt_ref)
%MM_BUILD_ETA_REFERENCE
%  Precompute η(t), η̇(t), η̈(t) for the entire simulation horizon.
%  Uses SE(2) planner via mm_traj_planner_base_arm and converts base
%  trajectory to wheel trajectories (θr, θl).
%
%  Base pose is at O_b: [x_b, y_b, theta_m].

    %% 1) Time grid consistency
    if isfield(traj,'Tf') && abs(traj.Tf - Tf) > 1e-12
        warning('mm_build_eta_reference:InconsistentTf', ...
           'Tf argument (%.4f) differs from traj.Tf (%.4f). Using traj.Tf.', ...
            Tf, traj.Tf);
    end
    Tf = traj.Tf;

    t_ref = (0:dt_ref:Tf).';
    Nd    = numel(t_ref);

    %% 2) Preallocate
    base_ref_all = zeros(Nd,3);   % [x_b  y_b  theta_m]
    arm_ref_all  = zeros(Nd,3);   % [theta1 theta2 theta3]

    v_r_all     = zeros(Nd,1);    % commanded base linear velocity v_c
    omega_r_all = zeros(Nd,1);    % commanded base angular velocity ω_c

    arm_q_all   = zeros(Nd,3);
    arm_qd_all  = zeros(Nd,3);
    arm_qdd_all = zeros(Nd,3);

    %% 3) Sample trajectory planner
    for k = 1:Nd
        t = t_ref(k);

        [base_k, arm_k] = mm_traj_planner_base_arm(t, traj);

        % Base references
        base_ref_all(k,:) = [base_k.x, base_k.y, base_k.theta];
        v_r_all(k)        = base_k.v_r;
        omega_r_all(k)    = base_k.omega_r;

        % Arm references
        arm_ref_all(k,:)  = arm_k.q(:).';
        arm_q_all(k,:)    = arm_k.q(:).';
        arm_qd_all(k,:)   = arm_k.qdot(:).';
        arm_qdd_all(k,:)  = arm_k.qddot(:).';
    end

    %% 4) Differentiate base_ref_all to get ẋ_b, ẏ_b, θ̇_m
    x_b_all     = base_ref_all(:,1);
    y_b_all     = base_ref_all(:,2);
    theta_m_all = base_ref_all(:,3);

    x_b_dot_all     = gradient(x_b_all,     t_ref);
    y_b_dot_all     = gradient(y_b_all,     t_ref);
    theta_m_dot_all = gradient(theta_m_all, t_ref); %#ok<NASGU> % (not used now, but kept for completeness)

    %% 5) Base → wheel (exact format mm_base_vel_to_wheel_rates)
    % Requires phys.r, phys.L, phys.d
    if ~isfield(phys,'d')
        error('phys struct must contain phys.d (offset of O_b from axle).');
    end

    [theta_r_dot_d, theta_l_dot_d] = mm_base_vel_to_wheel_rates( ...
        v_r_all, omega_r_all, x_b_dot_all, y_b_dot_all, theta_m_all, phys);

    % Integrate to get θ_r, θ_l
    theta_r_d = cumtrapz(t_ref, theta_r_dot_d);
    theta_l_d = cumtrapz(t_ref, theta_l_dot_d);

    % Differentiate to get θ̈_r, θ̈_l
    theta_r_ddot_d = gradient(theta_r_dot_d, t_ref);
    theta_l_ddot_d = gradient(theta_l_dot_d, t_ref);

    %% 6) Assemble η, η̇, η̈
    % η      = [θ_r, θ_l, q_arm]
    % η̇     = [θ̇_r, θ̇_l, q̇_arm]
    % η̈     = [θ̈_r, θ̈_l, q̈_arm]
    eta_all      = [theta_r_d,      theta_l_d,      arm_q_all];
    eta_dot_all  = [theta_r_dot_d,  theta_l_dot_d,  arm_qd_all];
    eta_ddot_all = [theta_r_ddot_d, theta_l_ddot_d, arm_qdd_all];

    %% 7) Pack output struct
    ref = struct();
    ref.t        = t_ref;
    ref.eta      = eta_all;
    ref.eta_dot  = eta_dot_all;
    ref.eta_ddot = eta_ddot_all;
end
