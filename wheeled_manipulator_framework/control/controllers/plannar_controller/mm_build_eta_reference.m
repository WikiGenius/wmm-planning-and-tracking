function [ref, base_ref_all, arm_ref_all] = mm_build_eta_reference( ...
                                phys, traj, Tf, dt)
%MM_BUILD_ETA_REFERENCE
%  Offline η, η̇, η̈ reference generator for the mobile manipulator.
%
%  Uses:
%    [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj, phys)
%
%  Then:
%    - converts base (v_r, omega_r) → wheel rates
%    - integrates wheel angles
%    - collects arm joint trajectories
%    - computes η̈ via finite differences (for feedforward)
%
%  Inputs:
%    phys : physical params (must contain phys.r, phys.L)
%    traj : scenario struct
%    Tf   : final time
%    dt   : time step for reference grid
%
%  Outputs:
%    ref : struct with fields
%           .t         : 1xN time grid
%           .eta       : 5xN (θr, θl, q1, q2, q3)
%           .eta_dot   : 5xN
%           .eta_ddot  : 5xN
%    base_ref_all : struct with arrays for plotting
%    arm_ref_all  : struct with arrays for plotting

    % ----------------------------------
    % Time grid
    % ----------------------------------
    t_vec = 0:dt:Tf;
    N     = numel(t_vec);

    % ----------------------------------
    % Preallocate arrays
    % ----------------------------------
    eta      = zeros(5, N);
    eta_dot  = zeros(5, N);
    eta_ddot = zeros(5, N);   % filled later via finite diff

    base_ref_all = struct();
    arm_ref_all  = struct();

    base_ref_all.t      = t_vec;
    base_ref_all.x      = zeros(1,N);
    base_ref_all.y      = zeros(1,N);
    base_ref_all.theta  = zeros(1,N);
    base_ref_all.v_r    = zeros(1,N);
    base_ref_all.omega_r= zeros(1,N);

    arm_ref_all.q       = zeros(3,N);
    arm_ref_all.qdot    = zeros(3,N);
    arm_ref_all.qddot   = zeros(3,N);

    % ----------------------------------
    % Initial wheel angles
    % ----------------------------------
    theta_r = 0;
    theta_l = 0;

    r = phys.r;
    L = phys.L;

    % ----------------------------------
    % Loop over time, build η and η̇
    % ----------------------------------
    for k = 1:N
        t = t_vec(k);
        % upgrade in future to get v_dot, omega_dot to get etta_ddot for
        % the base. will give use better smoothness rather than taking
        % second derivatives
        [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj, phys);

        v     = base_ref.v_r;
        omega = base_ref.omega_r;

        % wheel rates
        theta_r_dot = (1/r) * (v + L*omega);
        theta_l_dot = (1/r) * (v - L*omega);

        % integrate wheel angles (Euler)
        if k == 1
            dt_local = 0;
        else
            dt_local = t_vec(k) - t_vec(k-1);
        end
        theta_r = theta_r + theta_r_dot * dt_local;
        theta_l = theta_l + theta_l_dot * dt_local;

        % store base refs
        base_ref_all.x(k)       = base_ref.x;
        base_ref_all.y(k)       = base_ref.y;
        base_ref_all.theta(k)   = base_ref.theta;
        base_ref_all.v_r(k)     = v;
        base_ref_all.omega_r(k) = omega;

        % store arm refs
        arm_ref_all.q(:,k)      = arm_ref.q;
        arm_ref_all.qdot(:,k)   = arm_ref.qdot;
        arm_ref_all.qddot(:,k)  = arm_ref.qddot;

        % assemble η and η̇ at this time
        eta(:,k) = [theta_r; theta_l; arm_ref.q(:)];

        eta_dot(:,k) = [theta_r_dot;
                        theta_l_dot;
                        arm_ref.qdot(:)];
    end

    % ----------------------------------
    % Approximate η̈ via finite differences
    % ----------------------------------
    for i = 1:5
        eta_ddot(i,:) = finite_diff_second_order(eta_dot(i,:), dt);
    end

    % ----------------------------------
    % Pack into ref struct
    % ----------------------------------
    ref = struct();
    ref.t        = t_vec;
    ref.eta      = eta;
    ref.eta_dot  = eta_dot;
    ref.eta_ddot = eta_ddot;
end

% ==================================================================
%  Simple 2nd derivative approximator (central differences)
% ==================================================================
function dd = finite_diff_second_order(dx, dt)
    N  = numel(dx);
    dd = zeros(1,N);

    if N < 3 || dt <= 0
        return;
    end

    % interior points: central difference
    for k = 2:N-1
        dd(k) = (dx(k+1) - dx(k-1)) / (2*dt);
    end

    % endpoints: forward/backward difference
    dd(1) = (dx(2) - dx(1)) / dt;
    dd(N) = (dx(N) - dx(N-1)) / dt;
end
