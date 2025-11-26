function dX = mm_forward_dynamics_reduced(t, X, u_fun)
%MM_FORWARD_DYNAMICS_REDUCED
%   State:
%       X = [q; eta_dot]
%   where
%       q       = [x_m; y_m; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3]
%       eta_dot = [theta_r_dot; theta_l_dot; theta_1_dot; theta_2_dot; theta_3_dot]
%
%   Kinematics:
%       q_dot    = R(q) * eta_dot
%
%   Reduced dynamics (projected):
%       M1(q) * eta_ddot + C1(q,q_dot)*eta_dot + G1(q) = B1(q) * u
%
%   Inputs:
%       u_fun(t,q,eta_dot) -> [tau_r; tau_l; tau_1; tau_2; tau_3]

    % Dimensions
    nq = 8;
    ne = 5;

    % Unpack state
    q       = X(1:nq);             % configuration
    eta_dot = X(nq+1:nq+ne);       % independent velocities (paper's \dot{eta})

    % Kinematic mapping: q_dot = R(q)*eta_dot
    R     = mm_R_fun(q);           % [8x5]
    q_dot = R * eta_dot;           % [8x1]

    % Reduced dynamics matrices
    M1 = mm_M1_fun(q);             % [5x5]
    C1 = mm_C1_fun(q, q_dot);      % [5x5]
    G1 = mm_G1_fun(q);             % [5x1]
    B1 = mm_B1_fun(q);             % [5x5]

    persistent D_eta
    if isempty(D_eta)
        s = load('mm_dynamics_reduced.mat','D_eta');
        D_eta = s.D_eta;
    end

    % Torques
    u = u_fun(t, q, eta_dot);      % [5x1]

    % eta_ddot from projected dynamics
    % eta_ddot = M1 \ (B1*u - C1*eta_dot - G1);

    % --- Reduced dynamics with damping ---
    % M1*eta_ddot + C1*eta_dot + D_eta*eta_dot + G1 = B1*u
    eta_ddot = M1 \ (B1*u - (C1 + D_eta)*eta_dot - G1);
    
    % State derivative
    dX = [q_dot; eta_ddot];
end
