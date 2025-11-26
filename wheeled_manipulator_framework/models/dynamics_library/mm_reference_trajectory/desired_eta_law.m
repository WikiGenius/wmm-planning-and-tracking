function [eta_d, eta_dot_d, eta_ddot_d] = desired_eta_law(t, phys)
%DESIRED_ETA_LAW
% Desired trajectories for:
%   eta = [theta_r; theta_l; theta_1; theta_2; theta_3]
%
% Here:
%   - Right & left wheels have constant angular velocity
%   - Arm joints follow sinusoids
%
% Inputs:
%   t    : time
%   phys : struct with physical parameters (not strictly needed here,
%          but kept as argument for future extensions)
%
% Outputs:
%   eta_d      : desired eta(t)
%   eta_dot_d  : desired eta_dot(t)
%   eta_ddot_d : desired eta_ddot(t)

    %#ok<*INUSD>  % phys not used in this simple version (kept for future use)

    % --- Wheels: simple constant angular speeds ---
    omega_r = 0.0;   % [rad/s]
    omega_l = 0.0;   % [rad/s]

    theta_r0 = 0.0;
    theta_l0 = 0.0;

    theta_r_d      = theta_r0 + omega_r * t;
    theta_l_d      = theta_l0 + omega_l * t;
    theta_r_dot_d  = omega_r;
    theta_l_dot_d  = omega_l;
    theta_r_ddot_d = 0.0;
    theta_l_ddot_d = 0.0;

    % --- Arm joints: sinusoids (smooth, periodic) ---
    wj = 0.5;       % joint frequency [rad/s]

    A1 = 0.3;       % amplitudes
    A2 = 0.4;
    A3 = 0.2;

    phi1 = 0.0;
    phi2 = pi/3;
    phi3 = pi/2;

    % theta_1
    theta_1_d      = A1 * sin(wj * t + phi1);
    theta_1_dot_d  = A1 * wj * cos(wj * t + phi1);
    theta_1_ddot_d = -A1 * wj^2 * sin(wj * t + phi1);

    % theta_2
    theta_2_d      = A2 * sin(wj * t + phi2);
    theta_2_dot_d  = A2 * wj * cos(wj * t + phi2);
    theta_2_ddot_d = -A2 * wj^2 * sin(wj * t + phi2);

    % theta_3
    theta_3_d      = A3 * sin(wj * t + phi3);
    theta_3_dot_d  = A3 * wj * cos(wj * t + phi3);
    theta_3_ddot_d = -A3 * wj^2 * sin(wj * t + phi3);

    % Collect in vectors
    eta_d = [ ...
        theta_r_d;
        theta_l_d;
        theta_1_d;
        theta_2_d;
        theta_3_d ];

    eta_dot_d = [ ...
        theta_r_dot_d;
        theta_l_dot_d;
        theta_1_dot_d;
        theta_2_dot_d;
        theta_3_dot_d ];

    eta_ddot_d = [ ...
        theta_r_ddot_d;
        theta_l_ddot_d;
        theta_1_ddot_d;
        theta_2_ddot_d;
        theta_3_ddot_d ];
end
