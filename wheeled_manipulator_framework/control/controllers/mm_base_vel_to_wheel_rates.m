function [theta_r_dot, theta_l_dot] = mm_base_vel_to_wheel_rates(v_cmd, omega_cmd, phys)
%MM_BASE_VEL_TO_WHEEL_RATES
%  Map base linear/angular velocity (v, omega) → wheel angular velocities.
%
%  Inputs:
%    v_cmd      : commanded linear velocity of base (m/s)
%    omega_cmd  : commanded angular velocity of base (rad/s)
%    phys       : struct containing base geometry
%                 phys.r  = wheel radius (m)
%                 phys.L  = half axle length (m)
%
%  Outputs:
%    theta_r_dot : right wheel angular velocity (rad/s)
%    theta_l_dot : left wheel angular velocity (rad/s)
%
%  Differential Drive Kinematic Model:
%
%        [θ̇_r]   = (1/r) * [1   L] [v]
%        [θ̇_l]               [1  -L] [ω]
%
%  Convention: positive ω rotates counterclockwise.

    % Extract physical parameters
    r = phys.r;     % wheel radius
    L = phys.L;     % half axle length

    % Safety checks
    if r <= 0
        error('mm_base_vel_to_wheel_rates: wheel radius r must be > 0.');
    end

    % Compute wheel rates
    theta_r_dot = ( v_cmd + omega_cmd * L ) / r;
    theta_l_dot = ( v_cmd - omega_cmd * L ) / r;

end
