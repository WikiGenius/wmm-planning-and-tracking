function [th_r_dot_d, th_l_dot_d] = mm_base_vel_to_wheel_rates( ...
        v_c, omega_c, x_b_dot, y_b_dot, theta_m, phys)
%MM_BASE_VEL_TO_WHEEL_RATES (Exact mixed-form general mapping)
%
% Implements the exact LaTeX expression (vectorized):
%
%   θ̇_r = v/r + ( L*ω  - d*( L*ẋ_b*sin(θ_m) - L*ẏ_b*cos(θ_m) ) ) / ( r*(1+d^2) )
%   θ̇_l = v/r - ( L*ω  - d*( L*ẋ_b*sin(θ_m) - L*ẏ_b*cos(θ_m) ) ) / ( r*(1+d^2) )
%
% All inputs can be scalars or column vectors of the same length.
%
% Inputs:
%   v_c      : commanded linear velocity v   (Nx1)
%   omega_c  : commanded angular velocity ω  (Nx1)
%   x_b_dot  : base velocity ẋ_b            (Nx1)
%   y_b_dot  : base velocity ẏ_b            (Nx1)
%   theta_m  : base heading θ_m              (Nx1)
%   phys     : struct with fields:
%                 r  : wheel radius
%                 L  : half axle length
%                 d  : offset of O_b from axle (normalized as in your model)
%
% Outputs:
%   th_r_dot_d : desired right wheel angular velocity θ̇_r (Nx1)
%   th_l_dot_d : desired left  wheel angular velocity θ̇_l (Nx1)

    % -----------------------------
    % Parameter checks
    % -----------------------------
    if ~isfield(phys,'r')
        error('phys must contain phys.r (wheel radius)');
    end
    if ~isfield(phys,'L')
        error('phys must contain phys.L (half axle length)');
    end
    if ~isfield(phys,'d')
        error('phys must contain phys.d (base offset)');
    end

    r = phys.r;
    L = phys.L;
    d = phys.d;

    % Ensure column vectors (for safety)
    v_c      = v_c(:);
    omega_c  = omega_c(:);
    x_b_dot  = x_b_dot(:);
    y_b_dot  = y_b_dot(:);
    theta_m  = theta_m(:);

    % -----------------------------
    % Correction term C (elementwise)
    % C = L*ω - d*( L*ẋ_b*sinθ - L*ẏ_b*cosθ )
    % -----------------------------
    C = L .* omega_c ...
        - d .* ( L .* x_b_dot .* sin(theta_m) ...
               - L .* y_b_dot .* cos(theta_m) );

    % -----------------------------
    % Exact-form wheel rates (elementwise)
    % -----------------------------
    denom = r * (1 + d^2);

    th_r_dot_d = ( v_c ./ r ) + ( C ./ denom );
    th_l_dot_d = ( v_c ./ r ) - ( C ./ denom );
end
