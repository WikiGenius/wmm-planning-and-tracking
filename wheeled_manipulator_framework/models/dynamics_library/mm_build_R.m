function [R_base, R] = mm_build_R(phys)
%MM_BUILD_R  Compute R_base and full R matrices for the mobile manipulator.
%
%   [R_base, R] = MM_BUILD_R(phys, theta_m)
%
% Inputs:
%   phys     - structure containing physical parameters:
%              phys.r : wheel radius [m]
%              phys.L : half-distance between wheels [m]
%              phys.d : offset between wheel axis and manipulator base [m]
%
% Outputs:
%   R_base   - 5x2 matrix mapping wheel rates [theta_r_dot; theta_l_dot]
%              to generalized base rates [x_dot_m; y_dot_m; theta_m_dot; theta_r_dot; theta_l_dot].
%   R        - 8x5 matrix including manipulator joints as independent DOFs.
%              q_dot = R * u,  where u = [theta_r_dot; theta_l_dot; theta_1_dot; theta_2_dot; theta_3_dot].

% -------------------------------------------------------------------------
% References:
% [1] Prada-Jiménez et al., "Coupled Dynamic Model for a Mobile Manipulator,"
%     Int. J. Applied Eng. Research, vol. 14, no. 11, 2019.

% -------------------------------------------------------------------------
%% ------------------------------------------------------------------------
    %% 2. Define symbolic generalized coordinates
    syms theta_m real

    % Extract parameters
    r = phys.r;
    L = phys.L;
    d = phys.d;

    % Compute trigonometric terms
    cm = cos(theta_m);
    sm = sin(theta_m);

    % ---------------------------------------------------------------------
    % Base kinematic mapping (differential-drive + wheel rotations)
    % ---------------------------------------------------------------------
    R_base = [ ...
        (r/(2*L))*(L*cm - d*sm),  (r/(2*L))*(L*cm + d*sm);   % x_m_dot
        (r/(2*L))*(L*sm + d*cm),  (r/(2*L))*(L*sm - d*cm);   % y_m_dot
         r/(2*L),                -r/(2*L);                   % theta_m_dot
         1,                       0;                         % theta_r_dot
         0,                       1                          % theta_l_dot
    ];

    % ---------------------------------------------------------------------
    % Augment with manipulator joints (3 DOFs assumed independent)
    % ---------------------------------------------------------------------
    R = blkdiag(R_base, eye(3));

end
