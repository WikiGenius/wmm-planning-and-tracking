function [tau, Lagr, K_total, U_total, q, qd, COM] = mm_build_unconstrained_tau()
% MM_BUILD_UNCONSTRAINED_TAU
% Combine mobile + manipulator energies and compute unconstrained
% Euler–Lagrange equations:
%
%   tau(q,qd) = d/dt(∂L/∂qd) - ∂L/∂q
%
% No nonholonomic constraints enforced here.
%
% Outputs:
%   tau      : 8x1 symbolic vector of generalized forces (unconstrained)
%   Lagr     : L = K_total - U_total
%   K_total  : total kinetic energy
%   U_total  : total potential energy
%   q, qd    : generalized coordinates and derivatives
%   COM      : struct with COM_mobile and COM_manip

%% Get energies from both subsystems
[K_mobile, U_mobile, q_m, qd_m, COM_mobile]   = mm_energy_mobile();
[K_manip, U_manip, q_p, qd_p, COM_manip]      = mm_energy_manipulator();

% Basic consistency checks (same q, qd layout)
assert(isequal(q_m,  q_p),  'q mismatch between mobile and manipulator.');
assert(isequal(qd_m, qd_p), 'qd mismatch between mobile and manipulator.');

q  = q_m;
qd = qd_m;

% Total energies
K_total = simplify(K_mobile + K_manip, 'Steps', 50);
U_total = simplify(U_mobile + U_manip, 'Steps', 50);

% Lagrangian
Lagr = simplify(K_total - U_total, 'Steps', 50);

% Build τ (unconstrained)
syms t real
dLdqd      = jacobian(Lagr, qd).';
ddt_dLdqd  = diff(dLdqd, t);        % time derivative (q,qdot are functions of t)
dLdq       = jacobian(Lagr, q).';

tau = simplify(ddt_dLdqd - dLdq, 'Steps', 50);

% Pack COM info
COM = struct('mobile', COM_mobile, 'manip', COM_manip);

end
