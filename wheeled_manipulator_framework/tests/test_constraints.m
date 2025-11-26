function test_constraints()
%TEST_CONSTRAINTS  Validate A(q), J(q), R(q) and projection properties.
clear; clc;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')),'..')));

% Random pose
q  = [1.2; -0.4; 0.7; 0.1; -0.2; 0.3];

% Build J and R at q (qd is not needed to build these)
[J,R] = mm_projection(q, zeros(6,1));

% 1) Orthogonality: R.'*J.' == 0
E = R.'*J.';
assert(norm(E, 'fro') < 1e-12, 'R^T J^T not zero.');

% ---------- Case A: Consistent qd (built from eta) ----------
eta_true = [0.5; -0.2; 0.01; 0.02; -0.01];
qd_cons  = R * eta_true;

% Recover eta and check exact reconstruction
eta_hat  = R \ qd_cons;
qd_rec   = R * eta_hat;

assert(norm(qd_rec - qd_cons) < 1e-12, 'Reconstruction failed for consistent qd.');
assert(norm(J * qd_cons) < 1e-12, 'Constraint violated for consistent qd.');

% ---------- Case B: Inconsistent qd (arbitrary) ----------
qd_rand  = [0.3; 0.1; 0.2; 0.01; -0.02; 0.03];  % does NOT necessarily satisfy J*qd=0
eta_ls   = R \ qd_rand;            % least-squares coefficients
qd_proj  = R * eta_ls;             % projection into nullspace of J
res      = qd_rand - qd_proj;      % rejected component

% Projection must satisfy the constraint
assert(norm(J * qd_proj) < 1e-12, 'Projected qd does not satisfy constraint.');

% Residual must be orthogonal to columns of R (i.e., R.'*res == 0)
assert(norm(R.' * res) < 1e-12, 'Residual not orthogonal to allowed directions.');

disp('✅ constraints/projection tests passed.');
end
