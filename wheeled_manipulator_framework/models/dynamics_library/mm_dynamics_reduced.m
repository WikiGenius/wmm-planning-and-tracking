
%% Helper Function --------------------------------------------------------
function [M1, C1, G1, B1] = mm_dynamics_reduced(M, C, G, B, R, q, qd)
%MM_DYNAMICS_REDUCED  Project full dynamics into constraint-consistent form.
%
%   Computes:
%       M1(q) = R(q)' * M(q) * R(q)
%       C1(q,qd) = R(q)' * [ M(q)*Rdot(q,qd) + C(q,qd)*R(q) ]
%       G1(q) = R(q)' * G(q)
%       B1(q) = R(q)' * B(q)
%
%   where Rdot = ∑_i ∂R/∂q_i * qd_i
%
% Inputs:
%   M   - [n×n] inertia matrix
%   C   - [n×n] Coriolis/centrifugal matrix
%   G   - [n×1] gravity vector
%   B   - [n×m_u] actuator mapping
%   R   - [n×m] constraint basis (Null(A))
%   q   - [n×1] generalized coordinates (symbolic)
%   qd  - [n×1] generalized velocities (symbolic)
%
% Outputs:
%   M1  - [m×m] reduced inertia matrix
%   C1  - [m×m] reduced Coriolis matrix
%   G1  - [m×1] reduced gravity vector
%   B1  - [m×m_u] reduced input matrix

    % --- Compute Rdot(q, qd)
    Rdot = sym(zeros(size(R)));
    for i = 1:numel(q)
        Rdot = Rdot + diff(R, q(i)) * qd(i);
    end

    % --- Reduced matrices ---
    M1 = simplify(R.' * M * R, 'Steps', 50);
    C1 = simplify(R.' * (M * Rdot + C * R), 'Steps', 50);
    G1 = simplify(R.' * G, 'Steps', 50);
    B1 = simplify(R.' * B, 'Steps', 50);
end
