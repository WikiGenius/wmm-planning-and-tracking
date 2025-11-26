function C = mm_compute_C_from_M(M, q_sym, qd_sym)
%MM_COMPUTE_C_FROM_M  Compute Coriolis/Centrifugal matrix C(q,qd) from M(q).
%
%   C_ij(q,qd) = sum_k (1/2)[ dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i ] * qd_k
%
%   Inputs:
%       M      : [n x n] symbolic inertia matrix in terms of q_sym
%       q_sym  : [n x 1] symbolic generalized coordinates
%       qd_sym : [n x 1] symbolic generalized velocities
%
%   Output:
%       C      : [n x n] symbolic Coriolis/Centrifugal matrix
%
%   Such that: C(q,qd)*qd_sym = Coriolis+centrifugal vector.

    q_sym  = q_sym(:);
    qd_sym = qd_sym(:);
    n = length(q_sym);

    C = sym(zeros(n,n));

    % Use Christoffel-based construction
    for i = 1:n
        for j = 1:n
            C_ij = sym(0);
            for k = 1:n
                c_ijk = 1/2 * ( diff(M(i,j), q_sym(k)) + ...
                                diff(M(i,k), q_sym(j)) - ...
                                diff(M(j,k), q_sym(i)) );
                C_ij = C_ij + c_ijk * qd_sym(k);
            end
            C(i,j) = C_ij;
        end
    end

    % Light simplification (full heavy simplify is expensive)
    C = simplify(C, 'Steps', 20);
end
