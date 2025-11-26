function dX = mm_closed_loop_rhs(t, X, u_fun)
%MM_CLOSED_LOOP_RHS
% Wrapper for the closed-loop dynamics:
%   dX/dt = f(t, X) = mm_forward_dynamics_reduced(t, X, u_fun)
%
% Inputs:
%   t     : time
%   X     : state [q; eta_dot]
%   u_fun : handle @(t,q,eta_dot) -> tau
%
% Output:
%   dX    : state derivative

    % Extract q and eta_dot (you can also do this inside u_fun if you prefer)
    % but mm_forward_dynamics_reduced expects X and a u_fun(t,q,eta_dot).
    dX = mm_forward_dynamics_reduced(t, X, u_fun);
end
