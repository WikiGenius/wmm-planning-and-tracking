function [tau_sol, E_sol] = mm_postprocess_control_energy(t_sol, X_sol, ctrl, phys)
%MM_POSTPROCESS_CONTROL_ENERGY
%  Recompute control torques and control energy along the simulated trajectory.
%
%  Inputs:
%    t_sol  : N x 1 time vector from ODE45
%    X_sol  : N x 13 state trajectory [q; eta_dot]
%    ctrl   : controller struct (uses ctrl.ref and ctrl.gains.eta)
%    phys   : physical parameters (passed to mm_fb_lin_tracker)
%
%  Outputs:
%    tau_sol : N x 5 torques [tau_r tau_l tau_1 tau_2 tau_3]
%    E_sol   : N x 1 cumulative control energy (approx. integral of tau^T eta_dot)

    t_sol = t_sol(:);
    simdata = struct('ctrl',ctrl,'phys',phys);
    tau_sol = zeros(numel(t_sol),5);
    for k = 1:numel(t_sol)
        [~,tau] = mm_closed_loop_dynamics(t_sol(k),X_sol(k,:).',simdata);
        tau_sol(k,:) = tau.';
    end
    E_sol = cumtrapz(t_sol,sum(tau_sol.*X_sol(:,9:13),2));
end
