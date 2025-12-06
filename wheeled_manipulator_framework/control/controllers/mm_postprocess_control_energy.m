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
    N     = numel(t_sol);

    % Unpack state
    q_sol       = X_sol(:, 1:8);   % [x_b y_b theta_b theta_r theta_l q1 q2 q3]
    eta_dot_sol = X_sol(:, 9:13);  % [theta_r_dot theta_l_dot q1dot q2dot q3dot]

    % Reference grids
    ref   = ctrl.ref;
    t_ref = ref.t(:);             % N_ref x 1

    eta_ref_grid      = ref.eta.';       % N_ref x 5
    eta_dot_ref_grid  = ref.eta_dot.';   % N_ref x 5
    eta_ddot_ref_grid = ref.eta_ddot.';  % N_ref x 5

    % Preallocate outputs
    tau_sol = zeros(N, 5);
    P_inst  = zeros(N, 1);        % instantaneous power tau^T * eta_dot
    gains_eta = ctrl.gains.eta;

    % Loop over all time samples
    for k = 1:N
        tk       = t_sol(k);
        qk       = q_sol(k, :).';
        eta_dotk = eta_dot_sol(k, :).';

        % Interpolate references at tk (column vectors 5x1)
        eta_d      = interp1(t_ref, eta_ref_grid,      tk, 'pchip', 'extrap').';
        eta_dot_d  = interp1(t_ref, eta_dot_ref_grid,  tk, 'pchip', 'extrap').';
        eta_ddot_d = interp1(t_ref, eta_ddot_ref_grid, tk, 'pchip', 'extrap').';

        % Current full state Xk = [q; eta_dot]
        Xk = [qk; eta_dotk];

        % Feedback linearization controller (must return tau 5x1)
        tau_k = mm_fb_lin_tracker( ...
                    tk, ...
                    Xk, ...               % [q; eta_dot]
                    eta_d, eta_dot_d, eta_ddot_d, ...
                    gains_eta, phys);

        tau_k = tau_k(:).';         % row 1x5
        tau_sol(k, :) = tau_k;

        % Instantaneous control power tau^T * eta_dot
        P_inst(k) = tau_k * eta_dotk;
    end

    % Cumulative control energy E(t) = ∫ tau^T eta_dot dt
    
    E_sol = cumtrapz(t_sol, P_inst);
end
