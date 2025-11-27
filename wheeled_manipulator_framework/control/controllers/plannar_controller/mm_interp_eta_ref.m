function [eta_d, eta_dot_d, eta_ddot_d] = mm_interp_eta_ref(ref, t)
%MM_INTERP_ETA_REF
%  Interpolate η, η̇, η̈ from precomputed reference struct:
%
%    ref.t        : 1xN time grid
%    ref.eta      : 5xN
%    ref.eta_dot  : 5xN
%    ref.eta_ddot : 5xN
%
%  For a given time t (scalar), returns:
%    eta_d      : 5x1
%    eta_dot_d  : 5x1
%    eta_ddot_d : 5x1
%
%  Uses 'pchip' interpolation and clamps t to [t(1), t(end)].

    % -----------------------------
    % 1) Time grid and clamping
    % -----------------------------
    t_grid = ref.t(:);         % N x 1
    t_min  = t_grid(1);
    t_max  = t_grid(end);

    t_clamped = min(max(t, t_min), t_max);

    % -----------------------------
    % 2) Prepare data for interp1
    % -----------------------------
    % ref.eta etc are 5xN (rows = DOF, cols = time).
    % interp1 expects V as N x M (rows = time samples).
    eta_grid      = ref.eta.';       % N x 5
    eta_dot_grid  = ref.eta_dot.';   % N x 5
    eta_ddot_grid = ref.eta_ddot.';  % N x 5

    % -----------------------------
    % 3) Interpolate at t_clamped
    % -----------------------------
    eta_d_row      = interp1(t_grid, eta_grid,      t_clamped, 'pchip');  % 1x5
    eta_dot_d_row  = interp1(t_grid, eta_dot_grid,  t_clamped, 'pchip');  % 1x5
    eta_ddot_d_row = interp1(t_grid, eta_ddot_grid, t_clamped, 'pchip');  % 1x5

    % -----------------------------
    % 4) Return as column vectors
    % -----------------------------
    eta_d      = eta_d_row(:);      % 5x1
    eta_dot_d  = eta_dot_d_row(:);  % 5x1
    eta_ddot_d = eta_ddot_d_row(:); % 5x1
end
