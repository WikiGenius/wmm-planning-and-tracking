function [eta_d, eta_dot_d, eta_ddot_d] = mm_interp_eta_ref(ref, t)
%MM_INTERP_ETA_REF
%  Interpolate η, η̇, η̈ at arbitrary time t from ref struct:
%    ref.t        : time grid (Nd x 1)
%    ref.eta      : η values      (Nd x 5)
%    ref.eta_dot  : η̇ values     (Nd x 5)
%    ref.eta_ddot : η̈ values     (Nd x 5)
%
%  Outputs are column vectors (5x1).

    t_grid = ref.t(:);
    t_min  = t_grid(1);
    t_max  = t_grid(end);

    % Clamp query time into [t_min, t_max] to avoid extrapolation issues
    t_clamped = min(max(t, t_min), t_max);

    eta_d      = interp1(t_grid, ref.eta,      t_clamped, 'pchip').';
    eta_dot_d  = interp1(t_grid, ref.eta_dot,  t_clamped, 'pchip').';
    eta_ddot_d = interp1(t_grid, ref.eta_ddot, t_clamped, 'pchip').';

    % Ensure column vectors
    eta_d      = eta_d(:);
    eta_dot_d  = eta_dot_d(:);
    eta_ddot_d = eta_ddot_d(:);
end
