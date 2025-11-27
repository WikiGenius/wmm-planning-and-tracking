function xi_scaled = mm_base_scale_twist_for_limits(xi_raw, Tf, traj)
%MM_BASE_SCALE_TWIST_FOR_LIMITS
%  Scale the SE(2) twist xi_raw so that, under quintic time-scaling,
%  the peak linear and angular speeds do not exceed requested limits.
%
%  Inputs:
%    xi_raw : [vx; vy; omega] (geodesic twist for s in [0,1])
%    Tf     : total duration
%    traj   : struct with optional fields
%             .v_des_base     (max linear speed)
%             .omega_des_base (max angular speed)
%
%  Output:
%    xi_scaled : scaled twist

    xi   = xi_raw(:);
    vvec = xi(1:2);
    w    = xi(3);

    v_mag = norm(vvec);

    % Peak sdot for standard quintic s(t)
    sdot_max = 1.875 / Tf;   % known constant

    v_peak_raw = v_mag * sdot_max;
    w_peak_raw = abs(w) * sdot_max;

    k_v = inf;
    k_w = inf;

    if isfield(traj, 'v_des_base') && traj.v_des_base > 0
        k_v = traj.v_des_base / max(v_peak_raw, 1e-9);
    end

    if isfield(traj, 'omega_des_base') && traj.omega_des_base > 0
        k_w = traj.omega_des_base / max(w_peak_raw, 1e-9);
    end

    k = min([k_v, k_w, 1]);   % don't *increase* twist if not needed
    if ~isfinite(k)
        k = 1;
    end

    xi_scaled = k * xi;
end
