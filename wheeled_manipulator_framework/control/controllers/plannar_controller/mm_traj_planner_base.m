function base_ref = mm_traj_planner_base(t, traj, phys)
%MM_TRAJ_PLANNER_BASE
%  Time-parameterized SE(2) trajectory for the MOBILE BASE.
%
%  Planner:
%    - R–T–R (rotate–translate–rotate) in CENTER frame M with quintic timing.
%    - Then map CENTER trajectory → MOUNT trajectory (B) using helpers.
%
%  Assumptions:
%    - Start/goal for planning are in CENTER frame M:
%         traj.p0_M = [xm0; ym0; th0];
%         traj.pf_M = [xmf; ymf; thf];
%      (conversion from user frame B/M is done ONCE in mm_scenario_basic).
%
%    - Offset from center M to mount B in body frame:
%         traj.offset_MB.dx, traj.offset_MB.dy
%      If missing, falls back to phys.d, dy = 0.
%
%  Output (IN MOUNT FRAME B):
%    base_ref.x, base_ref.y, base_ref.theta
%    base_ref.xdot, base_ref.ydot, base_ref.thetadot
%    base_ref.xddot, base_ref.yddot, base_ref.thetaddot
%    base_ref.v_r, base_ref.omega_r

    % --------------------------------------------------------------
    % 0) Time clamp
    % --------------------------------------------------------------
    Tf   = traj.Tf;
    t_cl = max(0, min(t, Tf));

    % --------------------------------------------------------------
    % 1) Start / goal in CENTER frame M
    % --------------------------------------------------------------
    p0M = traj.p0_M;   % [xm0; ym0; th0]
    pfM = traj.pf_M;   % [xmf; ymf; thf]

    xm0 = p0M(1); ym0 = p0M(2); th0 = p0M(3);
    xmf = pfM(1); ymf = pfM(2); thf = pfM(3);

    % Offset (M → B) – prefer traj.offset_MB if available
    if isfield(traj, 'offset_MB')
        offset = traj.offset_MB;
    else
        offset = struct('dx', phys.d, 'dy', 0.0);
    end

    % --------------------------------------------------------------
    % 2) R–T–R geometry in CENTER frame M
    % --------------------------------------------------------------
    dxM   = xmf - xm0;
    dyM   = ymf - ym0;
    distM = sqrt(dxM^2 + dyM^2);

    if distM < 1e-6
        % Pure rotation
        theta_line = th0;
    else
        theta_line = atan2(dyM, dxM);
    end

    dtheta1 = wrapToPi(theta_line - th0);
    dtheta2 = wrapToPi(thf       - theta_line);

    % --------------------------------------------------------------
    % 3) Segment times from nominal speeds
    % --------------------------------------------------------------
    v_nom = traj.v_des_base;
    w_nom = traj.omega_des_base;

    if abs(w_nom) < 1e-6, w_nom = 1e-3; end
    if abs(v_nom) < 1e-6, v_nom = 1e-3; end

    T1_raw = abs(dtheta1) / abs(w_nom);  % rotation 1
    T2_raw = distM       / abs(v_nom);   % translation
    T3_raw = abs(dtheta2)/ abs(w_nom);   % rotation 2

    Tsum = T1_raw + T2_raw + T3_raw;
    if Tsum < 1e-9
        % No motion: hold pose (use helpers)
        base_ref = hold_pose_B_from_M(xm0, ym0, th0, offset);
        return;
    end

    alpha = Tf / Tsum;
    T1 = T1_raw * alpha;
    T2 = T2_raw * alpha;
    T3 = T3_raw * alpha;

    t1_end = T1;
    t2_end = T1 + T2;
    % t3_end = T1 + T2 + T3 ≈ Tf

    % --------------------------------------------------------------
    % 4) Evaluate CENTER-frame reference at t_cl
    % --------------------------------------------------------------
    if t_cl <= t1_end + 1e-12
        center = segment_rotation_M(t_cl, 0, T1, xm0, ym0, th0, dtheta1);
    elseif t_cl <= t2_end + 1e-12
        center = segment_translation_M(t_cl, T1, T2, xm0, ym0, xmf, ymf, theta_line);
    else
        center = segment_rotation_M(t_cl, T1 + T2, T3, xmf, ymf, theta_line, dtheta2);
    end

    % --------------------------------------------------------------
    % 5) Convert CENTER trajectory → MOUNT trajectory with helpers
    % --------------------------------------------------------------
    mount = mm_center_traj_to_mount_traj(center, offset);

    % Body-frame forward velocity and yaw rate at mount
    v_r     =  mount.x_dot*cos(mount.theta) + mount.y_dot*sin(mount.theta);
    omega_r =  mount.theta_dot;

    % Pack reference in B-frame with "xdot" naming for controllers
    base_ref = struct( ...
        'x',         mount.x, ...
        'y',         mount.y, ...
        'theta',     mount.theta, ...
        'xdot',      mount.x_dot, ...
        'ydot',      mount.y_dot, ...
        'thetadot',  mount.theta_dot, ...
        'xddot',     mount.x_ddot, ...
        'yddot',     mount.y_ddot, ...
        'thetaddot', mount.theta_ddot, ...
        'v_r',       v_r, ...
        'omega_r',   omega_r );
end

% =====================================================================
% Helper: hold pose (CENTER M → MOUNT B)
% =====================================================================
function base_ref = hold_pose_B_from_M(xm, ym, th, offset)
    center = struct( ...
        'x',           xm, ...
        'y',           ym, ...
        'theta',       th, ...
        'x_dot',       0, ...
        'y_dot',       0, ...
        'theta_dot',   0, ...
        'x_ddot',      0, ...
        'y_ddot',      0, ...
        'theta_ddot',  0 );

    mount = mm_center_traj_to_mount_traj(center, offset);

    base_ref = struct( ...
        'x',         mount.x, ...
        'y',         mount.y, ...
        'theta',     mount.theta, ...
        'xdot',      mount.x_dot, ...
        'ydot',      mount.y_dot, ...
        'thetadot',  mount.theta_dot, ...
        'xddot',     mount.x_ddot, ...
        'yddot',     mount.y_ddot, ...
        'thetaddot', mount.theta_ddot, ...
        'v_r',       0, ...
        'omega_r',   0 );
end

% =====================================================================
% Quintic time-scaling
% =====================================================================
function [sigma, dsigma, ddsigma] = quintic_scaling(t, t_start, Tseg)
    if Tseg <= 0
        sigma = 0; dsigma = 0; ddsigma = 0;
        return;
    end

    tau = (t - t_start) / Tseg;
    tau = min(max(tau, 0), 1);

    sigma   = 10*tau^3 - 15*tau^4 + 6*tau^5;
    dsigma  = (30*tau^2 - 60*tau^3 + 30*tau^4) / Tseg;
    ddsigma = (60*tau - 180*tau^2 + 120*tau^3) / (Tseg^2);
end

% =====================================================================
% Segment: pure rotation in CENTER frame M
% =====================================================================
function center = segment_rotation_M(t, t_start, Tseg, xM, yM, th0M, dtheta)
    [sigma, dsigma, ddsigma] = quintic_scaling(t, t_start, Tseg);

    thetaM      = th0M + dtheta * sigma;
    theta_dotM  = dtheta * dsigma;
    theta_ddotM = dtheta * ddsigma;

    center = struct( ...
        'x',           xM, ...
        'y',           yM, ...
        'theta',       thetaM, ...
        'x_dot',       0, ...
        'y_dot',       0, ...
        'theta_dot',   theta_dotM, ...
        'x_ddot',      0, ...
        'y_ddot',      0, ...
        'theta_ddot',  theta_ddotM );
end

% =====================================================================
% Segment: straight-line translation in CENTER frame M
% =====================================================================
function center = segment_translation_M(t, t_start, Tseg, x0M, y0M, xfM, yfM, theta_line)
    [sigma, dsigma, ddsigma] = quintic_scaling(t, t_start, Tseg);

    dxM = xfM - x0M;
    dyM = yfM - y0M;

    xM      = x0M + dxM * sigma;
    yM      = y0M + dyM * sigma;
    x_dotM  = dxM * dsigma;
    y_dotM  = dyM * dsigma;
    x_ddotM = dxM * ddsigma;
    y_ddotM = dyM * ddsigma;

    center = struct( ...
        'x',           xM, ...
        'y',           yM, ...
        'theta',       theta_line, ...
        'x_dot',       x_dotM, ...
        'y_dot',       y_dotM, ...
        'theta_dot',   0, ...
        'x_ddot',      x_ddotM, ...
        'y_ddot',      y_ddotM, ...
        'theta_ddot',  0 );
end
