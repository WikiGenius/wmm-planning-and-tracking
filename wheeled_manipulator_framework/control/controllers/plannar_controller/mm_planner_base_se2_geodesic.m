function base_ref = mm_planner_base_se2_geodesic(t, traj)
%MM_PLANNER_BASE_SE2_GEODESIC
%  Base trajectory in SE(2) using:
%    - geodesic between p0 and pf
%    - quintic time-scaling s(t) with s(0)=0, s(Tf)=1
%
%  IMPORTANT:
%    This version always reaches pf exactly at t = Tf.
%
%  Inputs:
%    t    : time (clamped to [0, Tf] outside)
%    traj : struct with fields
%           .Tf
%           .p0 = [x0; y0; theta0]
%           .pf = [xf; yf; thetaf]
%
%  Output:
%    base_ref : struct with fields
%       x, y, theta
%       xdot, ydot, thetadot
%       xddot, yddot, thetaddot
%       v_r, omega_r, v_r_dot, omega_r_dot  (last two optional)

    Tf = traj.Tf;
    t  = min(max(t,0), Tf);   % clamp into [0, Tf]

    % 1) Build SE(2) transforms and relative motion
    T0      = se2_from_pose(traj.p0(:));
    Tf_pose = se2_from_pose(traj.pf(:));

    DeltaT = T0 \ Tf_pose;
    xi     = se2_log(DeltaT);    % [vx; vy; omega], s ∈ [0,1]
    % NOTE: no scaling here → ensures final pose is exactly pf.

    % 2) Quintic time-scaling s(t), sdot(t), sddot(t)
    [s, sdot, sddot] = mm_quintic_timescaling(t, Tf);

    % 3) Pose along SE(2) geodesic
    T_rel = se2_exp(xi, s);
    T     = T0 * T_rel;

    R = T(1:2,1:2);
    p = T(1:2,3);

    x  = p(1);
    y  = p(2);
    th = atan2(R(2,1), R(1,1));

    % 4) Velocities (body → world)
    v_body  = xi(1:2) * sdot;      % body-frame linear velocity
    omega_r = xi(3)    * sdot;     % scalar angular velocity

    v_world = R * v_body;
    xdot    = v_world(1);
    ydot    = v_world(2);
    v_r     = hypot(xdot, ydot);
    % v_r = v_body(1);

    % 5) Accelerations (body → world)
    a_body      = xi(1:2) * sddot;    % body-frame linear accel
    omega_r_dot = xi(3)    * sddot;   % scalar angular accel

    Omega = [ 0       -omega_r;
              omega_r  0      ];      % skew(omega) in 2D
    Rdot      = R * Omega;
    a_world   = R * a_body + Rdot * v_body;

    xddot     = a_world(1);
    yddot     = a_world(2);
    thetadot  = omega_r;
    thetaddot = omega_r_dot;

    % Optional: scalar speed derivative (not required, but useful)
    % v_r_dot = (xdot*xddot + ydot*yddot) / max(v_r, 1e-9);

    base_ref = struct( ...
        'x', x, 'y', y, 'theta', th, ...
        'xdot', xdot, 'ydot', ydot, ...
        'xddot', xddot, 'yddot', yddot, ...
        'thetadot', thetadot, 'thetaddot', thetaddot, ...
        'v_r', v_r, 'omega_r', omega_r ...
        ... % ,'v_r_dot', v_r_dot, 'omega_r_dot', omega_r_dot ...
        );
end
