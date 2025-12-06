function [v_c, omega_c, err] = mm_kin_ctrl_base_kanayama(q, base_ref, gains, phys)
%MM_KIN_CTRL_BASE_KANAYAMA
%  Kanayama-style kinematic controller for a unicycle / diff-drive base.
%
%  Inputs:
%    q        : [x; y; theta] current base pose in world frame
%    base_ref : struct with fields:
%                  x, y, theta, v_r, omega_r
%    gains    : struct with fields:
%                  Kx, Ky, Kth
%    phys     : struct (optional saturation fields):
%                  v_max, omega_max  (if present, used for clipping)
%
%  Outputs:
%    v_c, omega_c : commanded linear and angular velocities
%    err          : struct with fields:
%                   xe, ye, thetae (errors in robot frame)

    % -----------------------------
    % Basic safety / consistency checks
    % -----------------------------
    if numel(q) < 3
        error('mm_kin_ctrl_base_kanayama: q must be at least 3x1 [x;y;theta].');
    end

    required_ref_fields = {'x','y','theta','v_r','omega_r'};
    for k = 1:numel(required_ref_fields)
        f = required_ref_fields{k};
        if ~isfield(base_ref, f)
            error('mm_kin_ctrl_base_kanayama: base_ref.%s is missing.', f);
        end
    end

    required_gain_fields = {'Kx','Ky','Kth'};
    for k = 1:numel(required_gain_fields)
        f = required_gain_fields{k};
        if ~isfield(gains, f)
            error('mm_kin_ctrl_base_kanayama: gains.%s is missing.', f);
        end
    end

    % -----------------------------
    % Current pose
    % -----------------------------
    x  = q(1);
    y  = q(2);
    th = q(3);

    % -----------------------------
    % Reference pose and velocities
    % -----------------------------
    x_r     = base_ref.x;
    y_r     = base_ref.y;
    th_r    = base_ref.theta;
    v_r     = base_ref.v_r;
    omega_r = base_ref.omega_r;

    % -----------------------------
    % World-frame position & heading error
    % -----------------------------
    dx = x_r - x;
    dy = y_r - y;

    % Wrap heading error to (-pi, pi] ?
    dth = atan2(sin(th_r - th), cos(th_r - th));
    % -----------------------------
    % Transform position error into robot frame
    % R = R(-theta)
    % -----------------------------
    c = cos(th);
    s = sin(th);
    R = [ c  s;
         -s  c ];

    e_xy = R * [dx; dy];
    x_e  = e_xy(1);
    y_e  = e_xy(2);
    th_e = dth;

    % -----------------------------
    % Gains
    % -----------------------------
    Kx  = gains.Kx;
    Ky  = gains.Ky;
    Kth = gains.Kth;

    % -----------------------------
    % Kanayama tracking law
    % -----------------------------
    v_c     = v_r*cos(th_e) + Kx * x_e;
    
    omega_c = omega_r + v_r*(Ky*y_e + Kth*sin(th_e));

    % -----------------------------
    % Optional saturation from phys
    % -----------------------------
    if nargin >= 4 && ~isempty(phys)
        if isfield(phys,'v_max') && phys.v_max > 0
            v_c = max(min(v_c,  phys.v_max), -phys.v_max);
        end
        if isfield(phys,'omega_max') && phys.omega_max > 0
            omega_c = max(min(omega_c, phys.omega_max), -phys.omega_max);
        end
    end

    % -----------------------------
    % Export error struct for logging
    % -----------------------------
    err = struct('xe', x_e, 'ye', y_e, 'thetae', th_e);
    % err
end
