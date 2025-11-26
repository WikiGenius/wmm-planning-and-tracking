function tau = mm_fb_lin_tracker(t, X, eta_d, eta_dot_d, eta_ddot_d, gains_eta, phys)
%MM_FB_LIN_TRACKER
%  Feedback linearization controller on reduced η-coordinates.
%
%  Inputs:
%    t          : time (used only for diagnostics / warnings)
%    X          : [q; eta_dot] full state (13x1)
%                 q       = [x y theta_m theta_r theta_l theta_1 theta_2 theta_3]
%                 eta_dot = [theta_r_dot theta_l_dot theta_1_dot theta_2_dot theta_3_dot]
%    eta_d      : 5x1 desired η
%    eta_dot_d  : 5x1 desired η̇
%    eta_ddot_d : 5x1 desired η̈
%    gains_eta  : struct with fields:
%                   .Kp  (scalar, 5-vector, or 5x5)
%                   .Kd  (scalar, 5-vector, or 5x5)
%    phys       : robot physical parameters (currently unused here, kept
%                 for compatibility / future torque saturation)
%
%  Output:
%    tau        : 5x1 torque vector for [τr τl τ1 τ2 τ3]

    % -----------------------------
    % 0) Basic dimensional checks
    % -----------------------------
    X = X(:);
    if numel(X) < 13
        error('mm_fb_lin_tracker: X must be at least 13x1 ([q; eta_dot]).');
    end

    eta_d      = eta_d(:);
    eta_dot_d  = eta_dot_d(:);
    eta_ddot_d = eta_ddot_d(:);

    n_eta = 5;
    if any([numel(eta_d), numel(eta_dot_d), numel(eta_ddot_d)] ~= n_eta)
        error('mm_fb_lin_tracker: eta_d, eta_dot_d, eta_ddot_d must all be 5x1.');
    end

    % -----------------------------
    % 1) Unpack state
    % -----------------------------
    q       = X(1:8);          % [x y theta_m theta_r theta_l theta_1 theta_2 theta_3]
    eta_dot = X(9:13);         % η̇ = [θ̇r θ̇l θ̇1 θ̇2 θ̇3]
    eta     = q(4:8);          % η  = [θr θl θ1 θ2 θ3]

    % -----------------------------
    % 2) Extract & normalize gains
    % -----------------------------
    if ~isfield(gains_eta,'Kp') || ~isfield(gains_eta,'Kd')
        error('mm_fb_lin_tracker: gains_eta must have fields Kp and Kd.');
    end

    Kp_eta = expand_gain(gains_eta.Kp, n_eta, 'Kp');
    Kd_eta = expand_gain(gains_eta.Kd, n_eta, 'Kd');

    % -----------------------------
    % 3) Tracking errors
    % -----------------------------
    e     = eta     - eta_d;
    e_dot = eta_dot - eta_dot_d;

    % -----------------------------
    % 4) Virtual control (linear error dynamics)
    %
    %   η̈ = η̈_d - Kd*ė - Kp*e
    % -----------------------------
    v = eta_ddot_d - Kd_eta*e_dot - Kp_eta*e;

    % -----------------------------
    % 5) Robot reduced dynamics
    % -----------------------------
    R_q   = mm_R_fun(q);           % 8x5
    q_dot = R_q * eta_dot;         % 8x1

    M1 = mm_M1_fun(q);             % 5x5
    C1 = mm_C1_fun(q, q_dot);      % 5x5
    G1 = mm_G1_fun(q);             % 5x1
    B1 = mm_B1_fun(q);             % 5x5

    % -----------------------------
    % 6) Conditioning check (optional but useful)
    % -----------------------------
    % Gamma maps v (desired eta_ddot) to tau in the ideal linearized case.
    % Ill-conditioning ⇒ sensitive torque commands.
    Gamma = B1 \ M1;
    sG    = svd(Gamma);
    if any(sG <= eps)
        warning('mm_fb_lin_tracker:SingularGamma', ...
            'Singular Gamma at t=%.3f (svd min≈0).', t);
    else
        condG = max(sG) / min(sG);
        if condG > 1e4
            warning('mm_fb_lin_tracker:GammaIllConditioned', ...
                'cond(Gamma)=%.2e at t=%.3f', condG, t);
        end
    end

    % -----------------------------
    % 7) Damping matrix (loaded once)
    % -----------------------------
    persistent D_eta
    if isempty(D_eta)
        data = load('mm_dynamics_reduced.mat', 'D_eta');
        D_eta = data.D_eta;
        if ~all(size(D_eta) == [n_eta n_eta])
            error('mm_fb_lin_tracker: D_eta in mm_dynamics_reduced.mat must be 5x5.');
        end
    end

    % -----------------------------
    % 8) Final feedback-linearizing torque
    %
    %   M1*η̈ + (C1 + D)*η̇ + G = B1 * τ
    %
    %   ⇒ τ = B1⁻¹ ( M1*v + (C1 + D)*η̇ + G )
    % -----------------------------
    tau = B1 \ ( M1*v + (C1 + D_eta)*eta_dot + G1 );
end

% ------------------------------------------------------------------
% Helper: expand scalar/vector/matrix gain to full n x n
% ------------------------------------------------------------------
function K = expand_gain(Kraw, n, name)
    if isscalar(Kraw)
        K = Kraw * eye(n);
    elseif isvector(Kraw) && numel(Kraw) == n
        K = diag(Kraw(:));
    else
        K = Kraw;
    end

    if ~all(size(K) == [n n])
        error('mm_fb_lin_tracker: %s must be scalar, %d-vector, or %dx%d matrix.', ...
               name, n, n, n);
    end
end
