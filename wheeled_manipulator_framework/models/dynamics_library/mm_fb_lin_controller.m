function tau = mm_fb_lin_controller(t, X, ctrl)
%MM_FB_LIN_CONTROLLER
% Feedback linearization in the reduced η-coordinates:
%
% Reduced dynamics used by the plant (mm_forward_dynamics_reduced):
%
%   M1(q)*eta_ddot + (C1(q,q_dot) + D_eta)*eta_dot + G1(q) = B1(q)*tau
%
% We choose a virtual input v so that the η-error dynamics are:
%
%   e = eta - eta_d
%   e_dot = eta_dot - eta_dot_d
%   v = eta_ddot_d - Kd*e_dot - Kp*e
%
% Then the torque is:
%
%   tau = B1(q) \ ( M1(q)*v + (C1(q,q_dot) + D_eta)*eta_dot + G1(q) )
%
% Inputs:
%   t   : time
%   X   : state [q; eta_dot]
%   ctrl: struct with fields
%           - phys : physical parameters
%           - Kp   : 5x5 PD gain
%           - Kd   : 5x5 PD gain
%
% Output:
%   tau : 5x1 torque vector [tau_r; tau_l; tau_1; tau_2; tau_3]

    % ---------- Unpack state ----------
    q       = X(1:8);    % [x_b; y_b; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3]
    eta_dot = X(9:13);   % [theta_r_dot; theta_l_dot; theta_1_dot; theta_2_dot; theta_3_dot]

    % η = [theta_r; theta_l; theta_1; theta_2; theta_3]
    eta = q(4:8);

    % ---------- Desired η, η_dot, η_ddot ----------
    phys = ctrl.phys;
    [eta_d, eta_dot_d, eta_ddot_d] = desired_eta_law(t, phys);

    % Tracking errors
    e     = eta     - eta_d;
    e_dot = eta_dot - eta_dot_d;

    % Virtual acceleration (linear error dynamics e¨ + Kd e˙ + Kp e = 0)
    v = eta_ddot_d - ctrl.Kd*e_dot - ctrl.Kp*e;

    % ---------- Get reduced dynamics matrices ----------
    % Kinematics: q_dot = R(q) * eta_dot  (8×5 * 5×1 → 8×1)
    R_q   = mm_R_fun(q);        % [8x5]
    q_dot = R_q * eta_dot;      % [8x1]

    M1 = mm_M1_fun(q);          % [5x5]
    C1 = mm_C1_fun(q, q_dot);   % [5x5]  <-- IMPORTANT: uses q_dot (8x1)
    G1 = mm_G1_fun(q);          % [5x1]
    B1 = mm_B1_fun(q);          % [5x5]



    Gamma = B1 \ M1;
    
    % Quick conditioning check
    sG   = svd(Gamma);
    condG = max(sG) / min(sG);

    if condG > 1e4   % choose threshold
        warning('mm_fb_lin_controller:GammaIllConditioned', ...
            'cond(Gamma)=%.2e at t=%.3f. Consider lowering gains or avoiding this config.', ...
            condG, t);
        % adapt the gains
        % gain_scale = 1 / (1 + (condG/1e3));   % simple heuristic
        % ctrl.Kp = gain_scale * ctrl.Kp;
        % ctrl.Kd = gain_scale * ctrl.Kd;

    end

    % Damping matrix D_eta (same file used in mm_forward_dynamics_reduced)
    persistent D_eta
    if isempty(D_eta)
        data  = load('mm_dynamics_reduced.mat','D_eta');
        D_eta = data.D_eta;
    end

    % ---------- Feedback-linearizing torque ----------
    % Plant: M1*eta_ddot + (C1 + D_eta)*eta_dot + G1 = B1*tau
    % Force eta_ddot = v → tau:
    tau = B1 \ ( M1*v + (C1 + D_eta)*eta_dot + G1 );
end
