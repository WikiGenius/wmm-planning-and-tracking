function [tau, Lagr, K_total, U_total, mobile, manip] = mm_build_lagrangian_unconstrained()
% mm_build_lagrangian_unconstrained
% Unconstrained symbolic dynamics for a differential-drive mobile manipulator.
% Splits into:
%   mobile: wheels + base
%   manip : 3-DOF arm
% Outputs:
%   tau    : 8x1 Euler–Lagrange equations (no nonholonomic constraints)
%   Lagr   : L = K - U
%   K_total, U_total : total kinetic & potential energies
%   mobile, manip    : structs with COMs, velocities, energies

    %% Time + generalized coordinates (as functions of t)
    syms t real
    syms x_m(t) y_m(t)
    syms theta_m(t) theta_r(t) theta_l(t) theta_1(t) theta_2(t) theta_3(t)

    %% Geometry (lengths, SI)
    syms L d r L_0 L_1 L_2 L_cmx_m L_cmx_1 L_cmx_2 L_cmx_3 real
    assumeAlso([L d r L_0 L_1 L_2 L_cmx_m L_cmx_1 L_cmx_2 L_cmx_3] > 0)

    %% Masses & inertias
    syms m_w m_m m1 m2 m3 real
    syms I_yyw I_zzw I_m I_zz1 I_zz2 I_yy2 I_zz3 I_yy3 real
    assumeAlso([m_w m_m m1 m2 m3 I_yyw I_zzw I_m I_zz1 I_zz2 I_yy2 I_zz3 I_yy3] > 0)

    %% Gravity
    syms g real

    %% Generalized coordinates vector
    q  = [x_m; y_m; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3];
    qd = diff(q, t);

    %% Subsystems (pass q, qd; use local names inside)
    mobile = mm_mobile_subsystem(t, q, qd, ...
                                 L, r, L_cmx_m, ...
                                 m_w, m_m, ...
                                 I_yyw, I_zzw, I_m);

    manip  = mm_manipulator_subsystem(t, q, qd, ...
                                      d, L_0, L_1, L_2, ...
                                      L_cmx_1, L_cmx_2, L_cmx_3, ...
                                      m1, m2, m3, ...
                                      I_zz1, I_zz2, I_yy2, I_zz3, I_yy3, ...
                                      r, g);

    %% Total energies
    K_total = simplify(mobile.K + manip.K, 'Steps', 50);
    U_total = simplify(mobile.U + manip.U, 'Steps', 50);

    %% Lagrangian
    Lagr = simplify(K_total - U_total, 'Steps', 50);

    %% Unconstrained Euler–Lagrange equations
    dLdqd     = jacobian(Lagr, qd).';   % ∂L/∂q̇
    ddt_dLdqd = diff(dLdqd, t);         % d/dt(∂L/∂q̇)
    dLdq      = jacobian(Lagr, q).';    % ∂L/∂q

    tau = simplify(ddt_dLdqd - dLdq, 'Steps', 20);
end


%% ===================== MOBILE SUBSYSTEM =====================
function mobile = mm_mobile_subsystem(t, q, qd, ...
                                      L, r, L_cmx_m, ...
                                      m_w, m_m, ...
                                      I_yyw, I_zzw, I_m)
% Wheels + mobile base body (no arm).
% Uses global q = [x_m; y_m; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3].

    % Local aliases (avoid name collision with outer symfuns)
    xm  = q(1);
    ym  = q(2);
    thm = q(3);
    thr = q(4);
    thl = q(5);

    % Trig
    cm = cos(thm);
    sm = sin(thm);

    % ---------- COM positions ----------
    x_cm_r = xm + L*sm;      y_cm_r = ym - L*cm;
    x_cm_l = xm - L*sm;      y_cm_l = ym + L*cm;

    x_cm_m = xm + L_cmx_m*cm;
    y_cm_m = ym + L_cmx_m*sm;

    % ---------- Velocities via Jacobians ----------
    vel   = @(p) jacobian(p, q) * qd;

    v_cmr = vel([x_cm_r; y_cm_r]);  % right wheel CM
    v_cml = vel([x_cm_l; y_cm_l]);  % left wheel CM
    v_cmm = vel([x_cm_m; y_cm_m]);  % base CM

    speed2 = @(v) simplify(v.'*v, 'Steps', 20);

    v2_cmr = speed2(v_cmr);
    v2_cml = speed2(v_cml);
    v2_cmm = speed2(v_cmm);

    % ---------- Angular rates ----------
    w_m = diff(thm, t);      % base yaw
    w_r = diff(thr, t);      % right wheel spin
    w_l = diff(thl, t);      % left wheel spin

    % ---------- Kinetic energy ----------
    K_r = 1/2*m_w*v2_cmr + 1/2*I_yyw*(w_m.^2) + 1/2*I_zzw*(w_r.^2);
    K_l = 1/2*m_w*v2_cml + 1/2*I_yyw*(w_m.^2) + 1/2*I_zzw*(w_l.^2);

    K_m = 1/2*m_m*v2_cmm + 1/2*I_m*(w_m.^2);

    mobile.K = simplify(K_r + K_l + K_m, 'Steps', 20);

    % No gravitational PE here
    mobile.U = sym(0);

    % ---------- Store COM & velocities ----------
    mobile.COM.x_cm_r = x_cm_r;  mobile.COM.y_cm_r = y_cm_r;
    mobile.COM.x_cm_l = x_cm_l;  mobile.COM.y_cm_l = y_cm_l;
    mobile.COM.x_cm_m = x_cm_m;  mobile.COM.y_cm_m = y_cm_m;

    mobile.v.v_cmr = v_cmr;
    mobile.v.v_cml = v_cml;
    mobile.v.v_cmm = v_cmm;
end


%% ===================== MANIPULATOR SUBSYSTEM =====================
function manip = mm_manipulator_subsystem(t, q, qd, ...
                                          d, L_0, L_1, L_2, ...
                                          L_cmx_1, L_cmx_2, L_cmx_3, ...
                                          m1, m2, m3, ...
                                          I_zz1, I_zz2, I_yy2, I_zz3, I_yy3, ...
                                          r, g)
% 3-DOF manipulator mounted on the mobile base.
% Uses same global q.

    % Local aliases
    xm  = q(1);
    ym  = q(2);
    thm = q(3);
    th1 = q(6);
    th2 = q(7);
    th3 = q(8);

    % Trig shorthands
    cm  = cos(thm);              sm  = sin(thm);
    cm1 = cos(thm + th1);        sm1 = sin(thm + th1);
    c2  = cos(th2);              s2  = sin(th2);
    c23 = cos(th2 + th3);        s23 = sin(th2 + th3);

    % ---------- COM positions ----------
    % Link 1
    x_cm_1 = xm + d*cm + L_cmx_1*cm1;
    y_cm_1 = ym + d*sm + L_cmx_1*sm1;

    % Link 2
    x_cm_2 = xm + d*cm + (L_1 + L_cmx_2*c2)*cm1;
    y_cm_2 = ym + d*sm + (L_1 + L_cmx_2*c2)*sm1;
    z_cm_2 = r + L_0 + L_cmx_2*s2;

    % Link 3
    x_cm_3 = xm + d*cm + (L_1 + L_2*c2 + L_cmx_3*c23)*cm1;
    y_cm_3 = ym + d*sm + (L_1 + L_2*c2 + L_cmx_3*c23)*sm1;
    z_cm_3 = r + L_0 + L_2*s2 + L_cmx_3*s23;

    % ---------- Velocities via Jacobians ----------
    vel   = @(p) jacobian(p, q) * qd;

    v_cm1 = vel([x_cm_1; y_cm_1]);              % 2D
    v_cm2 = vel([x_cm_2; y_cm_2; z_cm_2]);      % 3D
    v_cm3 = vel([x_cm_3; y_cm_3; z_cm_3]);      % 3D

    speed2 = @(v) simplify(v.'*v, 'Steps', 20);

    v2_cm1 = speed2(v_cm1);
    v2_cm2 = speed2(v_cm2);
    v2_cm3 = speed2(v_cm3);

    % ---------- Angular rates ----------
    w_m = diff(thm, t);
    w1  = diff(th1, t);
    w2  = diff(th2, t);
    w3  = diff(th3, t);

    % ---------- Kinetic energy ----------
    K_1 = 1/2*m1*v2_cm1 ...
        + 1/2*I_zz1*(w_m + w1).^2;

    K_2 = 1/2*m2*v2_cm2 ...
        + 1/2*I_zz2*(w_m + w1).^2 ...
        + 1/2*I_yy2*(w2.^2);

    K_3 = 1/2*m3*v2_cm3 ...
        + 1/2*I_zz3*(w_m + w1).^2 ...
        + 1/2*I_yy3*(w2 + w3).^2;

    manip.K = simplify(K_1 + K_2 + K_3, 'Steps', 20);

    % ---------- Potential energy ----------
    U_1 = sym(0);
    U_2 = -m2*g*(L_0 + L_cmx_2*s2);
    U_3 = -m3*g*(L_0 + L_2*s2 + L_cmx_3*s23);

    manip.U = simplify(U_1 + U_2 + U_3, 'Steps', 20);

    % ---------- Store COM & velocities ----------
    manip.COM.x_cm_1 = x_cm_1; manip.COM.y_cm_1 = y_cm_1;

    manip.COM.x_cm_2 = x_cm_2;
    manip.COM.y_cm_2 = y_cm_2;
    manip.COM.z_cm_2 = z_cm_2;

    manip.COM.x_cm_3 = x_cm_3;
    manip.COM.y_cm_3 = y_cm_3;
    manip.COM.z_cm_3 = z_cm_3;

    manip.v.v_cm1 = v_cm1;
    manip.v.v_cm2 = v_cm2;
    manip.v.v_cm3 = v_cm3;
end
