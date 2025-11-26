function [K_mobile, U_mobile, q, qd, COM_mobile] = mm_energy_mobile()
% MM_ENERGY_MOBILE
% Kinetic & potential energy of the differential-drive base + wheels
% (unconstrained, no nonholonomic A(q) yet).
%
% Outputs:
%   K_mobile : total kinetic energy (wheels + base)
%   U_mobile : total potential energy (usually 0 here)
%   q, qd    : generalized coordinates and their time-derivatives
%   COM_mobile : struct of COM positions for base & wheels

%% Time + generalized coordinates
syms t real
syms x_m(t) y_m(t)
syms theta_m(t) theta_r(t) theta_l(t) theta_1(t) theta_2(t) theta_3(t)

% Unified generalized coordinates (same ordering used everywhere)
q  = [x_m(t); y_m(t); theta_m(t); theta_r(t); theta_l(t); theta_1(t); theta_2(t); theta_3(t)];
qd = diff(q,t);

%% Geometry (subset relevant to mobile)
syms L d r L_cmx_m real
assumeAlso([L d r L_cmx_m] > 0)

cm = cos(theta_m(t));
sm = sin(theta_m(t));

%% COM positions (wheels + base body)
x_cm_r = x_m(t) + L*sm;        y_cm_r = y_m(t) - L*cm;
x_cm_l = x_m(t) - L*sm;        y_cm_l = y_m(t) + L*cm;

x_cm_m = x_m(t) + L_cmx_m*cm;  y_cm_m = y_m(t) + L_cmx_m*sm;

COM_mobile = struct( ...
    'x_cm_r', x_cm_r, 'y_cm_r', y_cm_r, ...
    'x_cm_l', x_cm_l, 'y_cm_l', y_cm_l, ...
    'x_cm_m', x_cm_m, 'y_cm_m', y_cm_m );

%% Velocities via Jacobians
vel = @(p) jacobian(p,q)*qd;

v_cmr = vel([x_cm_r; y_cm_r]);
v_cml = vel([x_cm_l; y_cm_l]);
v_cmm = vel([x_cm_m; y_cm_m]);

speed2 = @(v) simplify(v.'*v, 'Steps', 20);

v2_cmr = speed2(v_cmr);
v2_cml = speed2(v_cml);
v2_cmm = speed2(v_cmm);

%% Angular rates
w_m = diff(theta_m,t);   % base yaw
w_r = diff(theta_r,t);   % right wheel spin
w_l = diff(theta_l,t);   % left  wheel spin

%% Mass & inertias (mobile)
syms m_w m_m I_yyw I_zzw I_m real
assumeAlso([m_w m_m I_yyw I_zzw I_m] > 0)

%% Kinetic energies: (67)–(69)-style
K_r = 1/2*m_w*v2_cmr + 1/2*I_yyw*w_m^2 + 1/2*I_zzw*w_r^2;
K_l = 1/2*m_w*v2_cml + 1/2*I_yyw*w_m^2 + 1/2*I_zzw*w_l^2;

K_base = 1/2*m_m*v2_cmm + 1/2*I_m*w_m^2;

K_mobile = simplify(K_r + K_l + K_base, 'Steps', 50);
%% Potential energy (base & wheels usually taken as reference = 0)
U_mobile = sym(0);

end