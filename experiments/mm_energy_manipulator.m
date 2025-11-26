function [K_manip, U_manip, q, qd, COM_manip] = mm_energy_manipulator()
% MM_ENERGY_MANIPULATOR
% Kinetic & potential energy of 3-DOF manipulator mounted on the mobile base.
% Uses same generalized coordinates ordering as mm_energy_mobile.
%
% Outputs:
%   K_manip  : kinetic energy of links 1–3
%   U_manip  : potential energy of links 2–3 (gravity)
%   q, qd    : generalized coordinates and their time-derivatives
%   COM_manip: struct of COM positions for links 1–3

%% Time + generalized coordinates (MUST match mm_energy_mobile)
syms t real
syms x_m(t) y_m(t)
syms theta_m(t) theta_r(t) theta_l(t) theta_1(t) theta_2(t) theta_3(t)

q  = [x_m(t); y_m(t); theta_m(t); theta_r(t); theta_l(t); theta_1(t); theta_2(t); theta_3(t)];
qd = diff(q,t);

%% Geometry (manipulator + mount)
syms L d r L_0 L_1 L_2 L_cmx_1 L_cmx_2 L_cmx_3 real
assumeAlso([L d r L_0 L_1 L_2 L_cmx_1 L_cmx_2 L_cmx_3] > 0)

cm  = cos(theta_m(t));                sm  = sin(theta_m(t));
cm1 = cos(theta_m(t)+theta_1(t));     sm1 = sin(theta_m(t)+theta_1(t));
c2  = cos(theta_2(t));                s2  = sin(theta_2(t));
c23 = cos(theta_2(t)+theta_3(t));     s23 = sin(theta_2(t)+theta_3(t));

%% COM positions (45)–(52) for links 1–3

% Link 1 CM
x_cm_1 = x_m(t) + d*cm + L_cmx_1*cm1;
y_cm_1 = y_m(t) + d*sm + L_cmx_1*sm1;
% z_1 is constant here (no vertical motion), so no gravity contribution

% Link 2 CM
x_cm_2 = x_m(t) + d*cm + (L_1 + L_cmx_2*c2)*cm1;
y_cm_2 = y_m(t) + d*sm + (L_1 + L_cmx_2*c2)*sm1;
z_cm_2 = r + L_0 + L_cmx_2*s2;

% Link 3 CM
x_cm_3 = x_m(t) + d*cm + (L_1 + L_2*c2 + L_cmx_3*c23)*cm1;
y_cm_3 = y_m(t) + d*sm + (L_1 + L_2*c2 + L_cmx_3*c23)*sm1;
z_cm_3 = r + L_0 + L_2*s2 + L_cmx_3*s23;

COM_manip = struct( ...
    'x_cm_1', x_cm_1, 'y_cm_1', y_cm_1, ...
    'x_cm_2', x_cm_2, 'y_cm_2', y_cm_2, 'z_cm_2', z_cm_2, ...
    'x_cm_3', x_cm_3, 'y_cm_3', y_cm_3, 'z_cm_3', z_cm_3 );

%% Velocities via Jacobians
vel = @(p) jacobian(p,q)*qd;

v_cm1 = vel([x_cm_1; y_cm_1]);
v_cm2 = vel([x_cm_2; y_cm_2; z_cm_2]);
v_cm3 = vel([x_cm_3; y_cm_3; z_cm_3]);

speed2 = @(v) simplify(v.'*v, 'Steps', 20);

v2_cm1 = speed2(v_cm1);
v2_cm2 = speed2(v_cm2);
v2_cm3 = speed2(v_cm3);

%% Angular rates
w_m = diff(theta_m,t);
w1  = diff(theta_1,t);
w2  = diff(theta_2,t);
w3  = diff(theta_3,t);

%% Mass & inertias (links)
syms m1 m2 m3 real
syms I_zz1 I_zz2 I_yy2 I_zz3 I_yy3 real
assumeAlso([m1 m2 m3 I_zz1 I_zz2 I_yy2 I_zz3 I_yy3] > 0)

%% Kinetic energies (70)–(72) style
% Link 1: translational + rotation about z1 (theta_m + theta_1)
K_1 = 1/2*m1*v2_cm1 + 1/2*I_zz1*(w_m + w1)^2;

% Link 2: translational + rotation about z2 + pitch about y2
K_2 = 1/2*m2*v2_cm2 + 1/2*I_zz2*(w_m + w1)^2 + 1/2*I_yy2*w2^2;

% Link 3: translational + rotation about z3 + pitch about y3
K_3 = 1/2*m3*v2_cm3 + 1/2*I_zz3*(w_m + w1)^2 + 1/2*I_yy3*(w2 + w3)^2;

K_manip = simplify(K_1 + K_2 + K_3, 'Steps', 50);
%% Potential energies (76)–(77)
syms g real

U_1 = sym(0);  % link 1 no z-motion in this model
U_2 = -m2*g*(L_0 + L_cmx_2*s2);
U_3 = -m3*g*(L_0 + L_2*s2 + L_cmx_3*s23);

U_manip = simplify(U_1 + U_2 + U_3, 'Steps', 50);
end
