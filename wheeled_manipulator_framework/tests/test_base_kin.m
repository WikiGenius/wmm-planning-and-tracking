function test_base_kin()
%TEST_BASE_KIN  Unit test for base_kin_fwd / base_kin_inv.
clear; clc;

% Add parent folder (project root) to path so MATLAB finds models/params
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..')));

% Load parameters
P = mm_physical();
tol = 1e-12;

% --- A) Straight motion (equal wheel speeds) ---
wR = 10; wL = 10;
[v,om] = base_kin_fwd(wR,wL,P);
assert(abs(v - P.base.r*10) < tol);
assert(abs(om) < tol);

% --- B) Spin in place (opposite wheels) ---
wR = 10; wL = -10;
[v,om] = base_kin_fwd(wR,wL,P);
assert(abs(v) < tol);
assert(abs(om - (10*P.base.r/P.base.L)) < tol);

% --- C) Round-trip inverse check ---
v0 = 0.35; om0 = 0.8;
[wR,wL] = base_kin_inv(v0,om0,P);
[v1,om1] = base_kin_fwd(wR,wL,P);
assert(abs(v1-v0) < 1e-12);
assert(abs(om1-om0) < 1e-12);

disp('✅ base_kin_fwd / base_kin_inv tests passed.');
end
