function mm_test_interp_eta_ref()
%MM_TEST_INTERP_ETA_REF
%  Comprehensive test suite for mm_interp_eta_ref().
%
%  Tests:
%    1) Exact-time samples
%    2) Mid-point interpolation
%    3) Out-of-range clamping (t < min, t > max)
%    4) Output size (must be 5x1)
%    5) Monotonic θr, θl if reference is monotonic
%    6) Random query stress test
%
%  Author: Muhammed Elyamani (Clean Test Version)

fprintf("\n===============================================\n");
fprintf("      TESTING mm_interp_eta_ref() FUNCTION     \n");
fprintf("===============================================\n");

%% ------------------------------------------------------------
% 1) Build synthetic but consistent reference structure
%% ------------------------------------------------------------
t_ref = (0:0.1:5).';   % 51 samples
Nd    = numel(t_ref);

% Synthetic η = [θr θl q1 q2 q3]
theta_r = 0.2 * t_ref;
theta_l = -0.1 * t_ref;
q1 = sin(0.5*t_ref);
q2 = cos(0.3*t_ref);
q3 = 0.1 * t_ref.^2;

eta = [theta_r theta_l q1 q2 q3];

% Velocities (analytic derivatives)
eta_dot = [
    0.2 * ones(Nd,1), ...
   -0.1 * ones(Nd,1), ...
    0.5*cos(0.5*t_ref), ...
   -0.3*sin(0.3*t_ref), ...
    0.2 * t_ref
];

% Accelerations
eta_ddot = [
    zeros(Nd,1), ...
    zeros(Nd,1), ...
   -0.25*sin(0.5*t_ref), ...
   -0.09*cos(0.3*t_ref), ...
    0.2 * ones(Nd,1)
];

ref = struct('t',t_ref,'eta',eta,'eta_dot',eta_dot,'eta_ddot',eta_ddot);

fprintf("✔ Reference table created (Nd=%d).\n", Nd);

%% ------------------------------------------------------------
% 2) Test exact sample times
%% ------------------------------------------------------------
fprintf("\n[Test] Exact times... ");

ok = true;
for k = 1:Nd
    t = t_ref(k);
    [e, ed, edd] = mm_interp_eta_ref(ref, t);
    if max(abs(e - eta(k,:)')) > 1e-12
        ok = false;
    end
    if max(abs(ed - eta_dot(k,:)')) > 1e-12
        ok = false;
    end
    if max(abs(edd - eta_ddot(k,:)')) > 1e-12
        ok = false;
    end
end
assert(ok, "Exact-time interpolation mismatch!");
fprintf(" ✔ PASSED\n");

%% ------------------------------------------------------------
% 3) Mid-point interpolation
%% ------------------------------------------------------------
fprintf("[Test] Mid-point interpolation... ");

t_mid = 2.345;
[e_mid, ed_mid, edd_mid] = mm_interp_eta_ref(ref, t_mid);

assert(numel(e_mid)==5, "η size incorrect");
assert(numel(ed_mid)==5, "η̇ size incorrect");
assert(numel(edd_mid)==5, "η̈ size incorrect");

fprintf(" ✔ PASSED\n");

%% ------------------------------------------------------------
% 4) Boundary clamping
%% ------------------------------------------------------------
fprintf("[Test] Boundary clamping... ");

[e_low, ed_low, edd_low] = mm_interp_eta_ref(ref, -5);
[e_high, ed_high, edd_high] = mm_interp_eta_ref(ref, 10);

assert(all(e_low == eta(1,:)'), "t < min not clamped to first value");
assert(all(ed_low == eta_dot(1,:)'), "t < min not clamped η̇");
assert(all(edd_low == eta_ddot(1,:)'), "t < min not clamped η̈");

assert(all(e_high == eta(end,:)'), "t > max not clamped to last value");

fprintf(" ✔ PASSED\n");

%% ------------------------------------------------------------
% 5) Monotonicity (for wheel angles)
%% ------------------------------------------------------------
fprintf("[Test] Monotonic θr, θl... ");

% Query fine time grid
tq = linspace(0,5,200);
theta_r_interp = zeros(200,1);
theta_l_interp = zeros(200,1);

for i=1:200
    [e_i,~,~] = mm_interp_eta_ref(ref, tq(i));
    theta_r_interp(i) = e_i(1);
    theta_l_interp(i) = e_i(2);
end

assert(all(diff(theta_r_interp) >= -1e-8), ...
       "θ_r not monotonic increasing!");
assert(all(diff(theta_l_interp) <= 1e-8), ...
       "θ_l not monotonic decreasing!");

fprintf(" ✔ PASSED\n");

%% ------------------------------------------------------------
% 6) Random stress test (1000 random queries)
%% ------------------------------------------------------------
fprintf("[Test] Random stress test... ");

for i=1:1000
    t_rand = 5 * rand();
    [e_i, ed_i, edd_i] = mm_interp_eta_ref(ref, t_rand);

    assert(all(size(e_i) == [5 1]), "η wrong size");
    assert(all(size(ed_i) == [5 1]), "η̇ wrong size");
    assert(all(size(edd_i) == [5 1]), "η̈ wrong size");
end

fprintf(" ✔ PASSED\n");

%% ------------------------------------------------------------
fprintf("\n🎉 All tests for mm_interp_eta_ref() PASSED successfully!\n");
fprintf("===============================================\n\n");
end
