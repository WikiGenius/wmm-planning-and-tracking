function mm_test_planner_basic()
    clc; close all;
    fprintf('=== TEST 1: PLANNER BASIC CHECKS ===\n');

    % Load phys and basic scenario
    [phys, ~] = mm_params_build_save_load('load', 'mm_params');
    traj      = mm_scenario_basic();

    Tf   = traj.Tf;
    N    = 11;
    ts   = linspace(0, Tf, N);

    xs = zeros(1,N); ys = zeros(1,N); ths = zeros(1,N);
    vs = zeros(1,N); ws = zeros(1,N);
    q  = zeros(3,N);

    for k = 1:N
        t = ts(k);
        [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj, phys);

        xs(k)  = base_ref.x;
        ys(k)  = base_ref.y;
        ths(k) = base_ref.theta;
        vs(k)  = base_ref.v_r;
        ws(k)  = base_ref.omega_r;

        q(:,k) = arm_ref.q;
    end

    % 1) Check start/end pose match scenario
    fprintf(' Start pose  (planner): [%.3f %.3f %.3f]\n', xs(1), ys(1), ths(1));
    fprintf(' Desired p0 (scenario): [%.3f %.3f %.3f]\n', traj.p0(1), traj.p0(2), traj.p0(3));

    fprintf(' End pose    (planner): [%.3f %.3f %.3f]\n', xs(end), ys(end), ths(end));
    fprintf(' Desired pf (scenario): [%.3f %.3f %.3f]\n', traj.pf(1), traj.pf(2), traj.pf(3));

    % 2) Check v, omega at t=0 and t=Tf approximately 0
    fprintf(' v(0)=%.4f, omega(0)=%.4f\n', vs(1), ws(1));
    fprintf(' v(Tf)=%.4f, omega(Tf)=%.4f\n', vs(end), ws(end));

    % 3) Simple plots to visually inspect
    figure; 
    subplot(2,2,1);
    plot(xs, ys, 'o-'); axis equal; grid on;
    title('Base path'); xlabel('x [m]'); ylabel('y [m]');

    subplot(2,2,2);
    plot(ts, ths,'o-'); grid on; title('\theta(t)');

    subplot(2,2,3);
    plot(ts, vs,'o-'); hold on; plot(ts, ws,'x-');
    legend('v_r','\omega_r'); grid on; title('Base velocities');

    subplot(2,2,4);
    plot(ts, q); grid on; title('Arm joints q(t)'); xlabel('t [s]');
    legend('q_1','q_2','q_3');

    fprintf('=== TEST 1 DONE ===\n');
end
