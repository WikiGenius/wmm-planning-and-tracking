function mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys)
%MM_PLOT_RESULTS_CONTROLLER
%  Visualization of closed-loop tracking for the mobile manipulator.
%
%  Inputs:
%    t_sol         : time vector from ODE solver (Nt x 1)
%    X_sol         : state trajectory from ODE solver (Nt x 13)
%                    X = [q; eta_dot]
%                    q       = [x y theta_m theta_r theta_l q1 q2 q3]
%                    eta_dot = [theta_r_dot theta_l_dot q1dot q2dot q3dot]
%
%    ctrl          : controller struct (from mm_controller_init), must contain:
%                      - ctrl.traj       (fields: p0, pf, etc.)
%                      - ctrl.gains.eta  (Kp, Kd)
%                      - ctrl.ref        (η reference, from mm_build_eta_reference)
%
%    base_ref_all  : (Nd x 3) base references [x y theta]
%    arm_ref_all   : (Nd x 3) arm joint references [q1 q2 q3]
%    phys          : physical parameters struct

    %% 0) Basic checks and unpack state
    t_sol = t_sol(:);
    Nt    = numel(t_sol);

    if size(X_sol,2) < 13
        error('mm_plot_results_controller: X_sol must have at least 13 columns ([q; eta_dot]).');
    end

    q_sol       = X_sol(:, 1:8);    % [x y theta_m theta_r theta_l q1 q2 q3]
    eta_dot_sol = X_sol(:, 9:13);   % [th_r_dot th_l_dot q1dot q2dot q3dot]
    eta_sol     = q_sol(:, 4:8);    % [th_r th_l q1 q2 q3]

    traj = ctrl.traj;
    ref  = ctrl.ref;

    %% 1) Interpolate η, η̇, η̈ references at solver times (vectorized)
    t_ref = ref.t(:);

    eta_d_all      = interp1(t_ref, ref.eta,      t_sol, 'pchip');
    eta_dot_d_all  = interp1(t_ref, ref.eta_dot,  t_sol, 'pchip');
    eta_ddot_d_all = interp1(t_ref, ref.eta_ddot, t_sol, 'pchip');

    % Errors
    e_eta     = eta_sol     - eta_d_all;
    e_eta_dot = eta_dot_sol - eta_dot_d_all;

    %% 2) Interpolate base & arm reference for overlay (planner-level)
    x_ref  = interp1(t_ref, base_ref_all(:,1), t_sol, 'pchip');
    y_ref  = interp1(t_ref, base_ref_all(:,2), t_sol, 'pchip');
    th_ref = interp1(t_ref, base_ref_all(:,3), t_sol, 'pchip');

    q1_ref = interp1(t_ref, arm_ref_all(:,1), t_sol, 'pchip');
    q2_ref = interp1(t_ref, arm_ref_all(:,2), t_sol, 'pchip');
    q3_ref = interp1(t_ref, arm_ref_all(:,3), t_sol, 'pchip');

    %% 3) Recompute torques τ(t) and conditioning cond(Gamma(t))
    tau_all   = zeros(Nt, 5);   % [tau_r tau_l tau1 tau2 tau3]
    condG_all = nan(Nt, 1);
    E_kin     = zeros(Nt, 1);
    E_pot     = nan(Nt, 1);     % may stay NaN if no potential function

    gains_eta = ctrl.gains.eta;
    Kp_eta    = expand_gain(gains_eta.Kp, 5);

    for k = 1:Nt
        t  = t_sol(k);
        Xk = X_sol(k,:).';

        % Desired η, η̇, η̈ at this time
        eta_d_k      = eta_d_all(k,:).';
        eta_dot_d_k  = eta_dot_d_all(k,:).';
        eta_ddot_d_k = eta_ddot_d_all(k,:).';

        % Torque from feedback-linearization tracker
        tau_k = mm_fb_lin_tracker(t, Xk, ...
                                  eta_d_k, eta_dot_d_k, eta_ddot_d_k, ...
                                  gains_eta, phys);
        tau_all(k,:) = tau_k(:).';

        % Conditioning of Gamma = B1^{-1}M1
        qk       = Xk(1:8);
        eta_dotk = Xk(9:13);
        Rq       = mm_R_fun(qk);
        q_dotk   = Rq * eta_dotk;
        M1k      = mm_M1_fun(qk);
        B1k      = mm_B1_fun(qk);

        Gamma = B1k \ M1k;
        sG    = svd(Gamma);
        if all(sG > eps)
            condG_all(k) = max(sG) / min(sG);
        else
            condG_all(k) = Inf;
        end

        % Kinetic energy in reduced coordinates: 1/2 η̇ᵀ M1 η̇
        E_kin(k) = 0.5 * (eta_dotk.' * M1k * eta_dotk);

        % Optional potential energy if mm_P1_fun exists:
        if exist('mm_P1_fun','file') == 2
            Pval     = mm_P1_fun(qk);  % assume scalar or 1x1
            E_pot(k) = Pval;
        end
    end

    % Lyapunov candidate: V = 1/2 e_etaᵀ Kp e_eta + 1/2 e_eta_dotᵀ e_eta_dot
    V = zeros(Nt,1);
    for k = 1:Nt
        ek  = e_eta(k,:).';
        edk = e_eta_dot(k,:).';
        V(k) = 0.5*ek.'*Kp_eta*ek + 0.5*edk.'*edk;
    end

    %% 4) Base planar trajectory (XY)
    figure;
    plot(q_sol(:,1), q_sol(:,2), 'LineWidth', 1.6); hold on; grid on;
    plot(x_ref, y_ref, '--', 'LineWidth', 1.2);
    plot(traj.p0(1), traj.p0(2), 'go', 'MarkerFaceColor','g');
    plot(traj.pf(1), traj.pf(2), 'ro', 'MarkerFaceColor','r');
    axis equal;
    xlabel('x_b [m]');
    ylabel('y_b [m]');
    title('Base trajectory in XY-plane');
    legend('actual','reference','start','goal','Location','best');

    %% 5) Base orientation and velocities
    theta_m = q_sol(:,3);   % base heading from state

    figure;
    tiledlayout(3,1);

    % Heading
    nexttile;
    plot(t_sol, theta_m, 'LineWidth', 1.4); hold on; grid on;
    plot(t_sol, th_ref, '--', 'LineWidth', 1.2);
    ylabel('\theta_b [rad]');
    title('Base heading');
    legend('actual','reference','Location','best');

    % Forward speed v_b and angular speed omega_b (from wheels)
    theta_r_dot = eta_dot_sol(:,1);
    theta_l_dot = eta_dot_sol(:,2);

    r = phys.r;
    L = phys.L;

    v_from_wheels     = r/2     * (theta_r_dot + theta_l_dot);
    omega_from_wheels = r/(2*L) * (theta_r_dot - theta_l_dot);

    nexttile;
    plot(t_sol, v_from_wheels, 'LineWidth', 1.4); grid on;
    ylabel('v_b [m/s]');
    title('Forward speed from wheels');

    nexttile;
    plot(t_sol, omega_from_wheels, 'LineWidth', 1.4); grid on;
    xlabel('Time [s]');
    ylabel('\omega_b [rad/s]');
    title('Angular speed from wheels');

    %% 6) η and η̇ tracking + errors
    figure;
    tiledlayout(2,1);

    nexttile;
    plot(t_sol, eta_sol, 'LineWidth', 1.4); hold on;
    plot(t_sol, eta_d_all, '--', 'LineWidth', 1.0);
    grid on;
    xlabel('Time [s]');
    ylabel('\eta [rad]');
    title('\eta(t) vs \eta_d(t)');
    legend('\theta_r','\theta_l','q_1','q_2','q_3', ...
           '\theta_{r,d}','\theta_{l,d}','q_{1,d}','q_{2,d}','q_{3,d}', ...
           'Location','bestoutside');

    nexttile;
    plot(t_sol, e_eta, 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('Error [rad]');
    title('\eta tracking error');

    figure;
    tiledlayout(2,1);

    nexttile;
    plot(t_sol, eta_dot_sol, 'LineWidth', 1.4); hold on;
    plot(t_sol, eta_dot_d_all, '--', 'LineWidth', 1.0);
    grid on;
    xlabel('Time [s]');
    ylabel('\dot{\eta} [rad/s]');
    title('\dot{\eta}(t) vs \dot{\eta}_d(t)');
    legend('\theta_r','\theta_l','q_1','q_2','q_3', ...
           '\theta_{r,d}','\theta_{l,d}','q_{1,d}','q_{2,d}','q_{3,d}', ...
           'Location','bestoutside');

    nexttile;
    plot(t_sol, e_eta_dot, 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('Error [rad/s]');
    title('\dot{\eta} tracking error');

    %% 7) Arm joint trajectories vs reference
    q1 = q_sol(:,6);
    q2 = q_sol(:,7);
    q3 = q_sol(:,8);

    figure;
    tiledlayout(2,1);

    nexttile;
    plot(t_sol, [q1 q2 q3], 'LineWidth', 1.4); hold on;
    plot(t_sol, [q1_ref q2_ref q3_ref], '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('q_i [rad]');
    title('Arm joint angles vs reference');
    legend('q_1','q_2','q_3','q_{1,d}','q_{2,d}','q_{3,d}', ...
           'Location','bestoutside');

    nexttile;
    plot(t_sol, [q1 - q1_ref, q2 - q2_ref, q3 - q3_ref], 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('Error [rad]');
    title('Arm joint angle tracking error');
    legend('e_{q1}','e_{q2}','e_{q3}','Location','best');

    %% 8) Torques τ(t)
    figure;
    plot(t_sol, tau_all, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Torque [N·m]');
    title('Actuator torques');
    legend('\tau_r','\tau_l','\tau_1','\tau_2','\tau_3','Location','best');

    %% 9) Conditioning of feedback linearization Gamma
    figure;
    semilogy(t_sol, condG_all, 'LineWidth', 1.5); grid on;
    xlabel('Time [s]');
    ylabel('cond(\Gamma)');
    title('Condition number of \Gamma = B_1^{-1} M_1 over time');
    yline(1e4, '--r', 'Ill-conditioning threshold');
    legend('cond(\Gamma)','threshold','Location','best');

    %% 10) End-effector trajectory & error (planar)
    [xee, yee]         = mm_fk_ee_planar_series(q_sol, phys);
    [xee_ref, yee_ref] = mm_fk_ee_planar_series([x_ref, y_ref, th_ref, ...
                                                 zeros(Nt,2), q1_ref, q2_ref, q3_ref], phys);

    figure;
    plot(xee, yee, 'LineWidth', 1.6); hold on; grid on;
    plot(xee_ref, yee_ref, '--', 'LineWidth', 1.4);
    axis equal;
    xlabel('x_{ee} [m]');
    ylabel('y_{ee} [m]');
    title('End-effector planar trajectory');
    legend('actual','reference','Location','best');

    figure;
    ex_ee = xee - xee_ref;
    ey_ee = yee - yee_ref;
    plot(t_sol, [ex_ee, ey_ee], 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('End-effector tracking error');
    legend('e_{x,ee}','e_{y,ee}','Location','best');

    %% 11) Energy plots
    figure;
    if all(~isnan(E_pot))
        plot(t_sol, E_kin, 'LineWidth', 1.5); hold on;
        plot(t_sol, E_pot, 'LineWidth', 1.5);
        plot(t_sol, E_kin + E_pot, 'LineWidth', 1.5);
        legend('Kinetic','Potential','Total','Location','best');
    else
        plot(t_sol, E_kin, 'LineWidth', 1.5);
        legend('Kinetic','Location','best');
    end
    grid on;
    xlabel('Time [s]');
    ylabel('Energy [J] (up to constant)');
    title('Energy evolution');

    %% 12) Lyapunov candidate V(t)
    figure;
    plot(t_sol, V, 'LineWidth', 1.8);
    grid on;
    xlabel('Time [s]');
    ylabel('V(t)');
    title('Lyapunov candidate based on \eta errors');

    %% 13) Animation
    mm_animate_mm(t_sol, q_sol, phys, traj);

    fprintf('mm_plot_results_controller: plotting completed.\n');
end

%% ------------------------------------------------------------------
function K = expand_gain(Kraw, n)
% Expand scalar/vector/matrix gain to full n x n
    if isscalar(Kraw)
        K = Kraw * eye(n);
    elseif isvector(Kraw) && numel(Kraw) == n
        K = diag(Kraw(:));
    else
        K = Kraw;
    end
end

%% ------------------------------------------------------------------
function [xee, yee] = mm_fk_ee_planar_series(q_sol, phys)
%... (same as your version) ...
    x   = q_sol(:,1);
    y   = q_sol(:,2);
    thb = q_sol(:,3);
    q1  = q_sol(:,6);
    q2  = q_sol(:,7);
    q3  = q_sol(:,8);

    L0 = phys.L_0;
    L1 = phys.L_1;
    L2 = phys.L_2;
    L3 = phys.L_3;

    xs = x + L0 .* cos(thb);
    ys = y + L0 .* sin(thb);

    th1 = thb + q1;
    x1  = xs + L1 .* cos(th1);
    y1  = ys + L1 .* sin(th1);

    th2 = th1 + q2;
    x2  = x1 + L2 .* cos(th2);
    y2  = y1 + L2 .* sin(th2);

    th3 = th2 + q3;
    xee = x2 + L3 .* cos(th3);
    yee = y2 + L3 .* sin(th3);
end

%% ------------------------------------------------------------------
function mm_animate_mm(t_sol, q_sol, phys, traj)
%... (same as your version) ...
    fprintf('mm_plot_results_controller: starting animation...\n');

    Nt = numel(t_sol);

    L0 = phys.L_0;
    L1 = phys.L_1;
    L2 = phys.L_2;
    L3 = phys.L_3;

    figure;
    ax = gca;
    hold(ax, 'on'); grid(ax, 'on');
    xlabel('x [m]');
    ylabel('y [m]');
    title('Mobile manipulator animation (planar)');
    axis equal;

    x_all  = q_sol(:,1);
    y_all  = q_sol(:,2);
    margin = 0.5;
    xlim([min(x_all)-margin, max(x_all)+margin]);
    ylim([min(y_all)-margin, max(y_all)+margin]);

    plot(traj.p0(1), traj.p0(2), 'go', 'MarkerFaceColor','g');
    plot(traj.pf(1), traj.pf(2), 'ro', 'MarkerFaceColor','r');

    base_body = plot(ax, NaN, NaN, 'b-', 'LineWidth', 2);
    arm_line  = plot(ax, NaN, NaN, 'k-o', 'LineWidth', 2, 'MarkerFaceColor','k');

    step = max(1, floor(Nt/300));

    for k = 1:step:Nt
        qk  = q_sol(k,:);
        xb  = qk(1);
        yb  = qk(2);
        thb = qk(3);
        q1  = qk(6);
        q2  = qk(7);
        q3  = qk(8);

        half_w  = phys.L;
        half_l  = 0.4 * L0;
        corners = [ half_l  half_l -half_l -half_l;
                   -half_w  half_w  half_w -half_w ];

        Rb            = [cos(thb) -sin(thb); sin(thb) cos(thb)];
        corners_world = Rb * corners + [xb; yb];

        set(base_body, 'XData', [corners_world(1,:) corners_world(1,1)], ...
                       'YData', [corners_world(2,:) corners_world(2,1)]);

        xs = xb + L0*cos(thb);
        ys = yb + L0*sin(thb);

        th1 = thb + q1;
        x1  = xs + L1*cos(th1);
        y1  = ys + L1*sin(th1);

        th2 = th1 + q2;
        x2  = x1 + L2*cos(th2);
        y2  = y1 + L2*sin(th2);

        th3 = th2 + q3;
        x3  = x2 + L3*cos(th3);
        y3  = y2 + L3*sin(th3);

        set(arm_line, 'XData', [xs x1 x2 x3], ...
                      'YData', [ys y1 y2 y3]);

        drawnow limitrate;
    end

    fprintf('mm_plot_results_controller: animation completed.\n');
end
