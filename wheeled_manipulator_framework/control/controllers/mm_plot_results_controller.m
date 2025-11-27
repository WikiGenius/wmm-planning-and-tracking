function mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys)
%MM_PLOT_RESULTS_CONTROLLER
%  Visualization for:
%    - Base path vs reference (in B and M frames)
%    - η, η̇, η̈ trajectories vs references
%    - η tracking errors
%
%  Inputs:
%    t_sol        : time vector from ODE45 (Nx1 or 1xN)
%    X_sol        : state trajectory (N x 13)
%    ctrl         : controller struct (must contain ctrl.ref)
%    base_ref_all : struct from mm_build_eta_reference (B-frame reference)
%    arm_ref_all  : (currently unused here, kept for future)
%    phys         : physical params (for offset MB)

    %% ----------------------------------
    % 0) Ensure shapes
    %% ----------------------------------
    t_sol = t_sol(:);         % N x 1
    N     = numel(t_sol);

    %% ----------------------------------
    % 1) Unpack state: q, eta_dot, eta
    %% ----------------------------------
    X_sol       = X_sol(:,:);       % N x 13
    q_sol       = X_sol(:, 1:8);    % [x_b y_b theta_b theta_r theta_l q1 q2 q3]
    eta_dot_sol = X_sol(:, 9:13);   % [theta_r_dot, theta_l_dot, q1dot, q2dot, q3dot]
    eta_sol     = q_sol(:, 4:8);    % η = [theta_r; theta_l; q1; q2; q3], N x 5

    x_b   = q_sol(:,1);
    y_b   = q_sol(:,2);
    th_b  = q_sol(:,3);

    %% ----------------------------------
    % 2) Interpolate η, η̇, η̈ references on t_sol
    %% ----------------------------------
    ref    = ctrl.ref;
    t_ref  = ref.t(:);                   % N_ref x 1

    % reference arrays: 5 x N_ref  → N_ref x 5 for interp1
    eta_ref_grid      = ref.eta.';       % N_ref x 5
    eta_dot_ref_grid  = ref.eta_dot.';   % N_ref x 5
    eta_ddot_ref_grid = ref.eta_ddot.';  % N_ref x 5

    % interpolate with 'pchip'
    eta_ref_all      = interp1(t_ref, eta_ref_grid,      t_sol, 'pchip');  % N x 5
    eta_dot_ref_all  = interp1(t_ref, eta_dot_ref_grid,  t_sol, 'pchip');  % N x 5
    eta_ddot_ref_all = interp1(t_ref, eta_ddot_ref_grid, t_sol, 'pchip');  % N x 5

    % approximate actual η̈ via gradient of η̇ (for plotting only)
    eta_ddot_sol = zeros(N, 5);
    for i = 1:5
        eta_ddot_sol(:,i) = gradient(eta_dot_sol(:,i), t_sol);
    end

    % tracking errors in η space
    e_eta      = eta_sol - eta_ref_all;        % N x 5
    e_eta_norm = vecnorm(e_eta, 2, 2);         % N x 1

    %% ----------------------------------
    % 3) Base reference interpolation (B frame)
    %% ----------------------------------
    x_ref_B      = base_ref_all.x(:);
    y_ref_B      = base_ref_all.y(:);
    theta_ref_B  = base_ref_all.theta(:);
    v_ref        = base_ref_all.v_r(:);
    omega_ref    = base_ref_all.omega_r(:);

    x_ref_B_sol     = interp1(t_ref, x_ref_B,     t_sol, 'pchip');
    y_ref_B_sol     = interp1(t_ref, y_ref_B,     t_sol, 'pchip');
    theta_ref_B_sol = interp1(t_ref, theta_ref_B, t_sol, 'pchip');
    v_ref_sol       = interp1(t_ref, v_ref,       t_sol, 'pchip');
    omega_ref_sol   = interp1(t_ref, omega_ref,   t_sol, 'pchip');

    %% ----------------------------------
    % 3b) Convert B-reference → M-reference for plotting
    %% ----------------------------------
    offset = struct('dx', phys.d, 'dy', 0.0);
    x_ref_M_sol = zeros(N,1);
    y_ref_M_sol = zeros(N,1);

    for k = 1:N
        [xM, yM, ~] = mm_mount_to_center( ...
            x_ref_B_sol(k), y_ref_B_sol(k), theta_ref_B_sol(k), offset);
        x_ref_M_sol(k) = xM;
        y_ref_M_sol(k) = yM;
    end

    %% ----------------------------------
    % 4) Colors (dark theme friendly)
    %% ----------------------------------
    refColor_M     = [0.2 0.8 1.0];    % cyan  (center M)
    refColor_B     = [0.2 0.9 0.4];    % green (mount B)
    actualColor    = [1.0 0.5 0.0];    % bright orange
    refColor       = [0.2 0.9 0.8];    % generic ref
    errorColor     = [1.0 0.3 0.3];    % soft red
    errorNormColor = [1 1 1];          % white

    % small helper for axes styling
    darkAxes = @(ax) set(ax, 'Color','k', 'XColor','white', 'YColor','white');

    %% ============ Base Trajectory (B vs M) ============
    figure('Name','Base Trajectory','NumberTitle','off');
    plot(x_ref_M_sol, y_ref_M_sol, 'Color', refColor_M, 'LineWidth', 2); hold on;
    plot(x_ref_B_sol, y_ref_B_sol, 'Color', refColor_B, 'LineWidth', 2);
    plot(x_b,         y_b,         'Color', actualColor, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    legend('Ref (center M)','Ref (mount B)','Actual (B)', 'TextColor','white');
    title('Base Path in World Frame','Color','white');
    darkAxes(gca);

    %% ============ Base Orientation & Speeds ============
    figure('Name','Base Orientation & Velocities','NumberTitle','off');

    subplot(3,1,1);
    plot(t_sol, theta_ref_B_sol, 'Color', refColor,    'LineWidth',1.5); hold on;
    plot(t_sol, th_b,            'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('\theta_B [rad]','Color','white');
    legend('Ref (B)','Actual (B)','TextColor','white');
    title('Base Orientation (mount frame B)','Color','white');
    darkAxes(gca);

    subplot(3,1,2);
    plot(t_sol, v_ref_sol, 'Color', refColor, 'LineWidth',1.5);
    grid on; ylabel('v_r [m/s]','Color','white');
    title('Linear Velocity (reference)','Color','white');
    darkAxes(gca);

    subplot(3,1,3);
    plot(t_sol, omega_ref_sol, 'Color', refColor, 'LineWidth',1.5);
    grid on; ylabel('\omega_r [rad/s]','Color','white');
    xlabel('t [s]','Color','white');
    title('Angular Velocity (reference)','Color','white');
    darkAxes(gca);

    %% ============ Eta vs Eta_ref ============
    figure('Name','Eta vs Eta_ref','NumberTitle','off');
    labels = {'\theta_r','\theta_l','q_1','q_2','q_3'};
    for i = 1:5
        subplot(5,1,i);
        plot(t_sol, eta_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on; ylabel(labels{i},'Color','white');

        if i==1
            title('\eta trajectories (reference vs actual)','Color','white');
            legend('Ref','Actual','TextColor','white');
        end
        if i==5
            xlabel('t [s]','Color','white');
        end
        darkAxes(gca);
    end

    %% ============ Eta_dot vs Eta_dot_ref ============
    figure('Name','Eta_dot vs Eta_dot_ref','NumberTitle','off');
    for i = 1:5
        subplot(5,1,i);
        plot(t_sol, eta_dot_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_dot_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on; ylabel([labels{i} ' dot'],'Color','white');

        if i==1
            title('\etȧ trajectories (reference vs actual)','Color','white');
            legend('Ref','Actual','TextColor','white');
        end
        if i==5
            xlabel('t [s]','Color','white');
        end
        darkAxes(gca);
    end

    %% ============ Eta_ddot vs Eta_ddot_ref ============
    figure('Name','Eta_ddot vs Eta_ddot_ref','NumberTitle','off');
    for i = 1:5
        subplot(5,1,i);
        plot(t_sol, eta_ddot_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_ddot_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on; ylabel([labels{i} ' ddot'],'Color','white');

        if i==1
            title('\etä trajectories (reference vs actual)','Color','white');
            legend('Ref','Actual','TextColor','white');
        end
        if i==5
            xlabel('t [s]','Color','white');
        end
        darkAxes(gca);
    end

    %% ============ Eta Error ============
    figure('Name','Eta tracking error','NumberTitle','off');

    subplot(2,1,1);
    plot(t_sol, e_eta, 'Color', errorColor, 'LineWidth', 1.2);
    grid on;
    title('\eta Tracking Error (each DOF)','Color','white');
    ylabel('error','Color','white');
    legend('\theta_r','\theta_l','q_1','q_2','q_3','TextColor','white');
    darkAxes(gca);

    subplot(2,1,2);
    plot(t_sol, e_eta_norm, 'Color', errorNormColor, 'LineWidth', 2);
    grid on;
    xlabel('t [s]','Color','white');
    ylabel('||\eta - \eta_{ref}||_2','Color','white');
    title('Norm of \eta tracking error','Color','white');
    darkAxes(gca);

end
