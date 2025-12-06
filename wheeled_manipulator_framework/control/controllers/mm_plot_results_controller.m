function mm_plot_results_controller(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys, varargin)
%MM_PLOT_RESULTS_CONTROLLER
%  Visualization for:
%    - Base path vs reference (B and M frames)
%    - Base kinematics (x,y,theta and their derivatives)
%    - Control efforts (torques)  [optional]
%    - Energy / power             [optional]
%    - η, η̇, η̈ trajectories vs references
%    - η tracking errors
%
%  Inputs:
%    t_sol        : time vector from ODE45 (Nx1 or 1xN)
%    X_sol        : state trajectory (N x 13)
%    ctrl         : controller struct (must contain ctrl.ref)
%    base_ref_all : struct with B-frame reference (from planner)
%    arm_ref_all  : (currently unused here, kept for future)
%    phys         : physical parameters (for offset)
%
%    varargin (flexible order):
%      - numeric #1 (optional) : tau_sol (N x 5 control torques)
%      - numeric #2 (optional) : E_sol   (N x 1 total mechanical energy)
%      - char/string (optional): 'light' or 'dark' theme (default 'light')

    % -------- parse optional inputs: tau, E, theme --------
    tau_sol = [];
    E_sol   = [];
    theme   = 'light';  % default theme

    for k = 1:numel(varargin)
        arg = varargin{k};
        if ischar(arg) || (isstring(arg) && isscalar(arg))
            theme = lower(char(arg));
        elseif isnumeric(arg)
            if isempty(tau_sol)
                tau_sol = arg;
            elseif isempty(E_sol)
                E_sol = arg;
            end
        end
    end

    if ~ismember(theme, {'light','dark'})
        theme = 'light';
    end
    isDark = strcmp(theme,'dark');

    % Theme colors
    if isDark
        figColor  = [0 0 0];
        axBgColor = 'k';
        textColor = 'white';
    else
        figColor  = [1 1 1];
        axBgColor = 'w';
        textColor = 'k';
    end

    styleAxes = @(ax) set(ax, 'Color',axBgColor, ...
                              'XColor',textColor, ...
                              'YColor',textColor);

    % Helper to create themed figure
    newFig = @(name) set(figure('Name',name,'NumberTitle','off'), 'Color',figColor);

    % ----------------------------------
    % 0) Ensure shapes
    % ----------------------------------
    t_sol = t_sol(:);         % N x 1
    N     = numel(t_sol);

    % ----------------------------------
    % 1) Unpack state: q, eta_dot, eta
    % ----------------------------------
    X_sol       = X_sol(:,:);       % N x 13
    q_sol       = X_sol(:, 1:8);    % [x_b y_b theta_b theta_r theta_l q1 q2 q3]
    eta_dot_sol = X_sol(:, 9:13);   % [theta_r_dot, theta_l_dot, q1dot, q2dot, q3dot]
    eta_sol     = q_sol(:, 4:8);    % η = [theta_r; theta_l; q1; q2; q3], N x 5

    x_b   = q_sol(:,1);
    y_b   = q_sol(:,2);
    th_b  = q_sol(:,3);

    % ----------------------------------
    % 2) Interpolate η, η̇, η̈ references on t_sol
    % ----------------------------------
    ref    = ctrl.ref;
    t_ref  = ref.t(:);                   % N_ref x 1

    eta_ref_grid      = ref.eta.';       % N_ref x 5
    eta_dot_ref_grid  = ref.eta_dot.';   % N_ref x 5
    eta_ddot_ref_grid = ref.eta_ddot.';  % N_ref x 5

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

    % ----------------------------------
    % 3) Base reference interpolation (B frame)
    % ----------------------------------
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

    % ----------------------------------
    % 3b) Convert B-reference → M-reference for plotting
    % ----------------------------------
    offset = struct('dx', phys.d, 'dy', 0.0);
    x_ref_M_sol = zeros(N,1);
    y_ref_M_sol = zeros(N,1);
    for k = 1:N
        [xM, yM, ~] = mm_mount_to_center( ...
            x_ref_B_sol(k), y_ref_B_sol(k), theta_ref_B_sol(k), offset);
        x_ref_M_sol(k) = xM;
        y_ref_M_sol(k) = yM;
    end

    % ----------------------------------
    % 4) Actual base kinematics (B frame): xdot, xddot, ...
    % ----------------------------------
    xdot_b   = gradient(x_b,  t_sol);
    ydot_b   = gradient(y_b,  t_sol);
    thetad_b = gradient(th_b, t_sol);

    xddot_b  = gradient(xdot_b,  t_sol);
    yddot_b  = gradient(ydot_b,  t_sol);
    thetadd_b= gradient(thetad_b,t_sol);

    % ----------------------------------
    % 5) Colors (same as original dark theme)
    % ----------------------------------
    refColor_M     = [0.2 0.8 1.0];    % cyan  (center M)
    refColor_B     = [0.2 0.9 0.4];    % green (mount B)
    actualColor    = [1.0 0.5 0.0];    % bright orange
    refColor       = [0.2 0.9 0.8];    % generic ref
    errorColor     = [1.0 0.3 0.3];    % soft red
    errorNormColor = [1 1 1];          % white (for dark theme; still ok for light)

    %% ============================================================
    %  A) Base trajectory (B and M)
    %% ============================================================
    newFig('Base Trajectory');
    plot(x_ref_M_sol, y_ref_M_sol, 'Color', refColor_M, 'LineWidth', 2); hold on;
    plot(x_ref_B_sol, y_ref_B_sol, 'Color', refColor_B, 'LineWidth', 2);
    plot(x_b,         y_b,         'Color', actualColor, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('x [m]','Color',textColor);
    ylabel('y [m]','Color',textColor);
    legend('Ref (center M)','Ref (mount B)','Actual (B)', 'TextColor',textColor);
    title('Base Path in World Frame','Color',textColor);
    styleAxes(gca);

    %% ============================================================
    %  B) Base orientation & commanded speeds (reference)
    %% ============================================================
    newFig('Base Orientation & Velocities');

    subplot(3,1,1);
    plot(t_sol, theta_ref_B_sol, 'Color', refColor,    'LineWidth',1.5); hold on;
    plot(t_sol, th_b,            'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('\theta_B [rad]','Color',textColor);
    legend('Ref (B)','Actual (B)','TextColor',textColor);
    title('Base Orientation (mount frame B)','Color',textColor);
    styleAxes(gca);

    subplot(3,1,2);
    plot(t_sol, v_ref_sol, 'Color', refColor, 'LineWidth',1.5);
    grid on; ylabel('v_r [m/s]','Color',textColor);
    title('Linear Velocity (reference)','Color',textColor);
    styleAxes(gca);

    subplot(3,1,3);
    plot(t_sol, omega_ref_sol, 'Color', refColor, 'LineWidth',1.5);
    grid on; ylabel('\omega_r [rad/s]','Color',textColor);
    xlabel('t [s]','Color',textColor);
    title('Angular Velocity (reference)','Color',textColor);
    styleAxes(gca);

    %% ============================================================
    %  C) Actual base kinematics in B: x,y,theta, xdot,xddot, ...
    %% ============================================================
    % Positions
    newFig('Base Positions (B frame)');
    subplot(3,1,1);
    plot(t_sol, x_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('x_B [m]','Color',textColor);
    title('Base Position (B frame)','Color',textColor);
    styleAxes(gca);

    subplot(3,1,2);
    plot(t_sol, y_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('y_B [m]','Color',textColor);
    styleAxes(gca);

    subplot(3,1,3);
    plot(t_sol, th_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('$\theta_B$ [rad]','Color',textColor, Interpreter='latex');
    xlabel('t [s]','Color',textColor);
    styleAxes(gca);

    % Velocities
    newFig('Base Velocities (B frame)');
    subplot(3,1,1);
    plot(t_sol, xdot_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('$\dot{x}_B$ [m/s]','Color',textColor, Interpreter='latex');
    title('Base Velocity (B frame)','Color',textColor);
    styleAxes(gca);

    subplot(3,1,2);
    plot(t_sol, ydot_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('$\dot{y}_B [m/s]$','Color',textColor, Interpreter='latex');
    styleAxes(gca);

    subplot(3,1,3);
    plot(t_sol, thetad_b, 'Color', actualColor, 'LineWidth',1.5);
    grid on; ylabel('$\dot{\theta}_B$ [rad/s]','Color',textColor, Interpreter='latex');
    xlabel('t [s]','Color',textColor);
    styleAxes(gca);

    % Accelerations
    newFig('Base Accelerations (B frame)');
    
    % --- x_B acceleration ---
    subplot(3,1,1);
    plot(t_sol, xddot_b, 'Color', actualColor, 'LineWidth', 1.5);
    grid on;
    ylabel('$\ddot{x}_{B}\,[\mathrm{m/s^2}]$', ...
           'Color', textColor, 'Interpreter', 'latex');
    title('Base Accelerations (B frame)', 'Color', textColor);
    styleAxes(gca);
    
    % --- y_B acceleration ---
    subplot(3,1,2);
    plot(t_sol, yddot_b, 'Color', actualColor, 'LineWidth', 1.5);
    grid on;
    ylabel('$\ddot{y}_{B}\,[\mathrm{m/s^2}]$', ...
           'Color', textColor, 'Interpreter', 'latex');
    styleAxes(gca);
    
    % --- theta_B acceleration ---
    subplot(3,1,3);
    plot(t_sol, thetadd_b, 'Color', actualColor, 'LineWidth', 1.5);
    grid on;
    ylabel('$\ddot{\theta}_{B}\,[\mathrm{rad/s^2}]$', ...
           'Color', textColor, 'Interpreter', 'latex');
    xlabel('t [s]', 'Color', textColor);
    styleAxes(gca);

    %% ============================================================
    %  D) Control efforts (if provided)
    %% ============================================================
    if ~isempty(tau_sol)
        tau_sol = tau_sol(:,:);   % N x 5
        newFig('Control Efforts (Torques)');

        subplot(2,1,1);  % wheels
        plot(t_sol, tau_sol(:,1), 'LineWidth',1.5); hold on;
        plot(t_sol, tau_sol(:,2), 'LineWidth',1.5);
        grid on;
        ylabel('$\tau_{wheel}$ [Nm]','Color',textColor, Interpreter='latex');
        legend('\tau_r','\tau_l','TextColor',textColor);
        title('Wheel Torques','Color',textColor);
        styleAxes(gca);

        subplot(2,1,2);  % arm joints
        plot(t_sol, tau_sol(:,3), 'LineWidth',1.5); hold on;
        plot(t_sol, tau_sol(:,4), 'LineWidth',1.5);
        plot(t_sol, tau_sol(:,5), 'LineWidth',1.5);
        grid on;
        ylabel('$\tau_{arm}$ [Nm]','Color',textColor, Interpreter='latex');
        xlabel('t [s]','Color',textColor);
        legend('\tau_1','\tau_2','\tau_3','TextColor',textColor);
        title('Arm Joint Torques','Color',textColor);
        styleAxes(gca);

        % Optional: instantaneous power (tau·eta_dot)
        P_inst = sum(tau_sol .* eta_dot_sol, 2);
        newFig('Instantaneous Power');
        plot(t_sol, P_inst, 'LineWidth',1.5);
        grid on;
        xlabel('t [s]','Color',textColor);
        ylabel('P [W] (approx)','Color',textColor);
        title('Control Power \tau^T \etȧ','Color',textColor);
        styleAxes(gca);
    end

    %% ============================================================
    %  E) Energy (if provided)
    %% ============================================================
    if ~isempty(E_sol)
        newFig('Mechanical Energy');
        plot(t_sol, E_sol, 'LineWidth',1.8);
        grid on;
        xlabel('t [s]','Color',textColor);
        ylabel('E [J]','Color',textColor);
        title('Total Mechanical Energy','Color',textColor);
        styleAxes(gca);
    end

    %% ============================================================
    %  F) Eta vs references & errors (same as before)
    %% ============================================================
    newFig('Eta vs Eta_ref');
    labels = {'\theta_r','\theta_l','q_1','q_2','q_3'};
    for i = 1:5
        subplot(5,1,i);
        plot(t_sol, eta_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on; ylabel(labels{i},'Color',textColor);

        if i==1
            title('$\eta$ trajectories (reference vs actual)','Color',textColor, Interpreter='latex');
            legend('Ref','Actual','TextColor',textColor);
        end
        if i==5
            xlabel('t [s]','Color',textColor);
        end
        styleAxes(gca);
    end

    %% ============================
    %   η_dot vs η_dot_ref
    % =============================
    newFig('Eta_dot vs Eta_dot_ref');
    
    for i = 1:5
        subplot(5,1,i);
    
        plot(t_sol, eta_dot_ref_all(:,i), 'Color', refColor, 'LineWidth',1.5); hold on;
        plot(t_sol, eta_dot_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
    
        grid on;
    
        % LaTeX label for each η_i dot
        ylabel(['$\dot{\eta}_{' num2str(i) '}$'], ...
               'Color',textColor,'Interpreter','latex');
    
        if i == 1
            title('$\dot{\eta}$ trajectories (reference vs actual)', ...
                  'Color',textColor,'Interpreter','latex');
    
            legend('Ref','Actual', ...
                   'TextColor',textColor,'Interpreter','latex','Location','best');
        end
    
        if i == 5
            xlabel('t [s]','Color',textColor);
        end
    
        styleAxes(gca);
    end
    
    
    %% ============================
    %   η_ddot vs η_ddot_ref
    % =============================
    newFig('Eta_ddot vs Eta_ddot_ref');
    
    for i = 1:5
        subplot(5,1,i);
    
        plot(t_sol, eta_ddot_ref_all(:,i), 'Color', refColor, 'LineWidth',1.5); hold on;
        plot(t_sol, eta_ddot_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
    
        grid on;
    
        % LaTeX label for each η_i double-dot
        ylabel(['$\ddot{\eta}_{' num2str(i) '}$'], ...
               'Color',textColor,'Interpreter','latex');
    
        if i == 1
            title('$\ddot{\eta}$ trajectories (reference vs actual)', ...
                  'Color',textColor,'Interpreter','latex');
    
            legend('Ref','Actual', ...
                   'TextColor',textColor,'Interpreter','latex','Location','best');
        end
    
        if i == 5
            xlabel('t [s]','Color',textColor);
        end
    
        styleAxes(gca);
    end

    newFig('Eta tracking error');
    subplot(2,1,1);
    plot(t_sol, e_eta, 'Color', errorColor, 'LineWidth', 1.2);
    grid on;
    title('$\eta$ Tracking Error (each DOF)','Color',textColor, Interpreter='latex');
    ylabel('error','Color',textColor);
    legend('\theta_r','\theta_l','q_1','q_2','q_3','TextColor',textColor);
    styleAxes(gca);

    subplot(2,1,2);
    plot(t_sol, e_eta_norm, 'Color', errorNormColor, 'LineWidth', 2);
    grid on;
    xlabel('t [s]','Color',textColor);
    ylabel('||\eta - \eta_{ref}||_2','Color',textColor);
    title('Norm of \eta tracking error','Color',textColor);
    styleAxes(gca);

end
