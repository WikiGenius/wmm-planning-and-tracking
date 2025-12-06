function mm_plot_results_controller_arm_only(t_sol, X_sol, ctrl, base_ref_all, arm_ref_all, phys, varargin)
%MM_PLOT_RESULTS_CONTROLLER (ARM-FOCUSED VERSION)
%  Visualization focused on the 3-DOF arm:
%    - Arm joint angles q1,q2,q3 vs reference
%    - Arm joint velocities vs reference
%    - Arm joint accelerations vs reference (approx from gradient)
%    - Arm control efforts (torques) [optional]
%    - Arm-only instantaneous power [optional]
%    - Arm tracking errors (q, qdot, qddot)
%
%  NOTE:
%    - Inputs base_ref_all and phys are kept for compatibility
%      but are NOT used in this arm-focused visualization.
%
%  Inputs:
%    t_sol        : time vector from ODE45 (Nx1 or 1xN)
%    X_sol        : state trajectory (N x 13)
%    ctrl         : controller struct (must contain ctrl.ref)
%    base_ref_all : (unused here, kept for interface compatibility)
%    arm_ref_all  : (currently unused here, kept for future)
%    phys         : (unused here, kept for interface compatibility)
%
%    varargin:
%      {1} tau_sol (optional) : N x 5 control torques
%      {2} E_sol   (optional) : N x 1 total mechanical energy

    % -------- optional inputs --------
    tau_sol = [];
    E_sol   = [];
    if numel(varargin) >= 1 && ~isempty(varargin{1})
        tau_sol = varargin{1};
    end
    if numel(varargin) >= 2 && ~isempty(varargin{2})
        E_sol = varargin{2};
    end

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

    % Arm-only slices in η:
    % indices: 3 = q1, 4 = q2, 5 = q3
    idx_arm = 3:5;

    eta_arm_sol     = eta_sol(:, idx_arm);       % N x 3  → [q1 q2 q3]
    eta_dot_arm_sol = eta_dot_sol(:, idx_arm);   % N x 3  → [q1dot q2dot q3dot]

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

    % Arm-only references
    eta_arm_ref_all      = eta_ref_all(:, idx_arm);      % N x 3
    eta_dot_arm_ref_all  = eta_dot_ref_all(:, idx_arm);  % N x 3
    eta_ddot_arm_ref_all = eta_ddot_ref_all(:, idx_arm); % N x 3

    % approximate actual η̈ via gradient of η̇ (for plotting only)
    eta_ddot_arm_sol = zeros(N, 3);
    for i = 1:3
        eta_ddot_arm_sol(:,i) = gradient(eta_dot_arm_sol(:,i), t_sol);
    end

    % tracking errors in arm joint space
    e_arm      = eta_arm_sol - eta_arm_ref_all;  % N x 3
    e_arm_norm = vecnorm(e_arm, 2, 2);           % N x 1

    % ----------------------------------
    % 3) Colors (white theme friendly)
    % ----------------------------------
    % Use colors that are clear on white background
    refColor       = [0.0000 0.4470 0.7410];   % MATLAB default blue (Ref)
    actualColor    = [0.8500 0.3250 0.0980];   % MATLAB default orange (Actual)
    errorColor     = [0.6350 0.0780 0.1840];   % Dark red for errors
    errorNormColor = [0.0000 0.0000 0.0000];   % Black for norm

    %% ============================================================
    %  A) Arm joint angles q1,q2,q3 vs reference
    %% ============================================================
    figure('Name','Arm Joint Angles vs Reference','NumberTitle','off');

    joint_labels = {'q_1','q_2','q_3'};
    for i = 1:3
        subplot(3,1,i);
        plot(t_sol, eta_arm_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_arm_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on;
        ylabel(['$' joint_labels{i} '\,[\mathrm{rad}]$'], ...
               'Interpreter','latex');

        if i == 1
            title('Arm Joint Angles (reference vs actual)', ...
                  'Interpreter','latex');
            legend('Ref','Actual','Interpreter','latex','Location','best');
        end
        if i == 3
            xlabel('t [s]');
        end
    end

    %% ============================================================
    %  B) Arm joint velocities vs reference
    %% ============================================================
    figure('Name','Arm Joint Velocities vs Reference','NumberTitle','off');

    for i = 1:3
        subplot(3,1,i);
        plot(t_sol, eta_dot_arm_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_dot_arm_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on;

        ylabel(['$\dot{' joint_labels{i} '}\,[\mathrm{rad/s}]$'], ...
               'Interpreter','latex');

        if i == 1
            title('Arm Joint Velocities (reference vs actual)', ...
                  'Interpreter','latex');
            legend('Ref','Actual','Interpreter','latex','Location','best');
        end
        if i == 3
            xlabel('t [s]');
        end
    end

    %% ============================================================
    %  C) Arm joint accelerations vs reference
    %% ============================================================
    figure('Name','Arm Joint Accelerations vs Reference','NumberTitle','off');

    for i = 1:3
        subplot(3,1,i);
        plot(t_sol, eta_ddot_arm_ref_all(:,i), 'Color', refColor,    'LineWidth',1.5); hold on;
        plot(t_sol, eta_ddot_arm_sol(:,i),     'Color', actualColor, 'LineWidth',1.5);
        grid on;

        ylabel(['$\ddot{' joint_labels{i} '}\,[\mathrm{rad/s^2}]$'], ...
               'Interpreter','latex');

        if i == 1
            title('Arm Joint Accelerations (reference vs actual)', ...
                  'Interpreter','latex');
            legend('Ref','Actual','Interpreter','latex','Location','best');
        end
        if i == 3
            xlabel('t [s]');
        end
    end

    %% ============================================================
    %  D) Control efforts (ARM ONLY, if provided)
    %% ============================================================
    if ~isempty(tau_sol)
        tau_sol = tau_sol(:,:);   % N x 5
        figure('Name','Arm Joint Torques','NumberTitle','off');

        % Arm torques are columns 3,4,5
        plot(t_sol, tau_sol(:,3), 'LineWidth',1.5); hold on;
        plot(t_sol, tau_sol(:,4), 'LineWidth',1.5);
        plot(t_sol, tau_sol(:,5), 'LineWidth',1.5);
        grid on;
        ylabel('$\tau_{arm}$ [Nm]', 'Interpreter','latex');
        xlabel('t [s]');
        legend('\tau_1','\tau_2','\tau_3','Interpreter','latex','Location','best');
        title('Arm Joint Torques','Interpreter','latex');

        % Arm-only instantaneous power: tau_arm^T * qdot_arm
        P_arm_inst = sum(tau_sol(:,3:5) .* eta_dot_arm_sol, 2);
        figure('Name','Arm Instantaneous Power','NumberTitle','off');
        plot(t_sol, P_arm_inst, 'LineWidth',1.5);
        grid on;
        xlabel('t [s]');
        ylabel('P_{arm} [W] (approx)');
        title('Arm Control Power $\tau_{arm}^T \dot{q}$', ...
              'Interpreter','latex');
    end

    %% ============================================================
    %  E) Energy (if provided) – total mechanical energy
    %% ============================================================
    if ~isempty(E_sol)
        figure('Name','Mechanical Energy','NumberTitle','off');
        plot(t_sol, E_sol, 'LineWidth',1.8);
        grid on;
        xlabel('t [s]');
        ylabel('E [J]');
        title('Total Mechanical Energy','Interpreter','latex');
    end

    %% ============================================================
    %  F) Arm tracking error (q-space)
    %% ============================================================
    figure('Name','Arm Tracking Error (q-space)','NumberTitle','off');

    subplot(2,1,1);
    plot(t_sol, e_arm, 'LineWidth', 1.2);
    grid on;
    title('Arm Joint Tracking Error (q_1, q_2, q_3)', ...
          'Interpreter','latex');
    ylabel('error [rad]');
    legend('q_1','q_2','q_3','Interpreter','latex','Location','best');

    subplot(2,1,2);
    plot(t_sol, e_arm_norm, 'Color', errorNormColor, 'LineWidth', 2);
    grid on;
    xlabel('t [s]');
    ylabel('$\|q - q_{\mathrm{ref}}\|_2$','Interpreter','latex');
    title('Norm of Arm Tracking Error','Interpreter','latex');

end
