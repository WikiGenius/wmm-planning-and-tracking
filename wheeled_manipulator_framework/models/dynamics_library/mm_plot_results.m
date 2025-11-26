function mm_plot_results(t, X, t_ref, eta_d_all, U)
%MM_PLOT_RESULTS
% Plot base pose, joint/wheel motions, and control efforts.
%
% Inputs:
%   t         : simulation time vector (Nx1)
%   X         : state trajectory (Nx13)  [q; eta_dot]
%   t_ref     : time vector for desired eta (Nd x 1)
%   eta_d_all : desired eta(t_ref) (Nd x 5)
%   U         : control torques (Nx5)

    q       = X(:, 1:8);   % [x_b, y_b, theta_m, theta_r, theta_l, theta_1, theta_2, theta_3]
    eta_dot = X(:, 9:13);  % [theta_r_dot, ..., theta_3_dot]

    x_b     = q(:, 1);
    y_b     = q(:, 2);
    theta_m = q(:, 3);

    theta_r = q(:, 4);
    theta_l = q(:, 5);
    theta_1 = q(:, 6);
    theta_2 = q(:, 7);
    theta_3 = q(:, 8);

    % Desired wheels & joints (for overlay)
    theta_r_d = eta_d_all(:, 1);
    theta_l_d = eta_d_all(:, 2);
    theta_1_d = eta_d_all(:, 3);
    theta_2_d = eta_d_all(:, 4);
    theta_3_d = eta_d_all(:, 5);

    %% 1) Base trajectory in XY (O_b)
    figure;
    plot(x_b, y_b, 'LineWidth', 1.5); hold on; grid on;
    plot(x_b(1), y_b(1), 'ko', 'MarkerFaceColor', 'g');
    plot(x_b(end), y_b(end), 'ko', 'MarkerFaceColor', 'r');
    xlabel('x_b [m]');
    ylabel('y_b [m]');
    title('Base trajectory in XY (O_b)');
    axis equal;

    %% 2) Base orientation
    figure;
    plot(t, theta_m, 'LineWidth', 1.5); grid on;
    xlabel('Time [s]');
    ylabel('\theta_m [rad]');
    title('Base orientation');

    %% 3) Wheel angles vs reference
    figure;
    subplot(2,1,1);
    plot(t, theta_r, 'LineWidth', 1.5); hold on; grid on;
    plot(t_ref, theta_r_d, '--', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('\theta_r [rad]');
    legend('Actual', 'Desired');
    title('Right wheel angle');

    subplot(2,1,2);
    plot(t, theta_l, 'LineWidth', 1.5); hold on; grid on;
    plot(t_ref, theta_l_d, '--', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('\theta_l [rad]');
    legend('Actual', 'Desired');
    title('Left wheel angle');

    %% 4) Joint angles vs reference
    figure;
    subplot(3,1,1);
    plot(t, theta_1, 'LineWidth', 1.5); hold on; grid on;
    plot(t_ref, theta_1_d, '--', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('\theta_1 [rad]');
    legend('Actual', 'Desired');
    title('Joint 1 angle');

    subplot(3,1,2);
    plot(t, theta_2, 'LineWidth', 1.5); hold on; grid on;
    plot(t_ref, theta_2_d, '--', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('\theta_2 [rad]');
    legend('Actual', 'Desired');
    title('Joint 2 angle');

    subplot(3,1,3);
    plot(t, theta_3, 'LineWidth', 1.5); hold on; grid on;
    plot(t_ref, theta_3_d, '--', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('\theta_3 [rad]');
    legend('Actual', 'Desired');
    title('Joint 3 angle');

    %% 5) Control efforts
    figure;
    plot(t, U, 'LineWidth', 1.5); grid on;
    xlabel('Time [s]');
    ylabel('Torque [N·m]');
    legend('\tau_r', '\tau_l', '\tau_1', '\tau_2', '\tau_3', 'Location', 'best');
    title('Control efforts');
end
