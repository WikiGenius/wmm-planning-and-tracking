function mm_inspect_modes_at_q(q)
%MM_INSPECT_MODES_AT_Q
% Inspect conditioning and modal structure at a single configuration q.
%
% q layout:
%   q = [x_b; y_b; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3]

    if nargin < 1
        % default configuration if not provided
        q = [0;0;0; 0;0; 0; 0; 0];
    end

    fprintf('--- Inspecting configuration q ---\n');
    fprintf('q = [x_b; y_b; theta_m; theta_r; theta_l; theta_1; theta_2; theta_3]\n');
    disp(q.');

    % Get matrices
    M1 = mm_M1_fun(q);   % 5x5
    B1 = mm_B1_fun(q);   % 5x5

    % Decoupling matrix Gamma = B1^{-1} * M1
    Gamma = B1 \ M1;

    %% --- B1 analysis ---
    [U_B, S_B, V_B] = svd(B1);
    sB = diag(S_B);
    condB = max(sB) / min(sB);

    fprintf('\nB1(q) singular values:\n');
    disp(sB.');
    fprintf('cond(B1) = %.3e\n', condB);

    %% --- Gamma analysis ---
    [U_G, S_G, V_G] = svd(Gamma);
    sG = diag(S_G);
    condG = max(sG) / min(sG);

    fprintf('\nGamma(q) = B1^{-1}M1 singular values:\n');
    disp(sG.');
    fprintf('cond(Gamma) = %.3e\n', condG);

    %% --- Eigenvalues of M1 (inertia) ---
    lambda_M = eig(M1);
    fprintf('\nEigenvalues of inertia M1(q):\n');
    disp(lambda_M.');

    %% --- Visualize modes ---
    % Interpret columns of V_G as "input directions" (torque combinations)
    % and columns of U_G as "acceleration directions" (in eta-space).
    %
    % Index mapping for eta, tau:
    labels = {'\theta_r','\theta_l','\theta_1','\theta_2','\theta_3'};

    figure;
    subplot(2,1,1);
    bar(V_G); grid on;
    xlabel('Torque index i (input DOF)');
    ylabel('Component');
    legend(labels, 'Location','bestoutside');
    title('Columns of V_G: principal torque directions for v-modes');

    subplot(2,1,2);
    bar(U_G); grid on;
    xlabel('Acceleration index i (eta DOF)');
    ylabel('Component');
    legend(labels, 'Location','bestoutside');
    title('Columns of U_G: principal acceleration directions for v-modes');

    % Also plot singular values of Gamma
    figure;
    stem(1:5, sG, 'filled'); grid on;
    xlabel('Mode index');
    ylabel('Singular value \sigma_i(\Gamma)');
    title(sprintf('Singular values of \\Gamma at this q (cond = %.2e)', condG));
end
