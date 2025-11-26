function mm_conditioning_scan()
%MM_CONDITIONING_SCAN
% Scan over (theta_2, theta_3) space and evaluate:
%   - cond(B1(q))
%   - cond(Gamma(q)) where Gamma = B1^{-1} * M1
%
% This helps you see where feedback linearization will require huge
% torques (ill-conditioned decoupling) → "singular" or near-singular
% configurations for control.

% Where log10(cond(Gamma)) is small (e.g. < 2) → good conditioning.
% Where it's large (e.g. > 4–6) → small changes in virtual input produce huge torques → avoid these joint configurations or use very low gains.
    

    %% Load parameters (for completeness, although we only need them if
    %  mm_M1_fun / mm_B1_fun internally use phys)
    [phys, ~] = mm_params_build_save_load('load', 'mm_params'); %#ok<NASGU>

    %% Grid of arm joint angles
    n2 = 50;             % number of points in theta_2
    n3 = 50;             % number of points in theta_3

    theta2_vec = linspace(-pi/2,  pi/2, n2);   % you can adjust
    theta3_vec = linspace(-pi/2,  pi/2, n3);

    % We'll fix other coordinates:
    x_b0     = 0;
    y_b0     = 0;
    theta_m0 = 0;
    theta_r0 = 0;
    theta_l0 = 0;
    theta_1_0 = 0;

    condB  = zeros(n2, n3);
    condG  = zeros(n2, n3);
    sigma_min_B = zeros(n2, n3);
    sigma_min_G = zeros(n2, n3);

    %% Scan grid
    for i2 = 1:n2
        theta2 = theta2_vec(i2);
        for i3 = 1:n3
            theta3 = theta3_vec(i3);

            % Build configuration q
            q = [ ...
                x_b0; y_b0; theta_m0;
                theta_r0; theta_l0;
                theta_1_0;
                theta2;
                theta3 ];

            % Get matrices
            M1 = mm_M1_fun(q);      % 5x5
            B1 = mm_B1_fun(q);      % 5x5

            % --- B1 conditioning ---
            sB = svd(B1);
            condB(i2, i3)    = max(sB) / min(sB);
            sigma_min_B(i2, i3) = min(sB);

            % --- Gamma = B1^{-1} * M1 conditioning ---
            Gamma = B1 \ M1;        % same as inv(B1)*M1 but more stable
            sG   = svd(Gamma);
            condG(i2, i3)    = max(sG) / min(sG);
            sigma_min_G(i2, i3) = min(sG);
        end
    end

    %% Visualization: condition number heatmaps
    [T2, T3] = meshgrid(theta3_vec, theta2_vec); % note order for plotting

    figure;
    surf(T2, T3, log10(condB)); shading interp; colorbar;
    xlabel('\theta_3 [rad]');
    ylabel('\theta_2 [rad]');
    zlabel('log_{10}(cond(B_1))');
    title('Condition number of B_1(q)');

    figure;
    surf(T2, T3, log10(condG)); shading interp; colorbar;
    xlabel('\theta_3 [rad]');
    ylabel('\theta_2 [rad]');
    zlabel('log_{10}(cond(\Gamma))');
    title('Condition number of \Gamma(q) = B_1^{-1}M_1');

    %% Optional: highlight "dangerous" regions
    figure;
    imagesc(theta3_vec, theta2_vec, log10(condG));
    set(gca,'YDir','normal');
    colorbar;
    xlabel('\theta_3 [rad]');
    ylabel('\theta_2 [rad]');
    title('log_{10}(cond(\Gamma(q)))  (heatmap)');
end
