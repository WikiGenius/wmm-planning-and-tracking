function traj = mm_scenario_basic(phys)
%MM_SCENARIO_BASIC
%  Scenario / task configuration for the mobile manipulator.
%
%  Responsibilities:
%    - Define task-level data (what to do, not how).
%    - Take user base poses in either B or M.
%    - Produce:
%         * planner poses in center frame M  (xm, ym, theta_m)
%         * controller poses in mount frame B (xb, yb, theta_b)
%
%  Frames:
%    - B : mounting frame (xb, yb, theta_b)  [where the arm is mounted]
%    - M : base center frame (xm, ym, theta_m)
%
%  Output (key fields):
%    traj.base_input_frame : 'B' or 'M' (user choice)
%    traj.p0_M, traj.pf_M  : start/goal in center frame  (for planner)
%    traj.p0_B, traj.pf_B  : start/goal in mount frame   (for controller)
%    traj.p0,  traj.pf     : alias to M-frame poses      (for old code)
%    traj.offset_MB        : struct with dx, dy from M → B (body frame)

    traj = struct();

    %% --------------------------------------------------------------
    % 1) Total task duration
    %% --------------------------------------------------------------
    traj.Tf = 10.0;   % [s]

    %% --------------------------------------------------------------
    % 2) Base pose (SE(2)) start & goal
    %% --------------------------------------------------------------
    % Frame in which YOU specify the poses: 'B' (mount) or 'M' (center)
    traj.base_input_frame = 'B';   % change to 'M' if you think in center frame

    % User-specified start / goal pose [x; y; theta] in base_input_frame
    traj.p0_user = [0.0; 0.0; 0.0];   % [x0; y0; theta0]
    traj.pf_user = [2.0; 2.0; pi/2];    % [xf; yf; thetaf]

    % Offset of mounting point B relative to center M, expressed in body frame
    traj.offset_MB = struct('dx', phys.d, 'dy', 0.0);

    % --- Convert once to get BOTH frames ---------------------------
    switch traj.base_input_frame
        case 'B'
            % User gave poses at the mount → copy to B frame
            xb0 = traj.p0_user(1);  yb0 = traj.p0_user(2);  thb0 = traj.p0_user(3);
            xbf = traj.pf_user(1);  ybf = traj.pf_user(2);  thbf = traj.pf_user(3);

            % Convert to center frame M for planning
            [xm0, ym0, th0] = mm_mount_to_center(xb0, yb0, thb0, traj.offset_MB);
            [xmf, ymf, thf] = mm_mount_to_center(xbf, ybf, thbf, traj.offset_MB);

        case 'M'
            % User gave poses at the center → copy to M frame
            xm0 = traj.p0_user(1);  ym0 = traj.p0_user(2);  th0 = traj.p0_user(3);
            xmf = traj.pf_user(1);  ymf = traj.pf_user(2);  thf = traj.pf_user(3);

            % Convert to mount frame B for controller
            [xb0, yb0, thb0] = mm_center_to_mount(xm0, ym0, th0, traj.offset_MB);
            [xbf, ybf, thbf] = mm_center_to_mount(xmf, ymf, thf, traj.offset_MB);

        otherwise
            error('mm_scenario_basic: Unknown traj.base_input_frame = %s. Use ''B'' or ''M''.', ...
                  traj.base_input_frame);
    end

    % Store the canonical versions

    % For planner (center frame M)
    traj.p0_M = [xm0; ym0; th0];
    traj.pf_M = [xmf; ymf; thf];

    % For controller (mount frame B)
    traj.p0_B = [xb0; yb0; thb0];
    traj.pf_B = [xbf; ybf; thbf];

    % Backward-compatible aliases: planner always uses M-frame
    traj.p0 = traj.p0_M;
    traj.pf = traj.pf_M;

    % Nominal base speeds (used by planner to distribute segment times)
    traj.v_des_base     = 5;   % [m/s]   nominal forward speed
    traj.omega_des_base = 5;   % [rad/s] nominal yaw rate

    %% --------------------------------------------------------------
    % 3) Arm joint boundary conditions
    %% --------------------------------------------------------------
    traj.q0_arm = [ 0.0; -0.3;  0.5];   % initial joint positions
    traj.qf_arm = [ 2.0;  0.2; -0.4];   % final joint positions

    traj.qd0_arm = [0.0; 0.0; 0.0];     % initial joint velocities
    traj.qdf_arm = [0.0; 0.0; 0.0];     % final joint velocities

    traj.qdd0_arm = [0.0; 0.0; 0.0];    % initial joint accelerations
    traj.qddf_arm = [0.0; 0.0; 0.0];    % final joint accelerations
end
