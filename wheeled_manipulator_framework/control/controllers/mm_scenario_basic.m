function traj = mm_scenario_basic()
%MM_SCENARIO_BASIC
%  Scenario / task configuration for the mobile manipulator.
%  All "what to do" settings live here.

    % Total task duration
    traj.Tf = 10;      % [s]

    % Base pose (SE(2)) start & goal
    traj.p0 = [0; 0; 0];          % [x0; y0; theta0]
    traj.pf = [1; 1; pi];       % [xf; yf; thetaf]

    % (Optional) nominal desired base speeds (can be used in planner)
    traj.v_des_base     = 0.5;    % [m/s]  (not enforced yet)
    traj.omega_des_base = 0.4;    % [rad/s]

    % Arm joint boundary conditions
    traj.q0_arm   = [0.0; -0.3;  0.5];   % initial joint positions
    traj.qf_arm   = [2.0;  0.2; -0.4];   % final joint positions

    traj.qd0_arm  = [0; 0; 0];           % initial joint velocities
    traj.qdf_arm  = [0; 0; 0];           % final joint velocities

    traj.qdd0_arm = [0; 0; 0];           % initial joint accelerations
    traj.qddf_arm = [0; 0; 0];           % final joint accelerations
end
