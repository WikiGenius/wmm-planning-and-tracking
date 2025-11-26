function ctrl = mm_controller_init(phys, traj)
%MM_CONTROLLER_INIT
%  Centralized controller setup:
%    - dynamic (η-level) gains
%    - kinematic (base + arm) gains
%
%  NOTE:
%    This function must define ONLY tuning parameters.
%    NO trajectory generation.
%    NO η-reference generation.
%    NO planning logic.
%
%  The logic is now clean:
%     mm_scenario_basic      → defines the task
%     mm_traj_planner_*      → produces desired geometric trajectory
%     mm_kin_ctrl_*          → produces desired η̇_d
%     mm_fb_lin_tracker      → dynamics + torque tracking

    ctrl = struct();
    ctrl.phys = phys;
    ctrl.traj = traj;

    %% ============================================================
    % 1) Feedback–linearization gains (η coordinates)
    %% ============================================================
    wn_base = 2.0;   % natural freq for wheel DOFs
    wn_arm  = 1.5;   % natural freq for arm DOFs
    zeta    = 1.0;   % critical damping

    Kp_eta = diag([wn_base^2 wn_base^2 wn_arm^2 wn_arm^2 wn_arm^2]);
    Kd_eta = diag([2*zeta*wn_base 2*zeta*wn_base ...
                   2*zeta*wn_arm  2*zeta*wn_arm  2*zeta*wn_arm]);

    ctrl.gains.eta.Kp = Kp_eta;
    ctrl.gains.eta.Kd = Kd_eta;

    %% ============================================================
    % 2) Base kinematic tracking gains (Kanayama)
    %% ============================================================
    % ctrl.gains.base_kin = struct( ...
    %     'Kx',  0.15, ...
    %     'Ky',  0.20, ...
    %     'Kth', 0.1);
    % ctrl.gains.base_kin = struct( ...
    %     'Kx',  10000, ...
    %     'Ky',  10000, ...
    %     'Kth', 0.001);
ctrl.gains.base_kin = struct( ...
    'Kx',  5.0, ...
    'Ky',  5.0, ...
    'Kth', 2.0);


    %% ============================================================
    % 3) Arm kinematic PD gains
    %% ============================================================
    ctrl.gains.arm_kin = struct( ...
        'Kp', diag([2 2 2]), ...
        'Kd', diag([1 1 1]) );

    %% ============================================================
    % 4) No η reference or planning here
    %% ============================================================
    % Planning happens in:
    %   mm_traj_planner_base_arm
    % η-reference for plots can be computed outside if needed.

end
