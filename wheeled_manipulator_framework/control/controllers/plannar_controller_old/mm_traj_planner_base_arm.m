function [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj)
%MM_TRAJ_PLANNER_BASE_ARM
%  High-level wrapper:
%    - Base: SE(2) geodesic planner with time-scaling & velocity limits
%    - Arm : per-joint quintic planner with full boundary conditions.
%
%  Inputs:
%    t    : time
%    traj : struct with fields
%           .Tf, .p0, .pf,
%           .v_des_base, .omega_des_base (optional),
%           .q0_arm, .qf_arm, .qd0_arm, .qdf_arm, .qdd0_arm, .qddf_arm
%
%  Outputs:
%    base_ref : struct with base trajectory quantities
%    arm_ref  : struct with arm joint trajectories

    Tf = traj.Tf;
    t  = min(max(t,0), Tf);   % clamp into [0, Tf]

    % --- Base (SE(2) geodesic + time-scaling) ---
    base_ref = mm_planner_base_se2_geodesic(t, traj);

    % --- Arm (per-joint quintic) ---
    arm_ref  = mm_planner_arm_quintic(t, Tf, traj);
end
