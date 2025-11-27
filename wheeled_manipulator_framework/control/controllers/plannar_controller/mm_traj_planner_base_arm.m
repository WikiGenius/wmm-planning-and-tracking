function [base_ref, arm_ref] = mm_traj_planner_base_arm(t, traj, phys)
%MM_TRAJ_PLANNER_BASE_ARM
%  Wrapper that combines:
%    - SE(2) base planner
%    - 3-DOF arm planner
%
%  Keeps backward compatibility with existing code.

    base_ref = mm_traj_planner_base(t, traj, phys);
    arm_ref  = mm_traj_planner_arm(t, traj);
end
