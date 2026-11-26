# Original three-layer mobile manipulator framework

This folder preserves the original planning, kinematic coordination and dynamic tracking implementation and its report experiments. For a self-contained MATLAB benchmark, start with [adaptive-control](../adaptive-control/README.md). The adaptive model is a later axle-centered revision, not an interchangeable drop-in model for this framework.

## Architecture and entry points

| Layer | Implementation |
| --- | --- |
| 3 — Planner | [Base planner](control/controllers/plannar_controller/mm_traj_planner_base.m), [arm planner](control/controllers/plannar_controller/mm_traj_planner_arm.m), [reference builder](control/controllers/plannar_controller/mm_build_eta_reference.m) |
| 2 — Kinematic coordination | [Base Kanayama controller](control/controllers/kinematic_controller/mm_kin_ctrl_base_kanayama.m), [arm velocity PD](control/controllers/kinematic_controller/mm_kin_ctrl_arm_pd.m) |
| 1 — Dynamic tracking | [Feedback linearization](control/controllers/low_level_control_tracker/mm_fb_lin_tracker.m), [reduced forward dynamics](models/dynamics_library/mm_forward_dynamics_reduced.m) |

The [closed-loop dynamics](control/controllers/mm_closed_loop_dynamics.m) selects whether to use the precomputed reference alone or add online kinematic velocity corrections. The [controller initializer](control/controllers/mm_controller_init.m) currently sets the online flag to true.

## Original workflow (reproduction not yet verified)

The original entry points are [control/main_controller.mlx](control/main_controller.mlx) and [models/9_mm_main_fb_lin_tracking.mlx](models/9_mm_main_fb_lin_tracking.mlx). The function [mm_main_controller.m](control/controllers/mm_main_controller.m) is another driver. MATLAB and any toolboxes used by the original live scripts are needed, including Symbolic Math Toolbox for regeneration of symbolic dynamics. The original base planner calls wrapToPi; its availability must also be checked.

From the repository root, add the framework to the MATLAB path:

```matlab
addpath(genpath('wheeled_manipulator_framework'));
```

Before executing the historical live scripts, prepare their generated model data. The repository tracks generated numeric functions and parameter JSON but does not track the required mm_params.mat and mm_dynamics_reduced.mat files (MAT files are ignored). Review the [parameter builder](models/dynamics_library/mm_params_build_save_load.m) and the model-generation live scripts in [models/](models/). The data must be generated or restored to a location accessible to MATLAB. The function driver also contains relative path assumptions that need correction for a clean checkout. The top-level setup_project.m is an older starter scaffold, not an installer for the complete framework.

## Historical experiments

| Group | Purpose |
| --- | --- |
| E1 | Dynamic tracking with a supplied reference |
| E2 | Smooth open-loop planner references |
| E3 | Disturbed execution without full feedback compensation |
| E4 | Full stack under disturbance |

See the [original report](../report/Muhammed_Elyamani_paper.pdf), [figures](../report/figures/) and [controller tests](control/test_cases/).

## Remaining technical checks

- Restore or regenerate model MAT files and verify the full-stack run from a fresh clone.
- Check consistency of position, velocity and acceleration references in online correction mode; the current implementation changes velocity while retaining the offline position and acceleration references.
- Compute torque and signed mechanical work from the actual online controller commands: the existing post-processor reconstructs torques from offline references, so online-mode energy figures require verification.
- Audit mount/axle frame consistency and boundary cases; do not transfer the adaptive benchmark's numerical checks to this model.

Credit: Muhammed Elyamani, Ali Attyatullah Mohammed AlBeladi and Brahim Brahmi, *Motion Planning and Control of a Differential-Drive Mobile Manipulator*. The archived report documents the original implementation.
