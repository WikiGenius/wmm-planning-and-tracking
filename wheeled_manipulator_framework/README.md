# Original three-layer mobile manipulator framework

This folder preserves the original planning, kinematic coordination and dynamic tracking implementation and its report experiments. For a self-contained MATLAB benchmark, start with [adaptive-control](../adaptive-control/README.md). The adaptive model is a later axle-centered revision, not an interchangeable drop-in model for this framework.

## Architecture and entry points

| Layer | Implementation |
| --- | --- |
| 3 — Planner | [Base planner](control/controllers/plannar_controller/mm_traj_planner_base.m), [arm planner](control/controllers/plannar_controller/mm_traj_planner_arm.m), [reference builder](control/controllers/plannar_controller/mm_build_eta_reference.m) |
| 2 — Kinematic coordination | [Base Kanayama controller](control/controllers/kinematic_controller/mm_kin_ctrl_base_kanayama.m), [arm velocity PD](control/controllers/kinematic_controller/mm_kin_ctrl_arm_pd.m) |
| 1 — Dynamic tracking | [Feedback linearization](control/controllers/low_level_control_tracker/mm_fb_lin_tracker.m), [reduced forward dynamics](models/dynamics_library/mm_forward_dynamics_reduced.m) |

The [closed-loop dynamics](control/controllers/mm_closed_loop_dynamics.m) selects whether to use the precomputed reference alone or add online kinematic velocity corrections. The [controller initializer](control/controllers/mm_controller_init.m) currently sets the online flag to true.

## Run the original controller

From the repository root, run `matlab -batch "reproduce"` for the numerical benchmark and the legacy trajectory. For the original plots, run the following in MATLAB:

```matlab
addpath(fullfile(pwd,'wheeled_manipulator_framework','control','controllers'))
trajectory = mm_main_controller();
```

The driver returns time, 13 state channels, five commanded torques and cumulative signed actuator work. The torque post-processor evaluates the same feedback command used during integration, including online velocity corrections. The injected disturbance is excluded from commanded work.

The public checkout includes the original runtime inputs in `models/data/`. The five generated model functions and parameter JSON were matched against the source archive before restoring the two MAT files. Symbolic regeneration is a separate authoring workflow. For the full-stack live script run `run('wheeled_manipulator_framework/control/main_controller.mlx')`. For the standalone model-side tracking demonstration run `run('wheeled_manipulator_framework/models/mm_main_fb_lin_tracking.mlx')`. Both commands start at the repository root. The root `setup_project.m` is the historical scaffold generator.

## Historical experiments

| Group | Purpose |
| --- | --- |
| E1 | Dynamic tracking with a supplied reference |
| E2 | Smooth open-loop planner references |
| E3 | Disturbed execution without full feedback compensation |
| E4 | Full stack under disturbance |

See the [original report](../report/Muhammed_Elyamani_paper.pdf), [figures](../report/figures/) and [controller tests](control/test_cases/).

## Remaining technical checks

- Check consistency of position, velocity and acceleration references in online correction mode; the current implementation changes velocity while retaining the offline position and acceleration references.
- Audit mount/axle frame consistency and boundary cases; do not transfer the adaptive benchmark's numerical checks to this model.

Credit: Muhammed Elyamani, Ali Attyatullah Mohammed AlBeladi and Brahim Brahmi, *Motion Planning and Control of a Differential-Drive Mobile Manipulator*. The archived report documents the original implementation.
