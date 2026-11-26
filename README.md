# Wheeled Mobile Manipulator: Motion Planning and Control

How can a mobile base and an arm execute a coordinated motion while respecting the robot's kinematics and dynamics? What happens when a disturbance pushes the robot away from its reference?

I study these questions using a differential-drive mobile manipulator with two actuated wheels and a three-joint arm. The project connects motion planning, kinematic coordination and torque-level tracking, then investigates adaptive disturbance rejection.

<p align="center">
  <img src="adaptive-control/nominal-vs-adaptive.gif" alt="Animated comparison of the nominal and adaptive mobile manipulator trajectories" width="680">
</p>

*Simulation of the high asymmetric torque-disturbance case. The dashed reference is the axle path; blue is nominal feedback linearization and red is adaptive feedback linearization. Both robots execute the same desired maneuver.*

[Run the adaptive demo](#run-the-demo) · [Three-layer architecture](#planning-to-control) · [Results](#adaptive-disturbance-rejection) · [Project report (PDF)](report/Muhammed_Elyamani_paper.pdf)

## Why this problem matters

The base and arm are mechanically coupled. A differential-drive base cannot move sideways at its axle, and a change in the arm configuration changes the coupled dynamics. A geometric path alone does not specify how to execute the motion. The planner must generate time-parameterized references compatible with the kinematics, and the controller must turn those references into actuator commands.

At the task level, different base and arm configurations can lead to the same end-effector pose. The original framework studies a prescribed base/arm maneuver; it does not solve a general redundant task-space optimization or obstacle-avoidance problem.

## Planning to control

The original framework is organized into three layers, from the task to the actuators.

| Layer | Input → output | Implementation |
| --- | --- | --- |
| **3 — Motion planning** | Start/goal base poses and arm angles → smooth position, velocity and acceleration references | Rotate–translate–rotate base path, quintic timing, joint-space arm planner |
| **2 — Kinematic coordination** | Reference and robot state → admissible wheel and joint rate commands | Kanayama-style base tracking and arm velocity PD |
| **1 — Dynamic tracking** | Desired reduced-coordinate motion and robot state → actuator torques | Reduced-coordinate feedback linearization |

<p align="center">
  <img src="media/demo/offline_motion_planner_block_diagram.png" alt="Original offline planning architecture" width="720">
</p>

*Original planning architecture. The implementation also has an optional online kinematic correction mode. Evaluating a prescribed reference online is not online replanning.*

### Model and controller

The robot has eight configuration coordinates and five independent wheel/arm rates. With $\eta=[\theta_r,\theta_l,q_1,q_2,q_3]^T$, the nonholonomic velocity constraint is represented by

$$
\dot q=R(q)\dot\eta.
$$

The original reduced model and torque controller are

$$
M_1(q)\ddot\eta+(C_1(q,\dot q)+D_\eta)\dot\eta+G_1(q)=B_1(q)\tau,
$$

$$
e=\eta-\eta_d,\qquad v=\ddot\eta_d-K_d\dot e-K_p e,
$$

$$
\tau=B_1(q)^{-1}\left[M_1(q)v+(C_1(q,\dot q)+D_\eta)\dot\eta+G_1(q)\right].
$$

The implementation uses a linear solve rather than forming the inverse explicitly. The planner uses quintic time scaling, $\sigma(s)=10s^3-15s^4+6s^5$, for zero velocity and acceleration at segment endpoints.

[Original controller](wheeled_manipulator_framework/control/controllers/mm_closed_loop_dynamics.m) · [Reference builder](wheeled_manipulator_framework/control/controllers/plannar_controller/mm_build_eta_reference.m) · [Derivations](report/report.tex)

## Adaptive disturbance rejection

The newer, self-contained benchmark uses a corrected axle-centered reduced model and the same offline reference for two controllers: nominal feedback linearization and feedback linearization with an online estimate of the matched actuator disturbance. It does **not** exercise the original Layer-2 kinematic correction loop or introduce an adaptive planner.

![Axle paths and angular tracking error under the high asymmetric disturbance](adaptive-control/comparison.png)

| Disturbance case | Nominal RMS error (rad) | Adaptive RMS error (rad) | Reduction |
| --- | ---: | ---: | ---: |
| Moderate constant | 0.754 | 0.041 | 94.53% |
| High asymmetric constant | 3.045 | 0.137 | 95.52% |
| Sinusoidal | 0.956 | 0.213 | 77.77% |

In the high case, the time-weighted RMS norm of the five angular tracking errors falls from **3.045 to 0.137 rad**, while RMS commanded torque norm rises from **12.26 to 13.64 N·m**. This comparison shows the tracking/effort trade-off for the tested model, not a general robustness guarantee. Net signed actuator work is reported separately; it is not electrical energy consumption.

[Adaptive formulation, assumptions and model checks](adaptive-control/README.md) · [Numerical results](adaptive-control/results.csv) · [Parameters](adaptive-control/parameters.json)

## Run the demo

Start with the self-contained adaptive benchmark. From the repository root in MATLAB:

```matlab
cd('adaptive-control')
results = run_demo();
```

The benchmark uses core MATLAB functions (previously tested with R2025a Update 1, as documented in the project). It runs model checks, simulates the three disturbance cases with both controllers, and exports the comparison plot and CSV. To see both robots in 3-D:

```matlab
run_demo(true);
% Optional: export a local GIF from the MATLAB simulation
run_demo(true, fullfile(pwd,'nominal-vs-adaptive-local.gif'));
```

The committed animation is a separate RK4 replay of the same high case; the MATLAB export uses ODE45 and a different visualization. See the [animation provenance](adaptive-control/README.md#animation).

For the earlier full-stack experiments, see the [legacy framework instructions](wheeled_manipulator_framework/README.md). That workflow uses MATLAB live scripts and generated model data, and is not yet verified from a clean clone.

## Original simulation experiments

These are historical report figures, not outputs of the newer adaptive benchmark.

| Planner only | Full stack under disturbance | Reduced-coordinate tracking error |
| --- | --- | --- |
| ![Planner-only base path](media/demo/planner_only_base_path.png) | ![Full stack base path under disturbance](media/demo/full_stack_base_path_disturbance.png) | ![Full stack angular tracking error](media/demo/full_stack_tracking_error_eta.png) |

The report considers dynamic tracking alone (E1), planner-only references (E2), disturbed execution without full feedback compensation (E3), and the full stack under disturbance (E4). See the [report](report/Muhammed_Elyamani_paper.pdf) and [saved figures](report/figures).

## Repository guide

| Path | Contents |
| --- | --- |
| [adaptive-control/](adaptive-control/) | Self-contained benchmark, model checks, adaptive controller, results and animation |
| [wheeled_manipulator_framework/](wheeled_manipulator_framework/) | Original three-layer MATLAB framework, symbolic/generated dynamics, controllers and tests |
| [report/](report/) | Project manuscript, LaTeX source and historical figures |
| [presentations/](presentations/) | Planning and tracking presentations |
| [experiments/](experiments/) · [mobile_robot/](mobile_robot/) | Earlier exploratory modeling and mobile-robot exercises |
| [media/demo/](media/demo/) | Original project diagrams and selected result figures |

## Scope and limitations

- All shown results are simulations with prescribed references; no physical robot validation is claimed.
- The adaptive benchmark assumes exact model knowledge, effective rotational inertias, matched actuator disturbances and unsaturated commands. Its tests cover model identities and three deterministic cases.
- The legacy workflow still requires generated MAT files and path cleanup. Its online kinematic correction mode also needs a consistent reference-derivative and torque-logging audit before claiming a fully reproduced closed-loop result.
- Task-space optimization, collision avoidance, actuator limits, state-estimation errors and hardware validation remain outside the demonstrated scope.

## Credits

Original project report: Muhammed Elyamani, Ali Attyatullah Mohammed AlBeladi and Brahim Brahmi, *Motion Planning and Control of a Differential-Drive Mobile Manipulator*.

The adaptive benchmark develops Muhammed Elyamani's SCE 527 project on the earlier framework. See the [project report](report/Muhammed_Elyamani_paper.pdf) and [adaptive notes](adaptive-control/README.md). Publication status is not asserted.
