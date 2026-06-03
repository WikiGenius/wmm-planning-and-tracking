# Wheeled Mobile Manipulator Framework — Reproducible Simulation

This repository contains a MATLAB simulation framework for a differential-drive mobile manipulator (2 wheels + 1 caster, 3-DOF arm) using a three-layer architecture:
- Layer 3: Offline reference generation (planner)
- Layer 2: Kinematic coordination (Kanayama base + arm velocity PD)
- Layer 1: Dynamic tracking (reduced-coordinate feedback linearization)

This README explains how to reproduce the simulation results using the provided live scripts.

---

## 1) Requirements

### Software
- MATLAB (recommended R2022b or newer)
- Toolboxes typically used:
  - Control System Toolbox (recommended)
  - Symbolic Math Toolbox (only if you regenerate symbolic dynamics; not required for running the provided scripts)
  - (Optional) Robotics System Toolbox (not required for core runs)

### Hardware
- Any modern laptop is sufficient (simulation is lightweight).

---

## 2) Folder Structure (relevant parts)

- `.\wheeled_manipulator_framework\models\`
  - `9_mm_main_fb_lin_tracking.mlx`
- `.\wheeled_manipulator_framework\control\`
  - `main_controller.mlx`
---

## 3) Quick Start (Recommended Workflow)

### Step A — Run the main controller script (full stack results)
1. Open MATLAB.
2. Set the MATLAB Current Folder to the repository root, or add the repo to path.
3. Open and run:

   `.\wheeled_manipulator_framework\control\main_controller.mlx`

This script runs the complete pipeline (planner → kinematics → dynamics) and generates the figures used in the paper.

✅ Expected outputs:
- Plots for base trajectory, η/η̇/η̈ tracking, torques (wheels + arm), power, accumulated energy, and tracking errors.

---

### Step B — Run the model-side tracking script (Layer-1 focused / sanity checks)
Open and run:

`.\wheeled_manipulator_framework\models\9_mm_main_fb_lin_tracking.mlx`

Use this script to:
- Validate the reduced-dynamics + feedback linearization layer in isolation
- Compare behaviors with/without planner references depending on the options inside the live script

---

## 4) Reproducing Paper Experiments (Typical)

The project commonly reports experiments like:

### E1 — Layer 1 only (dynamic tracking focus)
- Goal: show that Layer 1 can track a provided η-reference but does not generate task-level motion alone.
- Output figure example:
  - `E1_layer1_only_base_path_world.png`

### E2 — Planner-only (open-loop reference quality)
- Goal: show smooth planned references (η, η̇, η̈) and base path without feedback execution.
- Output figure examples:
  - `E2_planner_only_base_path_world.png`
  - `E2_planner_only_eta.png`
  - `E2_planner_only_etadot.png`
  - `E2_planner_only_etaddot.png`

### E3 — Disturbance without full feedback compensation
- Goal: show drift under torque disturbances when feedback coordination is not fully active.
- Output figure examples:
  - `E3_dist_base_path_world.png`
  - `E3_dist_eta.png`

### E4 — Full stack under disturbance (planner + kinematics + dynamics)
- Goal: show convergence and robustness with complete architecture.
- Output figure examples:
  - `E4_dist_base_path_world.png`
  - `E4_nominal_base_path_world.png` (nominal after disturbance case, if enabled)
  - `E4_dist_eta.png`
  - `E4_dist_etadot.png`
  - `E4_dist_etaddot.png`
  - `E4_dist_full_tracking_error_eta.png`
  - `E4_dist_base_errors.png`
  - `E4_dist_torque_wheels.png`
  - `E4_dist_torque_arm.png`
  - `E4_dist_power.png`  (watts)
  - `E4_dist_energy.png` (joules)


---

## 5) Before You Run (Important)

1. Ensure the repository root and all subfolders are on the MATLAB path.
   - If needed:
     - Home → Set Path → Add with Subfolders → select `wheeled_manipulator_framework`

2. Ensure the figures folder exists:
   - `.\wheeled_manipulator_framework\figures\`

3. If the scripts expect precomputed `.mat` dynamics/model files:
   - Keep them in their expected locations (do not rename/move).
   - If you removed them, you must regenerate them first (only if the scripts require it).

---

## 7) How to Cite / Credit
If you use or modify this framework, please cite the accompanying project report/paper (IEEE format) and credit KFUPM / University of Ottawa affiliations as appropriate.

---

## 8) Contact
- Muhammed Elyamani: g202427600@kfupm.edu.sa, melya038@uottawa.ca
- Dr. Ali Attyatullah Mohammed AlBeladi: albeladi@kfupm.edu.sa
- Dr. Brahim Brahmi: brahim.brahmi@kfupm.edu.sa
