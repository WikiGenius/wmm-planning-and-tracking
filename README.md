# WMM Trajectory Tracking

## Overview
This repository contains MATLAB experiments for trajectory tracking in a whole-body / mobile-manipulator setting. It is kept public as portfolio evidence for robotics control, trajectory tracking, and structure-aware planning foundations.

The public version is intentionally lightweight: it communicates the engineering direction without exposing unpublished research algorithms, datasets, or experiment logs.

## Research/Engineering Motivation
Structure-aware scanning with a mobile manipulator requires more than planning a geometric path. The robot must also track scan-relevant trajectories while respecting motion constraints, base-arm coordination, and sensing quality requirements.

This repository supports that direction by focusing on trajectory tracking and control behavior for mobile manipulation workflows.

## Features
- MATLAB-based trajectory tracking experiments.
- Mobile robot / mobile-manipulator control structure.
- Public scaffold for documenting control assumptions and results.
- Space for future tracking metrics, plots, and experiment notes.

## Method
At a high level, the workflow is:

1. Define a reference trajectory or scan-relevant path.
2. Model the robot motion variables and constraints.
3. Design or test a tracking controller.
4. Simulate tracking behavior.
5. Evaluate tracking error, control effort, and qualitative motion quality.

Detailed unpublished controller variants and experiment logs should remain private until paper-ready.

## Installation
Clone the repository:

```bash
git clone https://github.com/WikiGenius/wmm-trajectory-tracking.git
cd wmm-trajectory-tracking
```

Open the MATLAB project/scripts from the repository root.

## Run
Run the relevant MATLAB scripts from the `mobile_robot/` folder after checking any local data or path assumptions.

```matlab
cd mobile_robot
% Run the experiment script for the selected tracking setup.
```

## Results
Add public-safe figures and summary metrics here as the experiments mature:

- reference vs tracked trajectory,
- tracking error over time,
- control input / effort,
- qualitative notes on constraints and failures.

## Limitations
- This is a public portfolio artifact, not the full private research workspace.
- Some unpublished controller variants, data, and logs are intentionally omitted.
- Scripts may need path cleanup before fully reproducible release.

## Roadmap
- [ ] Add a minimal reproducible example.
- [ ] Add tracking-error plots.
- [ ] Add controller baseline comparison.
- [ ] Add experiment log template.
- [ ] Link to report/paper when public.

## Citation / Acknowledgment
Acknowledge any course notes, papers, MATLAB examples, or robotics libraries used when results are added.
