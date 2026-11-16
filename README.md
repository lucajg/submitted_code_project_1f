# CuRo6R — Project 1F submission

MATLAB code accompanying the Project 1F report on kinematics, analytical inverse kinematics, singularities, and nonsingular changes of solution for CuRo6R.

The submission contains DH, Product of Exponentials (PoE), and URDF forward models; a PoE geometric Jacobian; analytical arm and wrist IK; and a planner that connects distinct IK solutions while maintaining tool orientation. The URDF is bundled in [Matlab/models/CuRo6R.urdf](Matlab/models/CuRo6R.urdf). The included calculations and plots do not need mesh files.

## Quick start

Open this repository as MATLAB's current folder, then run:

```matlab
addpath('Matlab');
curo6r_setup;
run_checks; % Numerical checks, including the invisible singularity explorer.
```

To reproduce the report's Section 5.3 experiment and plot its trajectory and margins:

```matlab
q_start = [0.1024; -1.1124; 1.8547; -1.4399; -0.3835; -2.6486];
[path, goal, candidates] = path_between_to_ik_solutions(q_start, n_samples=10000);
```

The minimum joint-limit clearance should be approximately **0.492993 rad**, and the minimum scaled Jacobian singular value approximately **0.0131581**. To explore other wrist positions and tool orientations interactively, run `singularity_map` after setup.

## Requirements and documentation

Numerical calculations require MATLAB with no additional toolbox. Symbolic calculations require Symbolic Math Toolbox. Interactive plots and UI checks require MATLAB graphics. The code is tested with MATLAB R2024b. See [setup and checks](Matlab/README.md) for running without graphics or including symbolic checks.

| Topic | Contents |
| --- | --- |
| [Models](Matlab/models/README.md) | Robot dimensions, bundled URDF, DH parameters, and home screw axes. |
| [Forward kinematics](Matlab/fk/README.md) | DH, PoE, and URDF transforms and their independent comparisons. |
| [Jacobians](Matlab/jacobian/README.md) | Numerical and symbolic PoE Jacobians, checked against differentiated DH FK. |
| [Inverse kinematics](Matlab/ik/README.md) | The report's quartic in `tan(q3/2)`, wrist alternatives, and full-pose solutions. |
| [Singularities](Matlab/singularity/README.md) | Arm maps, wrist-flip labels, branch tracking, and the interactive explorer. |
| [Planning](Matlab/planning/README.md) | Straight arm paths, continuous wrist recovery, margins, and rejection diagnostics. |
| [Shared helpers](Matlab/shared/README.md) | Transform and cross-product calculations. |

## Conventions

The base is `base_link`, the tool endpoint is `link_6`, and all models share the URDF joint order, positive directions, and home configuration. Distances are in metres; joint angles are in radians, except UI controls explicitly marked in degrees. Joint limits are `[-pi, pi]` for all six joints.

`T = [R, p; 0, 0, 0, 1]` expresses the tool pose in the base frame. IK returns joint configurations as columns. Geometric Jacobians place linear velocity above angular velocity; their `joint` argument selects the reference point and `frame` selects physical link axes for expressing base-relative velocities. Point 5 is the wrist centre and point 6 is the tool endpoint.

The planner holds tool orientation fixed while the tool position follows a closed path between distinct IK configurations. Its path parameter is not time, and its checks validate sampled configurations rather than certify the continuous intervals.
