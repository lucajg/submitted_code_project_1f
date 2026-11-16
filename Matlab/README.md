# MATLAB setup and checks

[Project overview and topic documentation](../README.md)

The code is grouped by responsibility. Each topic folder contains its implementation, relevant checks, and a README. Shared geometry is in `models/`; `shared/` contains transform and Jacobian calculations used across topics. Topic-specific helpers live in `private/` subfolders.

The bundled `data/CuRo6R.urdf` supplies the source geometry for the FK checks. The numerical code and project viewers do not require files outside this folder or the URDF's mesh assets.

## Entry points

| File | Purpose |
| --- | --- |
| [curo6r_setup.m](curo6r_setup.m) | Add the explicit list of project folders to the current MATLAB session. |
| [run_checks.m](run_checks.m) | Configure the path and run FK, Jacobian, IK, planning, and optional UI/symbolic checks. |

## Setup

From the repository root:

```matlab
addpath('Matlab');
curo6r_setup;
```

If you open the submitted `Matlab` folder itself as the current folder, call `curo6r_setup` directly. Use this setup function instead of `addpath(genpath(...))`: `ik/history/` contains archived solvers with the same function names as the active implementation and is deliberately excluded.

To reproduce the report's Section 5.3 experiment after setup:

```matlab
q_start = [0.1024; -1.1124; 1.8547; -1.4399; -0.3835; -2.6486];
[path, goal, candidates] = path_between_to_ik_solutions(q_start, n_samples=10000);
```

This displays joint trajectories, the tool path, the arm singularity map with IK branches, and joint-limit and scaled-Jacobian margins. Expected minimum margins are approximately `0.492993` rad and `0.0131581`, respectively. Report-quality figure generators are in the repository's `report/figures/` directory; they are separate from these MATLAB calculations and plots.

## Run checks

After adding `Matlab` to the path, the runner performs setup automatically:

```matlab
run_checks;               % Numerical checks and the invisible singularity app.
% run_checks(true);       % Also include symbolic Jacobian checks.
% run_checks(false,false); % Numerical checks only, without graphics.
```

`run_checks(include_symbolic, include_ui)` accepts scalar logical flags, defaulting to `false` and `true`. Failures propagate immediately; skipped groups are reported explicitly. Symbolic checks require Symbolic Math Toolbox and can take substantially longer. UI checks require MATLAB graphics even though their app window is invisible. Numerical checks require no additional toolbox.

Individual checks remain available after setup:

| Check | Coverage |
| --- | --- |
| [check_dh_poe](fk/README.md#checks) | DH, PoE, and URDF poses and joint axes. |
| [check_jacobians](jacobian/README.md#checks) | Numerical comparisons, finite differences, and optionally symbolic equality. |
| [check_analytical_ik](ik/README.md#checks) | Arm and wrist branches, round trips, singularities, and invalid targets. |
| [check_path_planning](planning/README.md#checks) | Start validation, regular paths, structured failure reports, and console output. |
| [check_singularity_map](singularity/README.md#checks) | Branch tracking, workspace curves, and UI callbacks. |

`check_jacobians` itself retains its existing default of including symbolic checks; call `check_jacobians(false)` for numerical checks alone. The [joint-axis viewer](visualization/README.md#checks) has a separate manual UI check.
