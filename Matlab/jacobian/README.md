# Geometric Jacobians

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Compute the CuRo6R geometric Jacobian from the PoE model, numerically or symbolically. Symbolic results can also be expressed using the report's DH length parameters.

## Files

| File | Purpose |
| --- | --- |
| [poe_jacobian.m](poe_jacobian.m) | Full tool velocity evaluated at a physical joint origin. |
| [poe_jacobian_symbolic.m](poe_jacobian_symbolic.m) | Entry script for symbolic PoE results. |
| [dh_parameter_form.m](dh_parameter_form.m) | Rewrite dimension symbols using nonzero DH parameters. |
| [check_jacobians.m](check_jacobians.m) | Check PoE Jacobians against differentiated DH FK and symbolic identities. |
| [point_jacobian.m](../shared/point_jacobian.m) | Shared cross-product calculation from joint origins and axes. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
poe_model = poe_data();
q = [20; -35; 50; 15; -40; 60]*pi/180;
J06 = poe_jacobian(q, 0, 6, poe_model); % Tool endpoint, base axes.
J05 = poe_jacobian(q, 0, 5, poe_model); % Wrist centre, base axes.
J35 = poe_jacobian(q, 3, 5, poe_model); % Wrist centre, physical link-3 axes.
J_position = J35(1:3,1:3);
check_jacobians(false);
```

## Dependencies

[Robot models](../models/README.md), [forward kinematics](../fk/README.md), and [shared kinematics helpers](../shared/README.md). Numerical evaluation needs no additional toolbox. Symbolic models, `poe_jacobian_symbolic`, `dh_parameter_form`, and the optional symbolic checks require Symbolic Math Toolbox.

## Inputs, outputs, and implementation

`J = poe_jacobian(q, frame, joint, poe_model)` accepts a six-element row or column joint vector and returns a `6 x 6` matrix with linear velocity above tool angular velocity. All six joints contribute. `joint` selects the physical joint origin at which the tool's velocity field is evaluated: point 5 is the wrist centre and point 6 is the endpoint. `frame` selects the physical link axes for expressing both velocity vectors, from 0 (`base_link`) through 6 (`link_6`); frame 3 is `link_3_d3`.

Both velocity vectors remain relative to the base. Selecting moving link axes rotates their components; it does not subtract that link's own motion. The angular rows describe the complete tool motion even when the selected reference point is an earlier joint origin.

The function uses the joint origins and axes returned by PoE FK to form each revolute column as `[cross(w_i, p_joint-p_i); w_i]`, then rotates both blocks into the selected frame. Numerical and symbolic inputs are supported.

At the wrist centre, `J(1:3,1:3)` is the arm's linear block `J_v_arm`, `J(4:6,1:3)` its angular block `J_omega_arm`, and `J(4:6,4:6)` the wrist's angular block `J_omega_wrist`. The wrist's linear columns vanish because its three axes meet there. Section 3 of the accompanying report derives the factorization `det(J) = -sin(q5)*det(J_v_arm)`.

To derive the exact wrist-centre Jacobian in physical link-3 axes:

```matlab
gs = robot_dimensions(true);
qs = sym('q',[6,1],'real');
poe = poe_data(gs);
Jpoe_sym = simplify(poe_jacobian(qs,3,5,poe));
Jpoe_ad_sym = dh_parameter_form(Jpoe_sym);
```

[poe_jacobian_symbolic.m](poe_jacobian_symbolic.m) provides the same calculation as a script, producing `Jpoe_sym` and `Jpoe_ad_sym`. Edit `k` and `f` in that script to select the reference point and expression frame.

`dh_parameter_form` changes the notation of a symbolic expression; it does not compute a second Jacobian. It substitutes `a1 = l2`, `a2 = l3`, `d1 = h1+h2`, `d2 = w3`, `d3 = h4`, `d4 = l4+l5`, and `d6 = l6`. The remaining split lengths `h1` and `l4` cancel at points 5 and 6; other physical origins may still need them. The parameter `d1` also drops out of these Jacobians because it only translates the robot vertically.

## Checks

`check_jacobians(false)` compares the numerical PoE Jacobian against central finite differences of the independent DH forward model at 114 configurations, for points 5 and 6 and expression frames 0 through 6, using default and modified dimensions. Physical expression-frame orientations are recovered from the DH transforms and their constant home alignments. The check also covers row-vector inputs, the wrist's zero linear columns and retained sixth angular column, and closed-form PoE motions versus matrix exponentials.

`check_jacobians(true)` additionally derives symbolic PoE Jacobians for `(frame, point) = (0,6), (0,5), (3,5)`. It verifies the exact physical link-3 wrist block, the wrist determinant `-sin(q5)`, and the report's closed-form arm determinant. Numerical evaluations of both the original dimension symbols and the rewritten DH parameter forms are checked against numerical PoE Jacobians for both geometries.

Calling `check_jacobians` directly includes symbolic checks by default. The full `run_checks` runner includes them only when its first argument is `true`.
