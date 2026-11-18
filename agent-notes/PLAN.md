# Project 4 Full Implementation Plan: Step and Solve Services

## Goal

Complete the implementation of Project 4 by adding `/ik/solve_robot_step` and `/ik/solve_robot` to the runtime, satisfying all requirements for Categories C (Gradient step), D (IK solve, position), E (IK solve, orientation), and the near-1 MiB description test from Category B, building on top of the validated `/ik/robot_jacobian` checkpoint.

## Relevant requirements

1. **Protocol & Gateway (`127.0.0.1:9095`)**:
   - Adhere to the newline-delimited TCP JSON ROS-like protocol.
   - Service responses match:
     `{"op": "service_response", "service": <svc>, "values": {...}, "result": <bool>, "status": <string>, "id": <req_id>}`.
   - Line and payload buffers must seamlessly handle >= 1 MiB.
   - Must answer within 10 seconds of `make run`.

2. **Parameter Server Integration & Version Tracking**:
   - Query `robot_description` from `/param_server/get_param`.
   - `description_version` included in every reply (success or error). Version 0 if no robot loaded.
   - Within 2 seconds of `set_param` returning version $V$, replies reflect version $\ge V$.
   - Rejection on invalid URDF without crashing, recovering when valid description arrives.

3. **Service: `/ik/solve_robot_step`**:
   - Request fields:
     * `joint_state`: current configuration $q$
     * `endeffector_frame`: string name of chain link
     * `endeffector_pose`: `position` {x,y,z} and `orientation` {x,y,z,w}
     * `ik_position_only`: bool
     * `method`: `"JacobianTranspose"` or `"JacobianPseudoinverse"`
     * `step_length`: $\alpha > 0$
     * `damping`: $\lambda \ge 0$
   - Course defaults if client leaves optional fields out:
     `method: "JacobianTranspose"`, `step_length: 0.1`, `ik_position_only: true`, `damping: 0.0`.
   - Step formula:
     * Position error $e = p_{\text{target}} - p_e(q)$.
     * Full pose error: $e_{6D} = [e ; e_\omega]$, with $e_\omega$ the rotation vector (axis $\times$ angle) of $R_{\text{target}} R_e(q)^T$.
     * `"JacobianTranspose"`: $\Delta q = \alpha J^T e$.
     * `"JacobianPseudoinverse"`: $\Delta q = \alpha J^T (J J^T + \lambda^2 I)^{-1} e$.
   - Stateless: does not remember or continue past calls.
   - Reply:
     `{"values": {"configuration_solution": {"name": [...], "position": [...]}, "description_version": V}, "result": true, "status": ""}`.
   - Precision: $q + \Delta q$ within $10^{-4}$ of exact value.

4. **Service: `/ik/solve_robot`**:
   - Request fields: all fields from step plus:
     * `position_tolerance`: meters $> 0$
     * `orientation_tolerance`: radians $> 0$ (used when `ik_position_only: false`)
     * `max_iterations`: integer $\ge 1$
   - Iteration algorithm:
     1. Evaluate FK at current $q$.
     2. Compute position error $\|p_{\text{target}} - p_e(q)\|$.
        For full pose: compute geodesic orientation error $\theta = \text{angle}(R_{\text{target}}^T R_e(q)) \in [0, \pi]$.
     3. If errors are within tolerance: stop with `converged: true`.
     4. If iteration count reached `max_iterations`: stop with `converged: false`.
     5. Take one gradient step with requested method, step length, damping.
     6. Clamp revolute and prismatic joints to their URDF limits $[lower, upper]$. Continuous joints have no limits.
     7. Repeat from 1.
   - Reply:
     * `configuration_solution`: final configuration.
     * `converged`: boolean.
     * `final_error`: `{"position": <m>, "orientation": <rad> | null}` of the returned configuration.
     * `iterations`: number of steps taken.
     * `description_version`: V.
     * `result`: MUST equal `converged`.
     * `status`: `"converged"` if converged, non-empty error status if not converged (e.g. `"did not converge"`).
   - Speed: Solves must complete within a fraction of a second (< 5s for 2000 iterations).

## Repository observations

- `IKService` (`src/ik_service.cpp`, `include/ik_service.hpp`) already has:
  * Thread-safe model caching and version tracking.
  * Chain extraction (root to end-effector link, excluding fixed joints and branches).
  * Joint state input validation and default zeroing.
  * Forward kinematics calculation via `robot::compute_fk`.
  * Joint frame origin $p_i$, rotation $R_i$, and Jacobian row computation for revolute, continuous, and prismatic joints.
- `math3d` (`src/math3d.cpp`, `include/math3d.hpp`) contains 3D vector and matrix utilities.
- `RosbridgeServer` supports `__status__` extraction to allow service handlers to customize top-level `status` for both success and non-converged/error responses.

## Proposed architecture

1. **`math3d` Extensions**:
   - Add $3 \times 3$ matrix multiplication and transpose helpers.
   - Add function to compute quaternion rotation matrix $R_{\text{target}}$.
   - Add function to compute orientation difference angle $\theta \in [0, \pi]$ and rotation vector $e_\omega$ according to spec formulas:
     $\theta = \text{atan2}(\|v\|/2, (\text{trace}(R) - 1)/2)$ where $v = [R_{32} - R_{23}, R_{13} - R_{31}, R_{21} - R_{12}]$.
     $u = v / \|v\|$ for $0 < \theta < \pi$, $e_\omega = \theta u$. Handle $\theta = 0$ and $\theta = \pi$ boundary conditions cleanly.
   - Add small linear system solver `solve_linear_system<N>(A, b, x)` using Gaussian elimination with partial row pivoting ($N=3$ and $N=6$).

2. **`IKService` Refactoring & Additions**:
   - Separate chain and configuration extraction into reusable internal helpers:
     * `extract_chain(ee_link_idx, chain_movable_joints, joint_names, chain_movable_names)`
     * `parse_and_validate_joint_state(js, chain_movable_names, config_out, error_msg)`
     * `compute_jacobian_internal(fk, chain_movable_joints, p_e, J_v, J_omega)`
   - Implement `compute_step_delta(...)`:
     * Handles position-only and full-pose errors.
     * Supports `"JacobianTranspose"` and `"JacobianPseudoinverse"` (damped least squares $J J^T + \lambda^2 I$).
   - Implement `handle_solve_robot_step(args)`:
     * Parses request, validates inputs and fills defaults.
     * Evaluates single step, constructs response.
   - Implement `handle_solve_robot(args)`:
     * Parses request, validates tolerances and max iterations.
     * Executes iterative gradient loop with limit clamping and convergence checking.
     * Evaluates final FK and errors on configuration returned.
     * Returns `result: converged`, `converged`, `final_error`, `configuration_solution`.

3. **`main.cpp` & `Makefile`**:
   - `ik::IKService` already created in `main.cpp`. It registers the two new services `/ik/solve_robot_step` and `/ik/solve_robot`.

## Implementation steps

1. Add rotation difference, rotation vector, and linear system solver to `include/math3d.hpp` and `src/math3d.cpp`.
2. Add unit tests for math routines in `tests-local/math3d_test.cpp` and verify.
3. Update `include/ik_service.hpp` and `src/ik_service.cpp` to implement `/ik/solve_robot_step` and `/ik/solve_robot`.
4. Compile with `make build` and fix any compiler warnings.
5. Create comprehensive verification tests in `tests-local/test_ik_verification.py` checking:
   - 2-link arm single steps (transpose and pseudoinverse, damped and undamped).
   - Numerical gradient vs analytical Jacobian step.
   - Multi-link full-pose 6D step.
   - Position-only solve to convergence.
   - Unreachable target reporting (`converged: false`).
   - Joint limits enforcement during solve.
   - Full-pose 6-DOF / 7-DOF arm solve.
   - Version tracking and dynamic reload during solves.
6. Verify 1 MiB line handling with large URDF descriptions.
7. Update `tests/` pass/fail suite (ensuring exactly $\le 25$ tests) to include Project 4 step and solve tests.
8. Run full `make test` and verify 25 of 25 pass cleanly.
9. Record implementation notes and handoff in `agent-notes/IMPLEMENTATION.md`.
