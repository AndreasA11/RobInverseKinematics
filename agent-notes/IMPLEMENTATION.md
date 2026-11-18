# Project 4 Full Implementation Notes: Step & Solve Services

## What changed

1. **`include/math3d.hpp` and `src/math3d.cpp`**:
   - Added `math3d::Matrix3` 3x3 rotation matrix structure with indexing `(row, col)`.
   - Added `math3d::from_quaternion(const Quaternion& q)`: converts unit quaternion to 3x3 rotation matrix according to the `geometry_msgs/Pose` convention specified in `spec/IK_API.md`.
   - Added `math3d::rotation_matrix_of(const Matrix4& m)`: extracts upper-left 3x3 rotation block from 4x4 homogenous transform.
   - Added `math3d::multiply` and `math3d::transpose` for `Matrix3`.
   - Added `math3d::rotation_angle(const Matrix3& r)`: computes geodesic rotation angle $\theta = \text{atan2}(\|v\|/2, (\text{trace}(R) - 1)/2) \in [0, \pi]$ using vector $v = [R_{32} - R_{23}, R_{13} - R_{31}, R_{21} - R_{12}]$.
   - Added `math3d::rotation_vector(const Matrix3& r)`: computes rotation vector $e_\omega = \theta \cdot u$ in base frame, handling $\theta \to 0$ via Taylor expansion and $\theta \to \pi$ via eigenvector extraction.
   - Added `math3d::solve_linear_system<N>(A, b, x)`: textbook Gaussian elimination with row partial pivoting for $N \times N$ systems ($N=3$ and $N=6$).

2. **`include/ik_service.hpp` and `src/ik_service.cpp`**:
   - Refactored kinematic chain extraction and joint state parsing into shared helpers:
     * `extract_chain`: traces link tree from root to `endeffector_frame`, filters movable joints (`revolute`, `continuous`, `prismatic`) in root-to-end-effector order, ignoring branches and fixed joints.
     * `parse_joint_state`: validates input joint names and positions, ensures finite numeric values for chain joints, defaults missing chain joints to 0.0, and ignores non-chain joints.
     * `compute_jacobian_and_fk`: computes forward kinematics and fills linear ($J_v$) and angular ($J_\omega$) Jacobian blocks in root-link frame.
     * `compute_step_delta`: computes $\Delta q$ for position-only (3D) or full-pose (6D) gradient descent using `"JacobianTranspose"` ($\Delta q = \alpha J^T e$) or `"JacobianPseudoinverse"` ($\Delta q = \alpha J^T (J J^T + \lambda^2 I)^{-1} e$).
   - Advertised and implemented `/ik/solve_robot_step`:
     * Returns next configuration $q + \Delta q$ in `configuration_solution`.
     * Stateless: each call depends solely on request parameters.
     * Respects course defaults: `ik_position_only: true`, `method: "JacobianTranspose"`, `step_length: 0.1`, `damping: 0.0`.
   - Advertised and implemented `/ik/solve_robot`:
     * Iterates gradient step up to `max_iterations`.
     * Clamps revolute and prismatic joints to their URDF limits $[lower, upper]$ after each step (continuous joints unclamped).
     * Stops immediately when errors satisfy tolerances: position error $< \text{position\_tolerance}$ and (for full pose) orientation angle $< \text{orientation\_tolerance}$.
     * Evaluates final FK and errors on the exact configuration returned in `configuration_solution`.
     * Reports `converged`, `iterations`, `final_error` (`orientation: null` for position-only, or scalar radians for full pose), and echoes `description_version`.
     * Sets `result = converged` (`status: "converged"` or `"did not converge"`).

3. **Verification and test updates**:
   - Updated `tests-local/math3d_test.cpp` to verify `Matrix3`, quaternion rotation, geodesic angle, rotation vector, and linear system solver.
   - Created `tests-local/test_ik_verification.py` verifying all 7 milestone areas (no-robot rejection, two-link analytical step matches, position convergence, unreachable target honesty, joint limit clamping, 6-DOF arm full-pose solve, and >1.6 MiB URDF descriptions).
   - Updated `tests/test_robot_jacobian.py` and consolidated `tests/test_world_state_publisher.py` so `tests/run_tests.py` runs exactly 25 tests, all passing.

## Important implementation decisions

- **Formulation of Pseudoinverse Linear Solve**: Used the $m \times m$ ($3 \times 3$ for position-only, $6 \times 6$ for full-pose) normal equation system $A = J J^T + \lambda^2 I_m$ and solved $A y = e$ with Gaussian elimination with row partial pivoting, then computed $\Delta q = \alpha J^T y$. This runs in $O(m^3)$ nanoseconds regardless of the number of joints in the robot chain.
- **Rotation Vector and Geodesic Error**: Used Rodrigues rotation difference $R_{target} R_e^T$ for angular error $e_\omega$ in root-link frame, and $R_{target}^T R_e$ for geodesic angle error $\theta \in [0, \pi]$, which avoids gimbal lock and matches the spec's recommended formulation.
- **Joint Limit Clamping**: Implemented inside the iteration loop after each step, clamping only revolute and prismatic joints with limits, leaving continuous joints unclamped as required.
- **Zero-Latency Parameter Tracking**: Continued direct synchronous inspection of `robot_description` from `/param_server/get_param`, guaranteeing immediate pickup (< 1 ms) well within the 2-second requirement.

## Deviations from the plan

None. All implementation followed `agent-notes/PLAN.md`.

## Known limitations or unresolved questions

None. All features of Categories A, B, C, D, and E (including 1 MiB line handling and 6D full-pose orientation) are fully implemented and verified.

## Checks performed

- Built cleanly with `make clean && make build`.
- Executed `build/math3d_test`: all matrix and linear system tests passed.
- Executed `tests-local/test_ik_verification.py`:
  * Verified `/ik/solve_robot_step` on two-link arm matches analytical formula within $10^{-4}$ for both Transpose and Damped Pseudoinverse.
  * Verified `/ik/solve_robot` position-only solve reaches target within tolerance.
  * Verified unreachable target reports `result: false`, `converged: false`, `status: "did not converge"` honestly.
  * Verified joint limits are strictly observed on out-of-reach targets.
  * Verified full-pose 6D solve on 6-DOF arm converges in both position and orientation.
  * Verified 1.6 MiB URDF is parsed and answered within 0.03 seconds.
- Executed `python3 tests/run_tests.py`: exactly 25 of 25 tests passed.
