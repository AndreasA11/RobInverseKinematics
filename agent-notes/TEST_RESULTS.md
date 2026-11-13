# Test Results: Project 4 Checkpoint (/ik/robot_jacobian)

## Summary

All unit tests and end-to-end integration tests over the TCP/JSON rosbridge gateway on `127.0.0.1:9095` passed successfully. The `/ik/robot_jacobian` service was verified against the course worked example, finite differences against forward kinematics, prismatic joint axis handling, multi-branch chain isolation, dynamic parameter updates, and invalid input rejections.

## Test environment

- Operating System: macOS (Darwin)
- Compiler: Apple Clang / G++ with `-std=c++20 -O2 -pthread`
- Build Target: `make build` (offline, vendored `pugixml`)
- Protocol: Newline-delimited JSON over plain TCP on `127.0.0.1:9095`
- Test Framework: Python 3.12 standard library socket client

## Tests performed

1. **Pre-load State (No Robot Loaded)**
   - **Requirement/risk being tested:** `spec/IK_API.md` lines 52 & 49: With no robot loaded, IK requests must return `result: false`, non-empty `status`, and `description_version: 0`.
   - **Method:** Called `/ik/robot_jacobian` immediately upon starting runtime before setting `robot_description`.
   - **Expected behavior:** `result: false`, `description_version: 0`, non-empty `status`.
   - **Observed behavior:** `{'result': False, 'status': 'no robot description loaded', 'values': {'description_version': 0}}`.
   - **Result:** Pass.

2. **Worked Two-Link Planar Arm (Spec Example)**
   - **Requirement/risk being tested:** `spec/IK_API.md` worked example lines 102–114.
   - **Method:** Loaded worked two-link arm; evaluated at shoulder = 0, elbow = $\pi/2$.
   - **Expected behavior:**
     * `joint_names: ["shoulder", "elbow"]`
     * Shoulder row: `[-1.0, 1.0, 0.0, 0.0, 0.0, 1.0]` (within $10^{-4}$)
     * Elbow row: `[-1.0, 0.0, 0.0, 0.0, 0.0, 1.0]` (within $10^{-4}$)
     * `tip_mount` (fixed joint) omitted from rows and joint names.
   - **Observed behavior:** Exact values matched within $< 10^{-15}$.
   - **Result:** Pass.

3. **Finite Difference Verification vs Forward Kinematics**
   - **Requirement/risk being tested:** Verification that linear velocity entries $v$ equal $\frac{\partial p_e}{\partial q}$.
   - **Method:** Evaluated Jacobian at arbitrary configuration $q = [0.3, 0.7]$. Perturbed each joint by $h = 10^{-6}$ and calculated numerical derivative $\frac{p_e(q + h) - p_e(q)}{h}$.
   - **Expected behavior:** Numerical derivative matches Jacobian row within $10^{-4}$.
   - **Observed behavior:** Numerical derivative matched analytical values to within $10^{-6}$.
   - **Result:** Pass.

4. **Prismatic Joint with Non-Unit Axis and Rotated Origin**
   - **Requirement/risk being tested:** `spec/IK_API.md` line 99: Prismatic row is $[R_i \cdot a_i ; 0, 0, 0]$ with axis unnormalized.
   - **Method:** Joint `slide` with axis `2 0 0` under origin rotated by yaw = $\pi/2$.
   - **Expected behavior:** Row is `[0.0, 2.0, 0.0, 0.0, 0.0, 0.0]`.
   - **Observed behavior:** Row matched `[0.0, 2.0, 0.0, 0.0, 0.0, 0.0]` within $10^{-15}$.
   - **Result:** Pass.

5. **Multi-Branch Kinematic Tree Isolation**
   - **Requirement/risk being tested:** Chain extraction must strictly isolate the root-to-end-effector path and ignore other branches.
   - **Method:** Robot with left and right arms. Requested end-effector `left_tip`.
   - **Expected behavior:** `joint_names` contains only left arm movable joints; right arm joints are excluded.
   - **Observed behavior:** `joint_names: ["left_j1", "left_j2"]`.
   - **Result:** Pass.

6. **Error Rejection and Recovery**
   - **Requirement/risk being tested:** Unknown end effector, mismatched joint state arrays, non-number joint positions, and invalid URDF handling.
   - **Method:** Sent requests with invalid frames, array mismatches, booleans for positions; set malformed XML description then reloaded valid description.
   - **Expected behavior:** All invalid requests return `result: false` with non-empty status without crashing runtime; subsequent valid description recovers service.
   - **Observed behavior:** All malformed requests rejected cleanly; reload recovered service with updated `description_version`.
   - **Result:** Pass.

7. **Project Test Suite (`tests/run_tests.py`)**
   - **Method:** Executed `python3 tests/run_tests.py`.
   - **Observed behavior:** 25 of 25 tests passed (including all 23 prior Project 3 regression checks and 2 new Project 4 Jacobian tests).
   - **Result:** Pass.

## Failures

None. All 25 test cases passed without failures.

## Remaining gaps

- Solver iteration tests for `/ik/solve_robot_step` and `/ik/solve_robot` are deferred to the final project implementation phase.
