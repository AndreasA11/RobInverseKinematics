# Project 4 Checkpoint: Robot Jacobian Implementation Notes

## What changed

1. **`include/math3d.hpp` and `src/math3d.cpp`**:
   - Added `math3d::cross(const Vector3& a, const Vector3& b)`: standard 3D right-handed cross product for linear velocity computation.
   - Added `math3d::rotate_vector(const Matrix4& transform, const Vector3& v)`: rotates a 3D vector using the upper-left 3x3 rotation block of a 4x4 transform matrix.

2. **`include/middleware/rosbridge_server.hpp`**:
   - Added `__status__` extraction in internal service response builder: allows internal service handlers to set custom non-empty status strings directly while keeping `values` clean.
   - Raised external service call timeout to 30 seconds to support longer solves as recommended by `spec/OVERVIEW.md`.

3. **`include/ik_service.hpp` and `src/ik_service.cpp`**:
   - Created `ik::IKService` class registering `/ik/robot_jacobian` on `Middleware`.
   - Thread-safe caching and parsing of `robot_description` fetched directly from `/param_server/get_param`:
     * Returns `result: false`, `status: "no robot description loaded"`, `description_version: 0` before any description is set.
     * Caches parsed models by description version to ensure zero-latency response.
     * Handles invalid URDFs by rejecting subsequent calls with `result: false` and the invalid version, recovering automatically when a valid description arrives.
   - Chain extraction and Jacobian calculation:
     * Traces the kinematic tree from `endeffector_frame` to root.
     * Extracts chain movable joints (`revolute`, `continuous`, `prismatic`) in root-to-end-effector order, ignoring branches and fixed joints.
     * Evaluates forward kinematics at requested configuration.
     * Computes linear and angular velocity rows:
       - Revolute/Continuous: $v = \hat{z} \times (p_e - p_i)$ with normalized joint axis rotated to root frame, $\omega = \hat{z}$.
       - Prismatic: $v = R \cdot a$ with axis used as written (unnormalized), $\omega = [0, 0, 0]$.
   - Validation & error handling:
     * Rejects unknown `endeffector_frame`.
     * Rejects `joint_state` name/position length mismatch.
     * Rejects non-number and boolean positions for movable joints on the chain.
     * Tolerates non-chain joints and defaults missing chain joints to 0.0.

4. **`src/main.cpp` and `Makefile`**:
   - Linked and instantiated `ik::IKService` in `main.cpp` alongside `ParameterServer` and `DescriptionLoader`.
   - Updated `Makefile` `main` target with `src/ik_service.cpp` and `include/ik_service.hpp`.

5. **`tests/test_robot_jacobian.py`**:
   - Added pass/fail tests for worked two-link arm and prismatic/branch-isolation testing under `tests/run_tests.py` (total tests = 25/25).

## Important implementation decisions

- **Zero-latency description polling**: Rather than a periodic background polling thread that could suffer from pickup latency jitter, `IKService` synchronously queries `/param_server/get_param` (an in-memory, thread-safe hash map lookup). This guarantees < 1 ms latency and instant pickup of updated versions, well within the 2-second requirement.
- **Axis rotation**: `rotate_vector` directly applies the upper-left 3x3 rotation of the joint frame transform $T = \text{link\_pose}[\text{parent}] \cdot T_{\text{origin}}$, ensuring joint axes declared with arbitrary orientation or RPY are correctly transformed to the root-link frame.
- **Prismatic axis handling**: The axis is deliberately unnormalized in accordance with `spec/OVERVIEW.md` and `spec/IK_API.md`, preserving the derivative of the forward kinematics translation $q \cdot a$.

## Deviations from the plan

None. All steps proceeded as planned in `agent-notes/PLAN.md`.

## Known limitations or unresolved questions

- Full gradient step (`/ik/solve_robot_step`) and IK solver (`/ik/solve_robot`) are scoped for the final milestone and not part of the checkpoint.
- Descriptions near 1 MiB are part of final testing; verified line buffers and XML parser handle arbitrarily large buffers without 64 KiB restrictions.

## Checks performed

- Built cleanly with `make build` using C++20 and G++ 13 flags.
- Executed `tests-local/test_jacobian_verification.py`:
  * Verified two-link worked example matches spec within $10^{-4}$.
  * Verified finite difference numerical derivatives match analytical Jacobian rows within $10^{-4}$.
  * Verified prismatic joints with rotated frames and non-unit axes.
  * Verified multi-arm branch isolation and fixed joint omission.
  * Verified error handling and version tracking across dynamic reloads.
- Executed `python3 tests/run_tests.py`: 25 of 25 tests passed.
