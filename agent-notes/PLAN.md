# Project 4 Checkpoint: Robot Jacobian Service (/ik/robot_jacobian)

## Goal

Implement the `/ik/robot_jacobian` service on top of the Project 3 runtime, passing all checkpoint requirements (Categories A and B from `spec/OVERVIEW.md` and `spec/IK_API.md`). The service returns the manipulator Jacobian for any kinematic chain of a loaded URDF robot at any requested configuration in the root-link frame.

## Relevant requirements

1. **Protocol & Gateway (`127.0.0.1:9095`)**:
   - Speak the ROS-like newline-delimited TCP/JSON protocol.
   - Service responses follow `{"op": "service_response", "service": "/ik/robot_jacobian", "values": {...}, "result": <bool>, "status": <string>, "id": <req_id>}`.
   - Handle payload and line sizes up to 1 MiB without fixed 64 KiB buffer limits.
   - The service must answer within 10 seconds of `make run` starting.

2. **Parameter Server Integration & Version Tracking**:
   - Robot description is fetched from `/param_server/get_param` (`name: "robot_description"`).
   - `values.description_version`: Every response (success or error) MUST carry the version of `robot_description` it was computed for, or 0 if `robot_description` was never set.
   - Pickup bound: Within 2 seconds of a `set_param` returning version $V$, every reply must carry `description_version >= V` and reflect description $V$.
   - Invalid descriptions: If newest description is invalid (per `spec/ROBOT_DESCRIPTION.md`), the service must not crash or hang; it rejects requests with `result: false`, non-empty `status`, and `description_version` matching the invalid version. When the next valid description is set, it must be picked up.
   - No robot loaded: If `robot_description` was never set, requests must return `result: false`, non-empty `status`, and `values: {"description_version": 0}`.

3. **Chain Extraction**:
   - For link $e$ (`endeffector_frame`), trace the path from root link to $e$.
   - Extract movable joints (`revolute`, `continuous`, `prismatic`) in root-to-end-effector order.
   - Fixed joints along the chain contribute their `<origin>` to kinematics but have no row in the Jacobian and are not in `joint_names`.
   - Joints on other branches are ignored.
   - Unknown `endeffector_frame` must return `result: false` and non-empty `status`.

4. **Jacobian Computation**:
   - Forward kinematics computed at given configuration $q$.
   - Joint states: `name` and `position` arrays. If lengths differ or position for a chain movable joint is not a finite number (or is boolean), reject with `result: false` and non-empty `status`.
   - Movable joints not in `joint_state` default to 0. Names not in movable joints of the robot are ignored.
   - Joint positions are used as given (never clamped to limits for Jacobian calculation).
   - For end-effector point $p_e$ (origin of $e$ in root frame) and each chain movable joint $i$ with origin $p_i$ and rotation $R_i$ (after origin, before motion) and URDF axis $a_i$:
     * Revolute/Continuous: row is $[\hat{z}_i \times (p_e - p_i) ; \hat{z}_i]$, where $\hat{z}_i = R_i \cdot a_i / \|a_i\|$.
     * Prismatic: row is $[R_i \cdot a_i ; 0, 0, 0]$, where $a_i$ is used as written (NOT normalized).
   - Layout: $N \times 6$ (row $i$ corresponds to `joint_names[i]`).
   - Every entry within $10^{-4}$ of exact value.

## Repository observations

- `src/main.cpp`: Initializes `Middleware`, `ParameterServer`, `DescriptionLoader`, `WorldStatePublisher`, and `RosbridgeServer`.
- `include/parameter_server.hpp`: Thread-safe parameter server advertising `/param_server/set_param` and `/param_server/get_param`.
- `include/robot_model.hpp` / `src/robot_model.cpp`: Complete URDF parser (`parse_urdf`) conforming to `spec/ROBOT_DESCRIPTION.md`.
- `include/forward_kinematics.hpp` / `src/forward_kinematics.cpp`: Computes `joint_transforms` and `link_poses` in root-link frame.
- `include/math3d.hpp` / `src/math3d.cpp`: 3D vector and matrix math, needs cross product and vector rotation helper.
- `include/middleware/rosbridge_server.hpp`: Dynamically buffers TCP input line-by-line; service response handler needs clean support for explicit `status` string.

## Proposed architecture

1. **`math3d` extensions** (`include/math3d.hpp`, `src/math3d.cpp`):
   - Add `cross(const Vector3& a, const Vector3& b)`: standard 3D cross product.
   - Add `rotate_vector(const Matrix4& transform, const Vector3& v)`: rotates vector using upper-left 3x3 block of matrix.

2. **`RosbridgeServer` status enhancement** (`include/middleware/rosbridge_server.hpp`):
   - In `call_service` response builder, check if `result.second` has `"__status__"`. If present, extract to top-level `status` and remove from `values`. This allows service handlers to set custom non-empty status strings while keeping `values` clean and standard.

3. **`IKService` module** (`include/ik_service.hpp`, `src/ik_service.cpp`):
   - Class `IKService` taking `middleware::Middleware&`.
   - Advertises `/ik/robot_jacobian` on startup, unadvertises on destruction.
   - Internal robot model caching:
     * On each request, inspect `/param_server/get_param` for `robot_description`.
     * If version is unchanged, reuse cached model.
     * If version changed, parse URDF using `parse_urdf`. If valid, update cache and mark valid; if invalid, cache invalid status and version.
     * Zero-latency, thread-safe, meets the < 2s pickup bound instantaneously.
   - Handler `handle_robot_jacobian(const JsonValue& args)`:
     * Validate robot loaded and valid.
     * Validate `endeffector_frame` exists in model.
     * Validate `joint_state` format and lengths.
     * Build kinematic chain from root to `endeffector_frame`.
     * Extract movable joints and validate their positions.
     * Run FK to get `link_poses`.
     * Compute end-effector point $p_e$.
     * For each movable joint, compute $T_{root \to \text{joint}}$, $p_i$, $R_i$, and Jacobian row.
     * Format response JSON with `joint_names`, `jacobian`, `description_version`, `endeffector_frame`, and `status`.

4. **Wiring into `src/main.cpp` and `Makefile`**:
   - Instantiate `ik::IKService ik_service(hub);` in `main.cpp`.
   - Update `Makefile` compilation dependencies for `main`.

## Interfaces and data flow

```
External Client (TCP :9095)
         │
         ▼
RosbridgeServer (reads newline-delimited JSON)
         │
         ▼
Middleware::call_service("/ik/robot_jacobian", args)
         │
         ▼
IKService::handle_robot_jacobian
   ├── queries /param_server/get_param ("robot_description")
   ├── parses / retrieves cached RobotModel
   ├── validates endeffector_frame and joint_state
   ├── traces chain root -> endeffector_frame
   ├── evaluates compute_fk(model, joint_positions)
   └── calculates 6D Jacobian rows for each movable joint
         │
         ▼
JSON response: { "op": "service_response", "service": "/ik/robot_jacobian",
                 "values": { "joint_names": [...], "jacobian": [...],
                             "description_version": V, "endeffector_frame": "tip" },
                 "result": true, "status": "" }
```

## Implementation steps

1. Extend `math3d` with `cross` and `rotate_vector`.
2. Enhance `RosbridgeServer` service response builder for `__status__` extraction.
3. Implement `IKService` (`include/ik_service.hpp` and `src/ik_service.cpp`).
4. Update `src/main.cpp` to initialize `IKService`.
5. Update `Makefile` to compile `src/ik_service.cpp`.
6. Compile project with `make build` and verify clean build.
7. Create local unit/integration tests and run all checks.

## Edge cases and failure modes

- **No robot loaded**: Returns `result: false`, `status: "no robot loaded"`, `description_version: 0`.
- **Invalid URDF loaded**: Returns `result: false`, non-empty `status`, `description_version: invalid_version`.
- **Unknown end-effector link**: Returns `result: false`, non-empty `status`, echoes current `description_version`.
- **Mismatched name/position lengths in joint_state**: Returns `result: false`, non-empty `status`.
- **Non-numeric or boolean position for chain movable joint**: Returns `result: false`, non-empty `status`.
- **Positions for non-chain joints**: Ignored, does not fail request.
- **Movable joint missing in joint_state**: Defaults position to 0.0.
- **Fixed joints**: Contributed to FK transforms, skipped in Jacobian rows and `joint_names`.
- **Branches in robot tree**: Only joints strictly on the direct chain from root to end-effector are included.
- **Prismatic joints**: Axis used as written (unnormalized), angular velocity set to $[0, 0, 0]$.
- **Revolute/Continuous joints**: Axis normalized, linear velocity is $\hat{z} \times (p_e - p_i)$, angular velocity is $\hat{z}$.
- **Positions outside limits**: Used as given, not clamped.

## Verification strategy

- Verify two-link worked example from `spec/IK_API.md`:
  At shoulder = 0, elbow = $\pi/2$, rows must match `[-1, 1, 0, 0, 0, 1]` and `[-1, 0, 0, 0, 0, 1]` within $10^{-4}$.
- Test finite differences vs Forward Kinematics to confirm numerical derivatives.
- Test spatial chains with arbitrary RPY and rotated axes.
- Test prismatic joints with non-unit axis.
- Test branch exclusion on multi-arm robots.
- Test version tracking, invalid URDF handling, error rejection for bad inputs.
- Run all existing Project 3 tests to ensure zero regressions.
