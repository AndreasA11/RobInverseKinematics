# Audit Report: Project 4 Checkpoint (/ik/robot_jacobian)

## Summary

The checkpoint implementation of `/ik/robot_jacobian` satisfies the specifications outlined in `spec/OVERVIEW.md`, `spec/IK_API.md`, and `spec/ROBOT_DESCRIPTION.md`. The service correctly answers on `127.0.0.1:9095`, reflects the parameter server's `robot_description` versioning, isolates the requested kinematic chain from other tree branches, handles fixed joints properly, calculates accurate analytical Jacobian rows ($N \times 6$) within the $10^{-4}$ tolerance, and rejects malformed inputs gracefully.

## Findings

- **Severity:** Informational
  **Location:** `src/ik_service.cpp`
  **Problem:** Missing movable joints in `joint_state` default to 0.0 without warning.
  **Why it matters:** Requirement in `spec/IK_API.md` line 70: "A movable joint of the robot that joint_state does not name is at 0."
  **Recommended action:** None; this is conforming behavior per specification.

- **Severity:** Informational
  **Location:** `src/ik_service.cpp`
  **Problem:** Prismatic joints do not normalize their axis vector.
  **Why it matters:** Requirement in `spec/IK_API.md` line 99: "with a_i not normalized (the derivative of q·axis)".
  **Recommended action:** None; intentional and verified via numerical differentiation.

- **Severity:** Informational
  **Location:** `include/middleware/rosbridge_server.hpp`
  **Problem:** Added internal service `__status__` support.
  **Why it matters:** Allows internal C++ service handlers to return exact custom status strings while cleanly separating `values` payload.
  **Recommended action:** Ensure future final project services (`/ik/solve_robot_step`, `/ik/solve_robot`) utilize this convention.

## Unverified risks

1. **Very deep chains (> 50 movable joints)**: Performance and numerical precision under extremely long serial chains should be monitored during final milestone stress testing.
2. **Near 1 MiB URDF descriptions**: While buffers are dynamic and verified offline up to 100+ KiB (PR2 scale), grading for near-1 MiB URDFs is explicitly deferred to the final project grading suite.
