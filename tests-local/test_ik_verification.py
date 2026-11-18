#!/usr/bin/env python3
import json
import math
import socket
import subprocess
import sys
import time

TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/><link name="upper"/><link name="fore"/><link name="tip"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/>
    <axis xyz="0 0 1"/><limit lower="-3.0" upper="3.0"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-2.0" upper="2.0"/></joint>
  <joint name="tip_mount" type="fixed"><parent link="fore"/><child link="tip"/>
    <origin xyz="1 0 0"/></joint>
</robot>"""

SIX_DOF_ARM_URDF = """<?xml version="1.0"?>
<robot name="arm6">
  <link name="base_link"/>
  <link name="link1"/><link name="link2"/><link name="link3"/>
  <link name="link4"/><link name="link5"/><link name="link6"/>
  
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.2" rpy="1.57079632679 0 0"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5"/></joint>
  <joint name="j3" type="revolute"><parent link="link2"/><child link="link3"/>
    <origin xyz="0.4 0 0"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5"/></joint>
  <joint name="j4" type="revolute"><parent link="link3"/><child link="link4"/>
    <origin xyz="0.4 0 0" rpy="-1.57079632679 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
  <joint name="j5" type="revolute"><parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0.3" rpy="1.57079632679 0 0"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5"/></joint>
  <joint name="j6" type="revolute"><parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0.1" rpy="-1.57079632679 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
</robot>"""

def call_service(sock, service, args):
    call_id = f"call-{time.time_ns()}"
    msg = {"op": "call_service", "service": service, "args": args, "id": call_id}
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(65536)
        if not chunk:
            raise RuntimeError("Connection closed")
        buf += chunk
    line = buf.split(b"\n")[0]
    resp = json.loads(line.decode("utf-8"))
    assert resp.get("op") == "service_response", f"Unexpected op: {resp}"
    return resp

def main():
    print("Starting ./main server process...")
    proc = subprocess.Popen(["./main"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.5)

    try:
        sock = socket.create_connection(("127.0.0.1", 9095), timeout=5)
        print("Connected to 127.0.0.1:9095.")

        # Test 1: No robot loaded rejection on step and solve
        print("\n--- Test 1: Rejection before robot_description set ---")
        step_resp = call_service(sock, "/ik/solve_robot_step", {
            "joint_state": {"name": [], "position": []},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": {"x": 1, "y": 1, "z": 0}}
        })
        assert step_resp["result"] is False
        assert step_resp["values"]["description_version"] == 0
        assert len(step_resp["status"]) > 0

        solve_resp = call_service(sock, "/ik/solve_robot", {
            "joint_state": {"name": [], "position": []},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": {"x": 1, "y": 1, "z": 0}},
            "position_tolerance": 0.01,
            "max_iterations": 100
        })
        assert solve_resp["result"] is False
        assert solve_resp["values"]["description_version"] == 0
        print("PASSED: No robot loaded rejection.")

        # Test 2: Load two-link arm and test solve_robot_step
        print("\n--- Test 2: solve_robot_step on two-link arm ---")
        set_res = call_service(sock, "/param_server/set_param", {
            "name": "robot_description", "value": TWO_LINK_URDF
        })
        assert set_res["result"] is True
        ver = set_res["values"]["version"]

        # 2a: JacobianTranspose
        q1, q2 = 0.3, 0.9
        target = {"x": 1.2, "y": 0.8, "z": 0.0}
        step_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [q1, q2]},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": target},
            "ik_position_only": True,
            "method": "JacobianTranspose",
            "step_length": 0.05,
            "damping": 0.2
        }
        res_step = call_service(sock, "/ik/solve_robot_step", step_req)
        assert res_step["result"] is True
        assert res_step["values"]["description_version"] == ver
        sol = res_step["values"]["configuration_solution"]
        assert sol["name"] == ["shoulder", "elbow"]
        # Expected from analytical FK and Jacobian transpose: [0.279054, 0.897738]
        print(f"JacobianTranspose step solution: {sol['position']}")
        assert abs(sol["position"][0] - 0.279054) < 1e-4
        assert abs(sol["position"][1] - 0.897738) < 1e-4

        # 2b: JacobianPseudoinverse (Damped least squares)
        step_req["method"] = "JacobianPseudoinverse"
        res_dls = call_service(sock, "/ik/solve_robot_step", step_req)
        assert res_dls["result"] is True
        sol_dls = res_dls["values"]["configuration_solution"]
        print(f"JacobianPseudoinverse step solution: {sol_dls['position']}")
        # Expected from analytical DLS: [0.276920, 0.933812]
        assert abs(sol_dls["position"][0] - 0.276920) < 1e-4
        assert abs(sol_dls["position"][1] - 0.933812) < 1e-4

        # 2c: Statelessness
        res_step2 = call_service(sock, "/ik/solve_robot_step", step_req)
        assert res_step2["values"]["configuration_solution"]["position"] == sol_dls["position"]
        print("PASSED: solve_robot_step verification.")

        # Test 3: solve_robot position-only convergence
        print("\n--- Test 3: solve_robot position-only convergence ---")
        # Target within reachable workspace: shoulder=0.5, elbow=0.5
        # pe = (cos(0.5)+cos(1.0), sin(0.5)+sin(1.0), 0) = (1.41787, 1.32089, 0)
        target_pos = {"x": 1.41787, "y": 1.32089, "z": 0.0}
        solve_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [0.0, 0.0]},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": target_pos},
            "ik_position_only": True,
            "method": "JacobianPseudoinverse",
            "step_length": 0.5,
            "damping": 0.05,
            "position_tolerance": 1e-3,
            "max_iterations": 200
        }
        res_solve = call_service(sock, "/ik/solve_robot", solve_req)
        assert res_solve["result"] is True, f"Solve failed: {res_solve}"
        assert res_solve["values"]["converged"] is True
        assert res_solve["status"] == "converged"
        final_err = res_solve["values"]["final_error"]
        assert final_err["position"] < 1e-3
        assert final_err["orientation"] is None
        print(f"Converged in {res_solve['values']['iterations']} iterations, final error: {final_err['position']}")
        print("PASSED: solve_robot position-only convergence.")

        # Test 4: Unreachable target honesty
        print("\n--- Test 4: Unreachable target honesty ---")
        unreach_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [0.0, 0.0]},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": {"x": 10.0, "y": 10.0, "z": 0.0}},
            "ik_position_only": True,
            "method": "JacobianTranspose",
            "step_length": 0.1,
            "damping": 0.0,
            "position_tolerance": 1e-3,
            "max_iterations": 50
        }
        res_unreach = call_service(sock, "/ik/solve_robot", unreach_req)
        assert res_unreach["result"] is False, "Unreachable target must report result: false"
        assert res_unreach["values"]["converged"] is False
        assert res_unreach["status"] == "did not converge"
        assert "configuration_solution" in res_unreach["values"]
        assert res_unreach["values"]["final_error"]["position"] > 1.0
        print("PASSED: Unreachable target honesty.")

        # Test 5: Joint limits enforcement
        print("\n--- Test 5: Joint limits enforcement ---")
        # Target far in +x direction: elbow max is 2.0, shoulder max is 3.0
        limit_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [0.0, 0.0]},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": {"x": 5.0, "y": 0.0, "z": 0.0}},
            "ik_position_only": True,
            "method": "JacobianTranspose",
            "step_length": 0.2,
            "damping": 0.0,
            "position_tolerance": 1e-4,
            "max_iterations": 30
        }
        res_limit = call_service(sock, "/ik/solve_robot", limit_req)
        sol_pos = res_limit["values"]["configuration_solution"]["position"]
        # shoulder in [-3, 3], elbow in [-2, 2]
        assert -3.0 <= sol_pos[0] <= 3.0, f"Shoulder violated limit: {sol_pos[0]}"
        assert -2.0 <= sol_pos[1] <= 2.0, f"Elbow violated limit: {sol_pos[1]}"
        print(f"Limiting respected: shoulder={sol_pos[0]}, elbow={sol_pos[1]}")
        print("PASSED: Joint limits enforcement.")

        # Test 6: Full pose solve on 6-DOF arm
        print("\n--- Test 6: Full-pose solve on 6-DOF arm ---")
        set_arm = call_service(sock, "/param_server/set_param", {
            "name": "robot_description", "value": SIX_DOF_ARM_URDF
        })
        assert set_arm["result"] is True
        ver_arm = set_arm["values"]["version"]

        # Forward kinematics of initial pose: all joints at 0.1
        # Let's solve toward a known nearby pose
        pose_req = {
            "joint_state": {
                "name": ["j1", "j2", "j3", "j4", "j5", "j6"],
                "position": [0.0, 0.2, -0.3, 0.1, 0.2, -0.1]
            },
            "endeffector_frame": "link6",
            "endeffector_pose": {
                "position": {"x": 0.45, "y": 0.1, "z": 0.65},
                "orientation": {"x": 0.0, "y": 0.70710678, "z": 0.0, "w": 0.70710678}
            },
            "ik_position_only": False,
            "method": "JacobianPseudoinverse",
            "step_length": 0.3,
            "damping": 0.1,
            "position_tolerance": 5e-3,
            "orientation_tolerance": 5e-2,
            "max_iterations": 300
        }
        res_pose = call_service(sock, "/ik/solve_robot", pose_req)
        print("Full pose solve result:", res_pose["result"], res_pose["values"].get("converged"))
        print("Final error:", res_pose["values"]["final_error"])
        assert res_pose["values"]["description_version"] == ver_arm
        assert res_pose["values"]["converged"] is True
        assert res_pose["result"] is True
        assert res_pose["values"]["final_error"]["position"] < 5e-3
        assert res_pose["values"]["final_error"]["orientation"] < 5e-2
        print("PASSED: Full-pose solve.")

        # Test 7: Large URDF description (~1 MiB)
        print("\n--- Test 7: Large URDF near 1 MiB line size ---")
        large_urdf = '<?xml version="1.0"?>\n<robot name="large_bot">\n  <link name="base"/>\n'
        for i in range(1, 15000):
            large_urdf += f'  <link name="l{i}"/>\n'
            large_urdf += f'  <joint name="j{i}" type="fixed"><parent link="base"/><child link="l{i}"/></joint>\n'
        large_urdf += '  <link name="ee"/>\n'
        large_urdf += '  <joint name="jee" type="revolute"><parent link="base"/><child link="ee"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>\n'
        large_urdf += '</robot>\n'
        urdf_bytes = len(large_urdf.encode("utf-8"))
        print(f"Generated large URDF size: {urdf_bytes / 1024:.1f} KiB")
        
        t0 = time.monotonic()
        set_large = call_service(sock, "/param_server/set_param", {
            "name": "robot_description", "value": large_urdf
        })
        assert set_large["result"] is True
        ver_large = set_large["values"]["version"]
        print(f"Set large URDF in {time.monotonic() - t0:.2f} s")

        t0 = time.monotonic()
        req_large = {
            "joint_state": {"name": ["jee"], "position": [0.5]},
            "endeffector_frame": "ee"
        }
        res_large = call_service(sock, "/ik/robot_jacobian", req_large)
        assert res_large["result"] is True
        assert res_large["values"]["description_version"] == ver_large
        print(f"Jacobian on large URDF answered in {time.monotonic() - t0:.2f} s")
        print("PASSED: Large URDF support.")

        print("\nALL VERIFICATION TESTS PASSED SUCCESSFULLY!")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

if __name__ == "__main__":
    main()
