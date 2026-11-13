#!/usr/bin/env python3
import json
import math
import socket
import sys
import time

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
    sock = socket.create_connection(("127.0.0.1", 9095))
    print("[1] Testing before robot_description set...")
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": [], "position": []},
        "endeffector_frame": "tip"
    })
    print("No robot reply:", resp)
    assert resp["result"] is False, "Expected result: false when no robot loaded"
    assert resp["values"].get("description_version") == 0, f"Expected version 0, got {resp['values']}"
    assert len(resp["status"]) > 0, "Expected non-empty status"
    print("PASSED: No robot test.")

    print("\n[2] Testing worked two-link arm example...")
    two_link_urdf = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/><link name="upper"/><link name="fore"/><link name="tip"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="tip_mount" type="fixed"><parent link="fore"/><child link="tip"/>
    <origin xyz="1 0 0"/></joint>
</robot>"""

    set_reply = call_service(sock, "/param_server/set_param", {
        "name": "robot_description", "value": two_link_urdf
    })
    assert set_reply["result"] is True
    version1 = set_reply["values"]["version"]
    print(f"Loaded two_link version {version1}")

    jac_req = {
        "joint_state": {
            "name": ["shoulder", "elbow"],
            "position": [0.0, math.pi / 2.0]
        },
        "endeffector_frame": "tip"
    }
    resp = call_service(sock, "/ik/robot_jacobian", jac_req)
    print("Worked example reply:", resp)
    assert resp["result"] is True
    assert resp["values"]["description_version"] == version1
    assert resp["values"]["joint_names"] == ["shoulder", "elbow"]
    
    # Expected: shoulder: [-1, 1, 0, 0, 0, 1], elbow: [-1, 0, 0, 0, 0, 1]
    expected_rows = [
        [-1.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    ]
    actual_rows = resp["values"]["jacobian"]
    for name, exp, act in zip(resp["values"]["joint_names"], expected_rows, actual_rows):
        print(f"Joint {name}: actual={act}, expected={exp}")
        for e, a in zip(exp, act):
            assert abs(e - a) < 1e-4, f"Mismatch in {name}: expected {e}, got {a}"
    print("PASSED: Worked example test.")

    print("\n[3] Testing finite difference comparison...")
    # At an arbitrary configuration q = [0.3, 0.7]
    q = [0.3, 0.7]
    jac_req2 = {
        "joint_state": {"name": ["shoulder", "elbow"], "position": q},
        "endeffector_frame": "tip"
    }
    resp2 = call_service(sock, "/ik/robot_jacobian", jac_req2)
    assert resp2["result"] is True
    J = resp2["values"]["jacobian"]

    # Compute tip position at q using forward kinematics formula:
    # shoulder rotates by q[0]: upper origin is at 0, rotates by q[0]
    # elbow origin is at (1, 0, 0) relative to upper, so in base: (cos(q0), sin(q0), 0)
    # fore rotates by q0 + q1
    # tip origin is at (1, 0, 0) relative to fore, so in base:
    # tip = (cos(q0) + cos(q0 + q1), sin(q0) + sin(q0 + q1), 0)
    def fk(q0, q1):
        return (
            math.cos(q0) + math.cos(q0 + q1),
            math.sin(q0) + math.sin(q0 + q1),
            0.0
        )

    h = 1e-6
    p0 = fk(q[0], q[1])
    p_q0 = fk(q[0] + h, q[1])
    p_q1 = fk(q[0], q[1] + h)

    dp_dq0 = [(p_q0[i] - p0[i]) / h for i in range(3)]
    dp_dq1 = [(p_q1[i] - p0[i]) / h for i in range(3)]

    print("Shoulder: numeric linear v:", dp_dq0, "Jacobian row:", J[0][:3])
    for num, jac in zip(dp_dq0, J[0][:3]):
        assert abs(num - jac) < 1e-4, f"Finite diff mismatch shoulder: {num} vs {jac}"

    print("Elbow: numeric linear v:", dp_dq1, "Jacobian row:", J[1][:3])
    for num, jac in zip(dp_dq1, J[1][:3]):
        assert abs(num - jac) < 1e-4, f"Finite diff mismatch elbow: {num} vs {jac}"
    print("PASSED: Finite difference test.")

    print("\n[4] Testing prismatic joint with non-unit axis and rotated origin...")
    prismatic_urdf = """<?xml version="1.0"?>
<robot name="prismatic_arm">
  <link name="base"/><link name="slider"/><link name="tip"/>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/>
    <origin xyz="0 1 0" rpy="0 0 1.5707963267948966"/>
    <axis xyz="2 0 0"/><limit lower="-5" upper="5"/></joint>
  <joint name="tip_fixed" type="fixed"><parent link="slider"/><child link="tip"/>
    <origin xyz="0.5 0 0"/></joint>
</robot>"""
    set_reply = call_service(sock, "/param_server/set_param", {
        "name": "robot_description", "value": prismatic_urdf
    })
    version2 = set_reply["values"]["version"]

    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": ["slide"], "position": [1.0]},
        "endeffector_frame": "tip"
    })
    print("Prismatic reply:", resp)
    assert resp["result"] is True
    assert resp["values"]["description_version"] == version2
    assert resp["values"]["joint_names"] == ["slide"]
    # Prismatic axis is (2, 0, 0) rotated by yaw = pi/2 -> (0, 2, 0)
    # Angular velocity must be 0, 0, 0
    row = resp["values"]["jacobian"][0]
    print("Prismatic row:", row)
    expected_prism = [0.0, 2.0, 0.0, 0.0, 0.0, 0.0]
    for e, a in zip(expected_prism, row):
        assert abs(e - a) < 1e-4, f"Prismatic row mismatch: expected {e}, got {a}"
    print("PASSED: Prismatic joint test.")

    print("\n[5] Testing multi-branch tree with branching arm...")
    tree_urdf = """<?xml version="1.0"?>
<robot name="tree_bot">
  <link name="base"/>
  <link name="left_1"/><link name="left_tip"/>
  <link name="right_1"/><link name="right_tip"/>
  <joint name="left_j1" type="revolute"><parent link="base"/><child link="left_1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="left_j2" type="revolute"><parent link="left_1"/><child link="left_tip"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="right_j1" type="revolute"><parent link="base"/><child link="right_1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="right_j2" type="revolute"><parent link="right_1"/><child link="right_tip"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
</robot>"""
    set_reply = call_service(sock, "/param_server/set_param", {
        "name": "robot_description", "value": tree_urdf
    })
    version3 = set_reply["values"]["version"]

    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {
            "name": ["left_j1", "left_j2", "right_j1", "right_j2"],
            "position": [0.0, 1.5707963267948966, 0.5, 0.5]
        },
        "endeffector_frame": "left_tip"
    })
    assert resp["result"] is True
    # Only left arm joints must be in joint_names
    assert resp["values"]["joint_names"] == ["left_j1", "left_j2"], f"Wrong joints: {resp['values']['joint_names']}"
    assert len(resp["values"]["jacobian"]) == 2
    print("PASSED: Multi-branch branch isolation test.")

    print("\n[6] Testing error cases...")
    # Unknown link
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": [], "position": []},
        "endeffector_frame": "nonexistent_link"
    })
    assert resp["result"] is False
    assert resp["values"]["description_version"] == version3
    assert len(resp["status"]) > 0

    # Mismatched name/position
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": ["left_j1"], "position": [1.0, 2.0]},
        "endeffector_frame": "left_tip"
    })
    assert resp["result"] is False
    assert resp["values"]["description_version"] == version3
    assert len(resp["status"]) > 0

    # Non-number boolean position for chain movable joint
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": ["left_j1", "left_j2"], "position": [True, 0.0]},
        "endeffector_frame": "left_tip"
    })
    assert resp["result"] is False
    assert resp["values"]["description_version"] == version3
    assert len(resp["status"]) > 0

    # Non-number position for non-chain joint should be ignored!
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": ["left_j1", "left_j2", "right_j1"], "position": [0.0, 0.0, "not_a_number"]},
        "endeffector_frame": "left_tip"
    })
    assert resp["result"] is True
    assert resp["values"]["joint_names"] == ["left_j1", "left_j2"]

    # Invalid description reload test
    set_reply = call_service(sock, "/param_server/set_param", {
        "name": "robot_description", "value": "<invalid xml"
    })
    invalid_ver = set_reply["values"]["version"]
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": [], "position": []},
        "endeffector_frame": "left_tip"
    })
    assert resp["result"] is False
    assert resp["values"]["description_version"] == invalid_ver
    assert len(resp["status"]) > 0

    # Next valid description recovers
    set_reply = call_service(sock, "/param_server/set_param", {
        "name": "robot_description", "value": two_link_urdf
    })
    recovered_ver = set_reply["values"]["version"]
    resp = call_service(sock, "/ik/robot_jacobian", {
        "joint_state": {"name": [], "position": []},
        "endeffector_frame": "tip"
    })
    assert resp["result"] is True
    assert resp["values"]["description_version"] == recovered_ver
    print("PASSED: Error cases and recovery tests.")

    print("\nALL VERIFICATION CHECKS PASSED SUCCESSFULLY!")

if __name__ == "__main__":
    main()
