"""Tests for /ik/robot_jacobian, /ik/solve_robot_step, and /ik/solve_robot services.
Verifies Jacobian, gradient step (transpose and damped pseudoinverse), and iterative IK solve.
"""

import math
from rosbridge_client import Rosbridge

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

TREE_URDF = """<?xml version="1.0"?>
<robot name="tree_bot">
  <link name="base"/>
  <link name="left_1"/><link name="left_tip"/>
  <link name="right_1"/><link name="right_tip"/>
  <joint name="left_j1" type="revolute"><parent link="base"/><child link="left_1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="left_j2" type="prismatic"><parent link="left_1"/><child link="left_tip"/>
    <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="2 0 0"/><limit lower="-3" upper="3"/></joint>
  <joint name="right_j1" type="revolute"><parent link="base"/><child link="right_1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="right_j2" type="revolute"><parent link="right_1"/><child link="right_tip"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
</robot>"""


def test_robot_jacobian_two_link_arm():
    with Rosbridge() as ros:
        s_reply = ros.call_service("/param_server/set_param", {
            "name": "robot_description", "value": TWO_LINK_URDF
        })
        assert s_reply.get("result") is True, f"Failed to set description: {s_reply}"
        ver = s_reply["values"]["version"]

        req = {
            "joint_state": {
                "name": ["shoulder", "elbow"],
                "position": [0.0, math.pi / 2.0]
            },
            "endeffector_frame": "tip"
        }
        reply = ros.call_service("/ik/robot_jacobian", req)
        assert reply.get("result") is True, f"Service failed: {reply}"
        assert reply["values"]["description_version"] == ver
        assert reply["values"]["joint_names"] == ["shoulder", "elbow"]

        expected = [
            [-1.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ]
        actual = reply["values"]["jacobian"]
        for exp_row, act_row in zip(expected, actual):
            for e, a in zip(exp_row, act_row):
                assert abs(e - a) < 1e-4, f"Value mismatch: {act_row} vs {exp_row}"


def test_robot_jacobian_prismatic_and_branch_isolation():
    with Rosbridge() as ros:
        s_reply = ros.call_service("/param_server/set_param", {
            "name": "robot_description", "value": TREE_URDF
        })
        assert s_reply.get("result") is True
        ver = s_reply["values"]["version"]

        # Call with extra joints on right arm; right arm must not be in chain
        req = {
            "joint_state": {
                "name": ["left_j1", "left_j2", "right_j1"],
                "position": [0.0, 1.0, 0.5]
            },
            "endeffector_frame": "left_tip"
        }
        reply = ros.call_service("/ik/robot_jacobian", req)
        assert reply.get("result") is True
        assert reply["values"]["description_version"] == ver
        assert reply["values"]["joint_names"] == ["left_j1", "left_j2"]

        # left_j2 is prismatic: axis (2, 0, 0) rotated by pi/2 yaw -> (0, 2, 0)
        p_row = reply["values"]["jacobian"][1]
        assert abs(p_row[0] - 0.0) < 1e-4
        assert abs(p_row[1] - 2.0) < 1e-4
        assert abs(p_row[2] - 0.0) < 1e-4
        assert abs(p_row[3] - 0.0) < 1e-4
        assert abs(p_row[4] - 0.0) < 1e-4
        assert abs(p_row[5] - 0.0) < 1e-4


def test_solve_robot_step_transpose_and_dls():
    with Rosbridge() as ros:
        s_reply = ros.call_service("/param_server/set_param", {
            "name": "robot_description", "value": TWO_LINK_URDF
        })
        assert s_reply.get("result") is True
        ver = s_reply["values"]["version"]

        # 1. JacobianTranspose step
        step_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [0.3, 0.9]},
            "endeffector_frame": "tip",
            "endeffector_pose": {
                "position": {"x": 1.2, "y": 0.8, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            },
            "ik_position_only": True,
            "method": "JacobianTranspose",
            "step_length": 0.05,
            "damping": 0.2
        }
        res_trans = ros.call_service("/ik/solve_robot_step", step_req)
        assert res_trans.get("result") is True, f"Step failed: {res_trans}"
        assert res_trans["values"]["description_version"] == ver
        pos_trans = res_trans["values"]["configuration_solution"]["position"]
        assert abs(pos_trans[0] - 0.279054) < 1e-4
        assert abs(pos_trans[1] - 0.897738) < 1e-4

        # 2. JacobianPseudoinverse (damped least squares)
        step_req["method"] = "JacobianPseudoinverse"
        res_dls = ros.call_service("/ik/solve_robot_step", step_req)
        assert res_dls.get("result") is True, f"DLS step failed: {res_dls}"
        pos_dls = res_dls["values"]["configuration_solution"]["position"]
        assert abs(pos_dls[0] - 0.276920) < 1e-4
        assert abs(pos_dls[1] - 0.933812) < 1e-4


def test_solve_robot_convergence_and_limits():
    with Rosbridge() as ros:
        s_reply = ros.call_service("/param_server/set_param", {
            "name": "robot_description", "value": TWO_LINK_URDF
        })
        assert s_reply.get("result") is True
        ver = s_reply["values"]["version"]

        # 1. Reachable target position solve
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
        res_solve = ros.call_service("/ik/solve_robot", solve_req)
        assert res_solve.get("result") is True, f"Solve failed: {res_solve}"
        assert res_solve["values"]["converged"] is True
        assert res_solve["status"] == "converged"
        assert res_solve["values"]["description_version"] == ver
        assert res_solve["values"]["final_error"]["position"] < 1e-3
        assert res_solve["values"]["final_error"]["orientation"] is None

        # 2. Unreachable target reporting
        unreach_req = {
            "joint_state": {"name": ["shoulder", "elbow"], "position": [0.0, 0.0]},
            "endeffector_frame": "tip",
            "endeffector_pose": {"position": {"x": 10.0, "y": 10.0, "z": 0.0}},
            "ik_position_only": True,
            "method": "JacobianTranspose",
            "step_length": 0.1,
            "damping": 0.0,
            "position_tolerance": 1e-3,
            "max_iterations": 20
        }
        res_unreach = ros.call_service("/ik/solve_robot", unreach_req)
        assert res_unreach["result"] is False
        assert res_unreach["values"]["converged"] is False
        assert len(res_unreach["status"]) > 0
        assert res_unreach["values"]["final_error"]["position"] > 1.0

        # 3. Limits respected: shoulder in [-3, 3], elbow in [-2, 2]
        sol_pos = res_unreach["values"]["configuration_solution"]["position"]
        assert -3.0 <= sol_pos[0] <= 3.0
        assert -2.0 <= sol_pos[1] <= 2.0
