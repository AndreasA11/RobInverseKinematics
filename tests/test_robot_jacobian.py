"""Tests for /ik/robot_jacobian service (Project 4 Checkpoint).
Verifies manipulator Jacobian on revolute, prismatic, and branching URDFs.
"""

import math
from rosbridge_client import Rosbridge

TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/><link name="upper"/><link name="fore"/><link name="tip"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
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
