#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
robot_bringup.launch.py

Robot-side bringup launch for *one robot*.

Goal:
- Start everything needed ON THE ROBOT so the laptop teleop can drive it.

Starts:
  1) motor_driver_node
      - Reads robot_profiles.yaml to determine drive profile (diff_drive vs mecanum, etc)
      - Exposes a uniform control interface so teleop/playbooks don't care about drive type

  2) heartbeat_node
      - Periodically publishes a small "I'm alive + my capabilities" heartbeat
      - This lets teleop discover robots and see their drive profile/capabilities

Optional (enable via launch args):
  3) unit_executor_action_server
      - Robot-side action server that can execute playbooks

  4) fpv_control_arbiter
      - Robot-side arbitration for "who has control" (local vs remote / pilot selection)

IMPORTANT ROS2 LAUNCH NOTE:
- You CANNOT do: node.condition = ...
- Conditions must be supplied at construction time:
    Node(..., condition=IfCondition(...))
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Default robot name: Linux username (your convention)
    default_robot_name = os.environ.get("USER", "robot")

    # -------------------------
    # Launch arguments
    # -------------------------
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value=default_robot_name,
        description=(
            "Robot identity name used in heartbeat/teleop. "
            "Default: Linux username."
        ),
    )

    profiles_path_arg = DeclareLaunchArgument(
        "profiles_path",
        default_value="",
        description=(
            "Optional: explicit path to robot_profiles.yaml. "
            "If empty, nodes use the installed config in share/<pkg>/config/robot_profiles.yaml."
        ),
    )

    enable_playbook_arg = DeclareLaunchArgument(
        "enable_playbook",
        default_value="true",
        description="Start unit_executor_action_server (robot-side playbook executor).",
    )

    enable_fpv_arg = DeclareLaunchArgument(
        "enable_fpv",
        default_value="false",
        description="Start fpv_control_arbiter (robot-side FPV control arbitration).",
    )

    # These parameters are passed into multiple nodes.
    # Each node reads robot_name + profiles_path the same way.
    common_params = [
        {"robot_name": LaunchConfiguration("robot_name")},
        {"profiles_path": LaunchConfiguration("profiles_path")},
    ]

    # -------------------------
    # Always-on nodes
    # -------------------------
    motor_driver = Node(
        package="robot_legion_teleop_python",
        executable="motor_driver_node",
        name="motor_driver_node",
        output="screen",
        parameters=common_params,
    )

    heartbeat = Node(
        package="robot_legion_teleop_python",
        executable="heartbeat_node",
        name="heartbeat_node",
        output="screen",
        parameters=common_params,
    )

    # -------------------------
    # Optional nodes (note condition=... is set HERE)
    # -------------------------
    unit_executor = Node(
        package="robot_legion_teleop_python",
        executable="unit_executor_action_server",
        name="unit_executor_action_server",
        output="screen",
        parameters=common_params,
        condition=IfCondition(LaunchConfiguration("enable_playbook")),
    )

    fpv_control = Node(
        package="robot_legion_teleop_python",
        executable="fpv_control_arbiter",
        name="fpv_control_arbiter",
        output="screen",
        parameters=common_params,
        condition=IfCondition(LaunchConfiguration("enable_fpv")),
    )

    # -------------------------
    # Return the launch description
    # -------------------------
    return LaunchDescription(
        [
            robot_name_arg,
            profiles_path_arg,
            enable_playbook_arg,
            enable_fpv_arg,
            motor_driver,
            heartbeat,
            unit_executor,
            fpv_control,
        ]
    )
