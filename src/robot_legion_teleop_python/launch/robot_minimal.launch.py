#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
robot_minimal.launch.py

Minimal “make it drivable” launch:
  - motor_driver_node
  - heartbeat_node

Use this when:
  - You do NOT need playbooks/action execution on the robot yet.
  - You want the smallest possible set of nodes for reliability.

This still supports heterogeneous fleets, because:
  - motor_driver_node picks diff_drive vs mecanum from robot_profiles.yaml
  - heartbeat_node advertises the chosen drive_type so teleop can adapt
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_robot_name = os.environ.get("USER", "robot")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value=default_robot_name,
        description="Robot identity name. Default: Linux username.",
    )

    profiles_path_arg = DeclareLaunchArgument(
        "profiles_path",
        default_value="",
        description="Optional path to robot_profiles.yaml; empty uses installed config.",
    )

    common_params = [
        {"robot_name": LaunchConfiguration("robot_name")},
        {"profiles_path": LaunchConfiguration("profiles_path")},
    ]

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

    return LaunchDescription(
        [
            robot_name_arg,
            profiles_path_arg,
            motor_driver,
            heartbeat,
        ]
    )
