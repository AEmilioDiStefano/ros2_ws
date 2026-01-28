#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
robot_bringup.launch.py

Purpose:
  Start ALL ROS 2 nodes that should run ON A PHYSICAL ROBOT to make it drivable
  by the fleet teleop / orchestrator system.

What this launch file starts (on the robot):
  1) motor_driver_node
     - Subscribes to /<robot_name>/cmd_vel (by default via the profile template)
     - Translates Twist -> GPIO motor outputs based on the selected drive profile
     - Drive type is resolved from robot_profiles.yaml (diff_drive vs mecanum, etc)

  2) heartbeat_node
     - Publishes /<robot_name>/heartbeat containing the robot’s drive_type, hardware,
       and profile name so that teleop can “learn” the drive type automatically.

  3) unit_executor_action_server (optional)
     - Provides an action interface so playbooks can be executed on the robot.

  4) usb_camera_node (optional)
     - If you want FPV from this robot. This is kept optional because not every robot
       needs a camera, and camera drivers can differ.
       
       ^^ FIXME ^^: FIND A WAY TO MAKE CAMERA USAGE "CAMERA-DRIVER-AGNOSTIC"

Key idea for easy deployment:
  - Each robot has its *own local copy* of config/robot_profiles.yaml.
  - On that robot you set ONE line: top-level `drive_profile: <profile_name>`
    and everything else self-configures.

HOW TO ADD NODES:
  - Add another Node(...) block below (search for "ADD FUTURE NODES HERE").
  - If it needs parameters, extend the shared `common_params` list/dict.
  - If it needs remaps, add remappings=[(...), (...)] to that Node.

Run examples:
  ros2 launch robot_legion_teleop_python robot_bringup.launch.py
  ros2 launch robot_legion_teleop_python robot_bringup.launch.py run_camera:=true
  ros2 launch robot_legion_teleop_python robot_bringup.launch.py run_executor:=false

Notes:
  - robot_name defaults to the Linux username (same default used in the nodes).
  - profiles_path defaults to "" which means: use the package’s installed config YAML.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ----------------------------
    # Launch arguments (user-tunable knobs)
    # ----------------------------

    # Default robot identity:
    # We default to the Linux username because your nodes already do that
    # (getpass.getuser()) and it naturally yields unique names if you set each robot’s user.
    default_robot_name = os.environ.get("USER", "robot")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value=default_robot_name,
        description=(
            "Robot name / namespace key. This must match what teleop will address. "
            "Default: current Linux username."
        ),
    )

    # Optional override: point to a specific YAML file on disk.
    # If left empty, nodes use the default installed file:
    #   <install>/share/robot_legion_teleop_python/config/robot_profiles.yaml
    profiles_path_arg = DeclareLaunchArgument(
        "profiles_path",
        default_value="",
        description=(
            "Optional path to robot_profiles.yaml. "
            "Leave empty to use the package-installed config."
        ),
    )

    run_executor_arg = DeclareLaunchArgument(
        "run_executor",
        default_value="true",
        description=(
            "If true, start unit_executor_action_server on this robot. "
            "Disable if you only want manual teleop driving."
        ),
    )

    run_camera_arg = DeclareLaunchArgument(
        "run_camera",
        default_value="false",
        description=(
            "If true, start usb_camera_node on this robot for FPV. "
            "Kept optional because camera setups vary."
        ),
    )

    # ----------------------------
    # Shared parameters passed into multiple nodes
    # ----------------------------
    #
    # These match what your nodes already declare:
    # - motor_driver_node declares robot_name and profiles_path :contentReference[oaicite:4]{index=4}
    # - heartbeat_node declares robot_name and profiles_path :contentReference[oaicite:5]{index=5}
    #
    common_params = [
        {"robot_name": LaunchConfiguration("robot_name")},
        {"profiles_path": LaunchConfiguration("profiles_path")},
    ]

    # ----------------------------
    # Required nodes for “drivable robot”
    # ----------------------------

    motor_driver = Node(
        package="robot_legion_teleop_python",
        executable="motor_driver_node",  # console_scripts entry point
        name="motor_driver_node",
        output="screen",
        parameters=common_params,
        # No explicit remaps needed because motor_driver_node computes its cmd_vel topic
        # from the selected profile’s cmd_vel_topic_template and robot_name.
    )

    heartbeat = Node(
        package="robot_legion_teleop_python",
        executable="heartbeat_node",
        name="heartbeat_node",
        output="screen",
        parameters=common_params,
    )

    # ----------------------------
    # Optional nodes (executor / camera)
    # ----------------------------

    unit_executor = Node(
        package="robot_legion_teleop_python",
        executable="unit_executor_action_server",
        name="unit_executor_action_server",
        output="screen",
        parameters=common_params,
        # NOTE:
        # If you later want different executors per drive type, keep this launch file
        # as the single entry point, and make the *node itself* branch based on profile.
    )

    usb_camera = Node(
        package="robot_legion_teleop_python",
        executable="usb_camera_node",
        name="usb_camera_node",
        output="screen",
        parameters=common_params,
        # If you later standardize topic naming (recommended), you may want to ensure
        # your camera publishes to something like:
        #   /<robot_name>/image_raw
        # If usb_camera_node uses a different topic internally, add remappings here.
        #
        # Example (only if needed):
        # remappings=[
        #     ("/image_raw", ["/", LaunchConfiguration("robot_name"), "/image_raw"]),
        # ],
    )

    # ----------------------------
    # Conditional inclusion without extra complexity
    # ----------------------------
    #
    # We keep this simple: launch always *declares* the nodes, and the Launch system
    # conditionally includes them based on run_* args.
    #
    # NOTE: Using "condition=" requires IfCondition. We import it here.
    from launch.conditions import IfCondition

    unit_executor.condition = IfCondition(LaunchConfiguration("run_executor"))
    usb_camera.condition = IfCondition(LaunchConfiguration("run_camera"))

    # ----------------------------
    # ADD FUTURE NODES HERE
    # ----------------------------
    #
    # Example pattern:
    #
    # some_node = Node(
    #   package="robot_legion_teleop_python",
    #   executable="new_node_entrypoint",
    #   name="new_node_name",
    #   output="screen",
    #   parameters=common_params + [{"some_param": "value"}],
    # )
    #
    # Then add it to the LaunchDescription([...]) list below.

    return LaunchDescription(
        [
            # Arguments must be declared first so ros2 launch can override them.
            robot_name_arg,
            profiles_path_arg,
            run_executor_arg,
            run_camera_arg,
            # Required nodes
            motor_driver,
            heartbeat,
            # Optional nodes
            unit_executor,
            usb_camera,
        ]
    )
