#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
setup.py (robot_legion_teleop_python)

This file tells ROS 2 / colcon how to install your Python package.

Key concept for launch files:
- `ros2 launch <package> <file.launch.py>` looks for launch files in:
    <install_prefix>/share/<package>/launch/

So we MUST copy your repository's ./launch/*.py files into:
    share/<package>/launch/

That's what the `data_files` entry below does.
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament index + package manifest
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # Install configuration YAML (robot_profiles.yaml) into share/<pkg>/config/
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),

        # ✅ Install launch files into share/<pkg>/launch/
        # IMPORTANT: these globs are relative to the package ROOT:
        #   src/robot_legion_teleop_python/
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # (optional but harmless if you ever name files *.launch.py explicitly)
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems",
    maintainer_email="devnull@example.com",
    description="Robot Legion teleop + robot-side drivers (proprietary).",
    license="LicenseRef-Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # These must match the node executables you run via `ros2 run ...`
            "teleop_legion_key = robot_legion_teleop_python.teleop_legion_key:main",
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",
            "heartbeat_node = robot_legion_teleop_python.heartbeat_node:main",
            "unit_executor_action_server = robot_legion_teleop_python.unit_executor_action_server:main",
            "fpv_control_arbiter = robot_legion_teleop_python.fpv_control_arbiter:main",
        ],
    },
)
