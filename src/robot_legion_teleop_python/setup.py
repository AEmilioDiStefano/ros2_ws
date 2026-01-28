from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),

    # data_files controls what gets copied into:
    #   <install>/share/<package_name>/
    # This is EXACTLY where `ros2 launch <pkg> <file>` looks.
    data_files=[
        # Standard ROS 2 ament package index registration
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),

        # package.xml must be in the share directory
        ("share/" + package_name, ["package.xml"]),

        # ---- Config files ----
        # These are read by drive_profiles.py via ament_index (installed) or fallback (source tree).
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),

        # ---- Web UI assets (if used) ----
        # These are served/loaded by your FPV web tooling when you point at share/.
        (os.path.join("share", package_name, "webui"), glob("robot_legion_teleop_python/webui/*")),

        # ros2 launch searches in: <install>/share/<package_name>/launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') + glob('launch/*.py')),
    ],

    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems",
    maintainer_email="",
    description="Robot Legion teleop + motor driver + executor (proprietary)",
    license="LicenseRef-Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # When you add new ROS nodes later, add them here so `ros2 run` can find them.
            "teleop_legion_key = robot_legion_teleop_python.teleop_legion_key:main",
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",
            "heartbeat_node = robot_legion_teleop_python.heartbeat_node:main",
            "unit_executor_action_server = robot_legion_teleop_python.unit_executor_action_server:main",
        ],
    },
)
