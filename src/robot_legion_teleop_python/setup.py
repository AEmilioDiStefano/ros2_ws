from setuptools import setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems LLC",
    maintainer_email="emilio@vitruvian.systems",
    description="Keyboard teleop, motor driver, camera node, and FPV mux for Robot Legion robots.",
    license="CC-BY-NC-4.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Teleop (keyboard)
            "legion_teleop_key = robot_legion_teleop_python.teleop_legion_key:main",

            # Real robot motor driver (L298N + yellow motors)
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",

            # Camera node (robot-agnostic)
            "legion_camera_node = robot_legion_teleop_python.legion_camera_node:main",

            # Backward-compatible alias (old name)
            "emiliobot_camera = robot_legion_teleop_python.legion_camera_node:main",

            # Gazebo reset utility
            "reset_gz = robot_legion_teleop_python.reset_gz:main",

            # FPV camera multiplexer (robot-agnostic)
            "fpv_camera_mux = robot_legion_teleop_python.fpv_camera_mux:main",
        ],
    },
)
