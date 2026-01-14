from setuptools import setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # Web UI assets
        ("share/" + package_name + "/webui", [
            "robot_legion_teleop_python/webui/index.html",
            "robot_legion_teleop_python/webui/app.js",
            "robot_legion_teleop_python/webui/styles.css",
        ]),
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

            # Camera nodes
            "legion_camera_node = robot_legion_teleop_python.legion_camera_node:main",
            "usb_camera_node = robot_legion_teleop_python.usb_camera_node:main",

            # Optional mux (teleop-selected active robot -> /fpv_camera/image_raw)
            "fpv_camera_mux = robot_legion_teleop_python.fpv_camera_mux:main",

            # ROS-native browser control arbiter (used with rosbridge + web_video_server)
            "fpv_control_arbiter = robot_legion_teleop_python.fpv_control_arbiter:main",

            # Gazebo reset utility
            "reset_gz = robot_legion_teleop_python.reset_gz:main",
        ],
    },
)
