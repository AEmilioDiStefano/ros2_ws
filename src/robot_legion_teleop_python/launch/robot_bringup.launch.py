from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import re
import getpass
from ament_index_python.packages import get_package_share_directory


def _sanitize_ros_name(name: str) -> str:
    # ROS name/topic-safe: alphanumerics + underscore only
    s = re.sub(r"[^A-Za-z0-9_]", "_", (name or "").strip())
    s = s.strip("_")
    return s or "robot1"


def _default_robot_name_from_username() -> str:
    # Linux username (like you requested)
    try:
        u = getpass.getuser()
        if u:
            return _sanitize_ros_name(u)
    except Exception:
        pass

    # Fallback: env var USER
    u = os.environ.get("USER", "").strip()
    if u:
        return _sanitize_ros_name(u)

    return "robot1"


def _make_nodes(context, *args, **kwargs):
    robot_name = _default_robot_name_from_username()

    video_device = LaunchConfiguration("video_device").perform(context)
    use_camera = LaunchConfiguration("use_camera").perform(context).lower() in ("1", "true", "yes", "on")
    drive_type = LaunchConfiguration("drive_type").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)
    profiles_path = LaunchConfiguration("profiles_path").perform(context) or ""

    nodes = []

    def _wrap_and_append_log(msg: str, width: int = 30):
        # split message into narrow chunks while respecting word boundaries
        # for terminal-friendly output on narrow terminals (e.g., 30-char windows)
        lines = []
        current_line = ""
        for word in msg.split(" "):
            if not current_line:
                current_line = word
            elif len(current_line) + 1 + len(word) <= width:
                current_line += " " + word
            else:
                if current_line:
                    lines.append(current_line)
                current_line = word
        if current_line:
            lines.append(current_line)
        for line in lines:
            nodes.append(LogInfo(msg=line))

    def _package_exists(pkg_name: str) -> bool:
        """Check if ROS 2 package is installed using ament index."""
        try:
            get_package_share_directory(pkg_name)
            return True
        except Exception:
            return False

    def _try_node(**node_kwargs):
        pkg = node_kwargs.get("package")
        name = node_kwargs.get("name") or node_kwargs.get("executable")
        
        # Check if package exists before creating Node
        if pkg and not _package_exists(pkg):
            _wrap_and_append_log(f"[SKIP] {name}: pkg '{pkg}'")
            _wrap_and_append_log(f"       not found")
            return
        
        try:
            n = Node(**node_kwargs)
            nodes.append(n)
            _wrap_and_append_log(f"[OK] {name} ready")
        except Exception as e:
            _wrap_and_append_log(f"[FAIL] {name}: {e}")
    # Motor driver under /<robot_name>/...
    _try_node(
        package="robot_legion_teleop_python",
        executable="motor_driver_node",
        name="motor_driver_node",
        namespace=robot_name,
        output="screen",
        parameters=[
            {"robot_name": robot_name},
            {"cmd_vel_topic": f"/{robot_name}/cmd_vel"},
            {"profiles_path": profiles_path},
            {"drive_type": drive_type},
            {"hardware": hardware},
        ],
    )

    # Heartbeat publisher under /<robot_name>/heartbeat so teleop/control can discover
    _try_node(
        package="robot_legion_teleop_python",
        executable="heartbeat_node",
        name="heartbeat_node",
        namespace=robot_name,
        output="screen",
        parameters=[
            {"robot_name": robot_name},
            {"profiles_path": profiles_path},
            {"drive_type": drive_type},
            {"hardware": hardware},
        ],
    )

    # Camera publisher under /<robot_name>/image_raw (USB webcam)
    if use_camera:
        _try_node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            namespace=robot_name,
            output="screen",
            parameters=[
                {"video_device": video_device},
            ],
            # In the namespace, image_raw becomes /<robot_name>/image_raw automatically,
            # but we keep this explicit for clarity and future node swaps.
            remappings=[
                ("image_raw", "image_raw"),
            ],
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument(
                "drive_type",
                default_value="diff_drive",
                description="Robot drive type: 'diff_drive' for differential drive, 'mecanum' for omnidirectional",
            ),
            DeclareLaunchArgument(
                "hardware",
                default_value="L298N_diff",
                description="Motor driver hardware profile: 'L298N_diff', 'dual_tb6612_mecanum', etc.",
            ),
            DeclareLaunchArgument(
                "profiles_path",
                default_value="",
                description="Optional path to custom robot_profiles.yaml file (empty = use installed default)",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
