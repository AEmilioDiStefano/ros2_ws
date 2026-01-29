# motor_driver_node.py
# Proprietary - Vitruvian Systems LLC
#
# PURPOSE
# -------
# A universal motor driver node that can support multiple drive types
# (diff_drive, mecanum, and future profiles) using the SAME ROS interface:
#
#   Subscribes: /<robot_name>/cmd_vel   (geometry_msgs/Twist)
#   Publishes:  (optional status logs)
#
# It reads a selected "drive_profile" from robot_profiles.yaml on the robot,
# and uses that profile to:
#   - determine drive type (diff vs mecanum)
#   - determine hardware type (h-bridge, tb6612, etc.)
#   - load GPIO pins and tuning parameters

from __future__ import annotations

import math
import time
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from robot_legion_teleop_python.drive_profiles import DriveProfileRegistry


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class MotorDriverNode(Node):
    """
    Universal motor driver node.

    NOTE TO ROS2-BEGINNERS:
    -----------------------
    This node subscribes to Twist messages and converts them to motor outputs.

    Twist convention:
      - linear.x  forward/back  (m/s)
      - linear.y  left/right     (m/s)  (often unused on diff-drive)
      - angular.z yaw rotation   (rad/s)
    """

    def __init__(self):
        super().__init__("motor_driver_node")

        self.registry = DriveProfileRegistry()

        # Determine robot_name from ROS namespace:
        # If node is started in namespace "/robot3", then this becomes "robot3"
        ns = self.get_namespace().strip("/")
        self.robot_name = ns if ns else "robot"

        # Load the selected profile from the robot-local YAML file
        self.profile = self.registry.selected_profile()
        self.drive_type = self.registry.selected_drive_type()
        self.hardware = self.registry.selected_hardware()
        self.profile_params: Dict[str, Any] = self.registry.selected_params()

        self.get_logger().info(
            f"[{self.robot_name}] motor_driver_node profile={self.profile} "
            f"drive_type={self.drive_type} hardware={self.hardware}"
        )

        # Subscribe to this robot's cmd_vel
        cmd_vel_topic = f"/{self.robot_name}/cmd_vel"
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self.get_logger().info(f"[{self.robot_name}] Listening on {cmd_vel_topic}")

        # TODO: initialize your hardware outputs here (GPIO / I2C / PWM drivers)
        # In your repository, you already have working low-level implementations;
        # the logic below focuses on the mixing + profile scaling.

    # ----------------------------
    # Core subscription callback
    # ----------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """
        Called whenever teleop (or orchestrator) publishes Twist to /<robot>/cmd_vel.
        We route it based on drive_type.
        """
        if self.drive_type == "mecanum":
            self._handle_mecanum(msg)
        else:
            # Safe default: diff drive
            self._handle_diff_drive(msg)

    # ----------------------------
    # Diff drive
    # ----------------------------

    def _handle_diff_drive(self, msg: Twist):
        """
        Convert Twist -> left/right normalized commands.
        """
        max_v = float(self.profile_params.get("max_linear_speed", 0.40))
        max_w = float(self.profile_params.get("max_angular_speed", 1.00))

        v = clamp(msg.linear.x / max_v, -1.0, 1.0) if max_v > 1e-6 else 0.0
        w = clamp(msg.angular.z / max_w, -1.0, 1.0) if max_w > 1e-6 else 0.0

        # Standard diff mixing:
        left = clamp(v + w, -1.0, 1.0)
        right = clamp(v - w, -1.0, 1.0)

        # Optional inversion (sometimes one side is mounted mirrored)
        inv_left = -1.0 if float(self.profile_params.get("invert_left", 1)) < 0 else 1.0
        inv_right = -1.0 if float(self.profile_params.get("invert_right", 1)) < 0 else 1.0
        left *= inv_left
        right *= inv_right

        self._send_diff_to_hardware(left, right)

    def _send_diff_to_hardware(self, left: float, right: float):
        """
        Replace this with your existing GPIO implementation.

        'left' and 'right' are normalized [-1..1].
        """
        # In your current codebase, this likely maps to:
        # - direction pins
        # - PWM pins
        # - enable pins
        #
        # Keep the hardware code proprietary and specific to your motor controller.
        pass

    # ----------------------------
    # Mecanum drive
    # ----------------------------

    def _handle_mecanum(self, msg: Twist):
        """
        Convert Twist -> four wheel normalized commands (fl, fr, rl, rr).

        Mecanum uses linear.x (forward), linear.y (strafe), angular.z (yaw).
        """
        max_v = float(self.profile_params.get("max_linear_speed", 0.50))
        max_w = float(self.profile_params.get("max_angular_speed", 1.20))

        vx = clamp(msg.linear.x / max_v, -1.0, 1.0) if max_v > 1e-6 else 0.0
        vy = clamp(msg.linear.y / max_v, -1.0, 1.0) if max_v > 1e-6 else 0.0
        omega = clamp(msg.angular.z / max_w, -1.0, 1.0) if max_w > 1e-6 else 0.0

        # k_omega tunes how aggressively angular.z contributes to wheel speed.
        # If rotation feels too weak, raise k_omega (e.g. 0.22 -> 0.4..0.8),
        # but do NOT fix rotation strain with k_omega; strain is usually wheel direction.
        k_omega = float(self.profile_params.get("k_omega", 0.22))

        def normalize(x: float) -> float:
            return clamp(x, -1.0, 1.0)

        # Standard mecanum mixing:
        # (Your mechanical mounting can swap signs; fix that with wheel_invert, below.)
        fl = normalize(vx + vy + k_omega * omega)
        fr = normalize(vx - vy - k_omega * omega)
        rl = normalize(vx - vy + k_omega * omega)
        rr = normalize(vx + vy - k_omega * omega)

        # Optional per-wheel inversion to match real wiring / gearbox direction.
        #
        # Why you want this:
        # - A very common mecanum symptom is: translation works fine, but pure rotation
        #   is slow and the motors "fight" / strain. That's almost always because one or
        #   more wheels are reversed (wiring swapped, motor mounted mirrored, etc.).
        #
        # Set this in robot_profiles.yaml under the selected profile params as:
        #   wheel_invert: {fl: 1, fr: -1, rl: 1, rr: -1}
        #
        # Keys: fl, fr, rl, rr. Values: 1 or -1 (anything else is treated as 1).
        wheel_invert = self.profile_params.get("wheel_invert", {}) if isinstance(self.profile_params, dict) else {}

        def inv(key: str) -> float:
            try:
                return -1.0 if float(wheel_invert.get(key, 1)) < 0 else 1.0
            except Exception:
                return 1.0

        fl *= inv("fl")
        fr *= inv("fr")
        rl *= inv("rl")
        rr *= inv("rr")

        self._send_mecanum_to_hardware(fl, fr, rl, rr)

    def _send_mecanum_to_hardware(self, fl: float, fr: float, rl: float, rr: float):
        """
        Replace this with your existing 4-channel motor controller implementation.

        Inputs are normalized [-1..1] per wheel.
        """
        pass


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
