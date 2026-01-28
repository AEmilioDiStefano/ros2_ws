#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

import getpass
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .drive_profiles import load_profile_registry, resolve_robot_profile


class HeartbeatNode(Node):
    def __init__(self):
        super().__init__("heartbeat_node")

        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("profiles_path", "")

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        reg = load_profile_registry(profiles_path)
        prof = resolve_robot_profile(reg, self.robot)

        self.drive_type = prof["drive_type"]
        self.hardware = prof["hardware"]
        self.profile_name = prof["profile_name"]

        self.topic = f"/{self.robot}/heartbeat"
        self.pub = self.create_publisher(String, self.topic, 10)
        self.create_timer(0.5, self._tick)  # 2 Hz

        self.get_logger().info(f"[{self.robot}] Heartbeat publishing on {self.topic}")
        self.get_logger().info(f"[{self.robot}] drive_type={self.drive_type} hardware={self.hardware} profile={self.profile_name}")

    def _tick(self):
        payload = {
            "robot": self.robot,
            "t": time.time(),
            "drive_type": self.drive_type,
            "hardware": self.hardware,
            "profile": self.profile_name,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
