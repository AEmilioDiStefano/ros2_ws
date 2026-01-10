#!/usr/bin/env python3
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FpvCameraMux(Node):
    def __init__(self):
        super().__init__("fpv_camera_mux")

        # Active robot name (teleop publishes this)
        self.active_robot: Optional[str] = None

        # Active camera subscription
        self.camera_sub: Optional[rclpy.subscription.Subscription] = None
        self.current_camera_topic: Optional[str] = None

        # Latest frame
        self.latest_frame: Optional[Image] = None

        # Output publisher
        self.output_pub = self.create_publisher(Image, "/fpv_camera/image_raw", 10)

        # Subscribe to teleop robot selection
        self.active_robot_sub = self.create_subscription(
            String,
            "/teleop/active_robot",
            self.active_robot_callback,
            10,
        )

        # Timer to republish active frame
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("FPV camera mux initialized (robot-agnostic).")

    def _set_camera_subscription(self, robot_name: str):
        camera_topic = f"/{robot_name}/camera/image_raw"

        if self.current_camera_topic == camera_topic:
            return

        # Destroy prior subscription (if any)
        if self.camera_sub is not None:
            try:
                self.destroy_subscription(self.camera_sub)
            except Exception:
                pass
            self.camera_sub = None

        self.latest_frame = None
        self.current_camera_topic = camera_topic

        self.camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10,
        )

        self.get_logger().info(f"FPV now listening to camera topic: {camera_topic}")

    def active_robot_callback(self, msg: String):
        robot = msg.data.strip()
        if not robot:
            return
        self.active_robot = robot
        self._set_camera_subscription(robot)

    def camera_callback(self, msg: Image):
        self.latest_frame = msg

    def timer_callback(self):
        if self.latest_frame is not None:
            self.output_pub.publish(self.latest_frame)


def main(args=None):
    rclpy.init(args=args)
    node = FpvCameraMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
