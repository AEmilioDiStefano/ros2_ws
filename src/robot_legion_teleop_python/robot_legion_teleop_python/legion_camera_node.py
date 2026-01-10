#!/usr/bin/env python3
import getpass
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LegionCameraNode(Node):
    def __init__(self):
        super().__init__("legion_camera_node")

        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_rate", 20.0)

        camera_index = self.get_parameter("camera_index").get_parameter_value().integer_value
        frame_rate = float(self.get_parameter("frame_rate").value)

        self.bridge = CvBridge()

        self.image_topic = f"/{self.robot_name}/camera/image_raw"
        self.publisher = self.create_publisher(Image, self.image_topic, 10)

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"[{self.robot_name}] Failed to open camera at index {camera_index}")
        else:
            self.get_logger().info(f"[{self.robot_name}] Opened camera at index {camera_index}")
            self.get_logger().info(f"[{self.robot_name}] Publishing images on {self.image_topic}")

        period = 1.0 / max(1.0, frame_rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(f"[{self.robot_name}] Failed to read frame from camera.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegionCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
