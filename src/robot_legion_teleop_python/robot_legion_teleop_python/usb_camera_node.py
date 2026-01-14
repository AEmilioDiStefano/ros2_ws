#!/usr/bin/env python3
"""
usb_camera_node.py

Generic USB camera publisher for real robots.
Publishes sensor_msgs/Image to:

  /<robot_name>/camera/image_raw
"""

import rclpy
import getpass
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        self.declare_parameter('robot_name', getpass.getuser())
        self.declare_parameter('device', 0)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        robot_name = self.get_parameter('robot_name').value
        device = self.get_parameter('device').value
        fps = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value

        topic = f'/{robot_name}/camera/image_raw'

        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device}')
            return

        self.timer = self.create_timer(1.0 / fps, self.capture_frame)

        self.get_logger().info(f'USB camera publishing on {topic}')

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
