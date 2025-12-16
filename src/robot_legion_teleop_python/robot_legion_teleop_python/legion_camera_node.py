#!/usr/bin/env python3
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LegionCameraNode(Node):
    def __init__(self):
        super().__init__('legion_camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 20.0)

        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/emiliobot/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {camera_index}')
        else:
            self.get_logger().info(f'Opened camera at index {camera_index}')

        period = 1.0 / max(1.0, self.frame_rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        # BGR (OpenCV) -> ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
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


if __name__ == '__main__':
    main()
