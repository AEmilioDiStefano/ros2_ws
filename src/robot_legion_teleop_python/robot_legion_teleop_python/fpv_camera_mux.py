#!/usr/bin/env python3
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FpvCameraMux(Node):
    def __init__(self):
        super().__init__('fpv_camera_mux')

        # Known robots / camera topics (expand later as needed)
        self.robot_camera_topics: Dict[str, str] = {
            'emiliobot': '/emiliobot/camera/image_raw',
            'my_robot': '/my_robot/camera/image_raw',
        }

        # Subscribers for each robot camera
        self.camera_subscribers: Dict[str, rclpy.subscription.Subscription] = {}

        # Latest frames
        self.latest_frames: Dict[str, Image] = {}

        # Active robot
        self.active_robot = 'emiliobot'

        # Output publisher
        self.output_pub = self.create_publisher(Image, '/fpv_camera/image_raw', 10)

        # Subscribe to active robot topic
        self.active_robot_sub = self.create_subscription(
            String,
            '/teleop/active_robot',
            self.active_robot_callback,
            10
        )

        # Camera subscriptions
        for robot_name, topic in self.robot_camera_topics.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, name=robot_name: self.camera_callback(msg, name),
                10
            )
            self.camera_subscribers[robot_name] = sub

        # Timer to republish active frame
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info('FPV camera mux initialized.')

    def active_robot_callback(self, msg: String):
        if msg.data in self.robot_camera_topics:
            self.active_robot = msg.data
            self.get_logger().info(f'FPV active robot set to: {self.active_robot}')
        else:
            self.get_logger().warn(f'Unknown robot name in active_robot message: {msg.data}')

    def camera_callback(self, msg: Image, robot_name: str):
        self.latest_frames[robot_name] = msg

    def timer_callback(self):
        # Republish the latest frame for the active robot
        frame = self.latest_frames.get(self.active_robot, None)
        if frame is not None:
            self.output_pub.publish(frame)


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


if __name__ == '__main__':
    main()
