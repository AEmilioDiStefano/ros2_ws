#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Parameters (you can override via YAML if needed)
        self.declare_parameter('wheel_separation', 0.18)  # meters between tracks
        self.declare_parameter('wheel_radius', 0.033)     # radius of track sprocket
        self.declare_parameter('max_linear_speed', 0.4)   # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('max_pwm', 100)            # percent

        self.declare_parameter('left_cmd_topic', '/emiliobot/cmd_vel')
        self.declare_parameter('right_cmd_topic', '/my_robot/cmd_vel')  # unused now

        self.left_cmd_topic = self.get_parameter('left_cmd_topic').get_parameter_value().string_value

        # GPIO pins (BCM numbering)
        self.EN_A = 12
        self.IN1 = 17
        self.IN2 = 27
        self.IN3 = 22
        self.IN4 = 23
        self.EN_B = 13

        self.left_pwm = None
        self.right_pwm = None

        if GPIO_AVAILABLE:
            self._setup_gpio()
        else:
            self.get_logger().warn(
                'RPi.GPIO not available. This node will not control real motors. '
                'This is expected on your laptop but not on the Pi.'
            )

        # Subscribe to cmd_vel for emiliobot
        self.subscription = self.create_subscription(
            Twist,
            self.left_cmd_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog / timeout
        self.last_cmd_time = time.time()
        self.timeout_sec = 0.5
        self.timer = self.create_timer(0.1, self._watchdog)

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.IN1, self.IN2, self.IN3, self.IN4, self.EN_A, self.EN_B]:
            GPIO.setup(pin, GPIO.OUT)

        # PWM at 1 kHz on ENA and ENB
        self.left_pwm = GPIO.PWM(self.EN_A, 1000)
        self.right_pwm = GPIO.PWM(self.EN_B, 1000)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

        # Start with motors stopped
        self._set_motor_outputs(0.0, 0.0)

        self.get_logger().info('GPIO and PWM initialized for motor control.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        wheel_sep = self.get_parameter('wheel_separation').value
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value

        v = max(-max_lin, min(max_lin, msg.linear.x))
        w = max(-max_ang, min(max_ang, msg.angular.z))

        # Differential drive kinematics
        v_left = v - w * wheel_sep / 2.0
        v_right = v + w * wheel_sep / 2.0

        self._set_motor_outputs(v_left, v_right)

    def _watchdog(self):
        # Stop the robot if no cmd_vel received recently
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self._set_motor_outputs(0.0, 0.0)

    def _set_motor_outputs(self, v_left: float, v_right: float):
        if not GPIO_AVAILABLE:
            # Log occasionally could be too spammy; keep it silent here.
            return

        max_lin = self.get_parameter('max_linear_speed').value
        max_pwm = float(self.get_parameter('max_pwm').value)

        # Normalize speeds to [-1, 1]
        def speed_to_pwm(v):
            if max_lin <= 0.0:
                return 0.0, 0
            ratio = v / max_lin
            ratio = max(-1.0, min(1.0, ratio))
            direction = 1 if ratio >= 0.0 else -1
            duty = abs(ratio) * max_pwm
            return duty, direction

        left_duty, left_dir = speed_to_pwm(v_left)
        right_duty, right_dir = speed_to_pwm(v_right)

        # Left motor direction
        if left_dir > 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        elif left_dir < 0:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)

        # Right motor direction
        if right_dir > 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        elif right_dir < 0:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)

        # Set PWM duty cycles
        if self.left_pwm is not None:
            self.left_pwm.ChangeDutyCycle(left_duty)
        if self.right_pwm is not None:
            self.right_pwm.ChangeDutyCycle(right_duty)

    def destroy_node(self):
        self._set_motor_outputs(0.0, 0.0)
        if GPIO_AVAILABLE:
            if self.left_pwm is not None:
                self.left_pwm.stop()
            if self.right_pwm is not None:
                self.right_pwm.stop()
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

