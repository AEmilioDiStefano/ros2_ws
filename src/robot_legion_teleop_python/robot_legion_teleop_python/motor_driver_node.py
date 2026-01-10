#!/usr/bin/env python3
import time
import getpass

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
        super().__init__("motor_driver_node")

        # Robot naming
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()

        # Parameters
        self.declare_parameter("wheel_separation", 0.18)   # meters
        self.declare_parameter("max_linear_speed", 0.4)    # m/s
        self.declare_parameter("max_angular_speed", 2.0)   # rad/s
        self.declare_parameter("max_pwm", 100)             # percent

        # Topic this robot listens to
        self.declare_parameter("cmd_vel_topic", f"/{self.robot_name}/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value.strip() or f"/{self.robot_name}/cmd_vel"

        # GPIO pins (BCM)
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
            self.get_logger().warn("RPi.GPIO not available. Motors will NOT move.")

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog
        self.last_cmd_time = time.time()
        self.timeout_sec = 0.5
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(f"[{self.robot_name}] Motor driver listening on {self.cmd_vel_topic}")

    # --------------------------------------------------

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.IN1, self.IN2, self.IN3, self.IN4, self.EN_A, self.EN_B]:
            GPIO.setup(pin, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.EN_A, 1000)
        self.right_pwm = GPIO.PWM(self.EN_B, 1000)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

        self._set_motor_outputs(0.0, 0.0)
        self.get_logger().info("GPIO initialized for motor control")

    # --------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        wheel_sep = float(self.get_parameter("wheel_separation").value)
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)

        v = max(-max_lin, min(max_lin, msg.linear.x))
        w = max(-max_ang, min(max_ang, msg.angular.z))

        # ===============================
        # TANK-SPIN OVERRIDE
        # ===============================
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            spin_speed = 0.7 * max_lin
            direction = 1.0 if w > 0.0 else -1.0
            v_left = -direction * spin_speed
            v_right = +direction * spin_speed
            self._set_motor_outputs(v_left, v_right)
            return

        # ===============================
        # NORMAL DIFF-DRIVE MODE
        # ===============================
        v_left = v - (w * wheel_sep / 2.0)
        v_right = v + (w * wheel_sep / 2.0)
        self._set_motor_outputs(v_left, v_right)

    # --------------------------------------------------

    def _watchdog(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self._set_motor_outputs(0.0, 0.0)

    # --------------------------------------------------

    def _set_motor_outputs(self, v_left: float, v_right: float):
        if not GPIO_AVAILABLE:
            return

        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_pwm = float(self.get_parameter("max_pwm").value)

        def speed_to_pwm(v):
            ratio = max(-1.0, min(1.0, v / max_lin))
            direction = 1 if ratio >= 0 else -1
            duty = abs(ratio) * max_pwm
            return duty, direction

        left_duty, left_dir = speed_to_pwm(v_left)
        right_duty, right_dir = speed_to_pwm(v_right)

        # Left track
        GPIO.output(self.IN1, GPIO.HIGH if left_dir > 0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if left_dir > 0 else GPIO.HIGH)

        # Right track
        GPIO.output(self.IN3, GPIO.HIGH if right_dir > 0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if right_dir > 0 else GPIO.HIGH)

        self.left_pwm.ChangeDutyCycle(left_duty)
        self.right_pwm.ChangeDutyCycle(right_duty)

    # --------------------------------------------------

    def destroy_node(self):
        self._set_motor_outputs(0.0, 0.0)
        if GPIO_AVAILABLE:
            self.left_pwm.stop()
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


if __name__ == "__main__":
    main()
