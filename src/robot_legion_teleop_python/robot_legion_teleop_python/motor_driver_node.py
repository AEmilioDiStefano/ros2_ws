#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

import time
import getpass
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .drive_profiles import load_profile_registry, resolve_robot_profile

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class MotorOut:
    duty: float
    direction: int  # +1 or -1


class MotorBackendBase:
    def setup(self):
        raise NotImplementedError

    def write_diff(self, left_norm: float, right_norm: float):
        """left_norm/right_norm in [-1, +1]"""
        raise NotImplementedError

    def write_mecanum(self, fl: float, fr: float, rl: float, rr: float):
        """wheel norms in [-1, +1]"""
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def cleanup(self):
        raise NotImplementedError


class HBridge2ChBackend(MotorBackendBase):
    """
    Classic 2-channel H-bridge:
      EN_A + IN1/IN2 for left, EN_B + IN3/IN4 for right
    """
    def __init__(self, gpio: Dict, pwm_hz: int = 1000):
        self.EN_A = int(gpio["en_a"])
        self.IN1 = int(gpio["in1"])
        self.IN2 = int(gpio["in2"])
        self.IN3 = int(gpio["in3"])
        self.IN4 = int(gpio["in4"])
        self.EN_B = int(gpio["en_b"])
        self.pwm_hz = int(pwm_hz)

        self.left_pwm = None
        self.right_pwm = None

    def setup(self):
        if not GPIO_AVAILABLE:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.IN1, self.IN2, self.IN3, self.IN4, self.EN_A, self.EN_B]:
            GPIO.setup(pin, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.EN_A, self.pwm_hz)
        self.right_pwm = GPIO.PWM(self.EN_B, self.pwm_hz)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        self.stop()

    def _apply(self, left: MotorOut, right: MotorOut):
        if not GPIO_AVAILABLE:
            return

        # Left
        GPIO.output(self.IN1, GPIO.HIGH if left.direction > 0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if left.direction > 0 else GPIO.HIGH)

        # Right
        GPIO.output(self.IN3, GPIO.HIGH if right.direction > 0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if right.direction > 0 else GPIO.HIGH)

        self.left_pwm.ChangeDutyCycle(float(left.duty))
        self.right_pwm.ChangeDutyCycle(float(right.duty))

    def write_diff(self, left_norm: float, right_norm: float):
        # Implemented in node; backend just applies outputs
        raise NotImplementedError

    def write_mecanum(self, fl: float, fr: float, rl: float, rr: float):
        raise RuntimeError("HBridge2ChBackend cannot drive mecanum (needs 4 channels)")

    def stop(self):
        if not GPIO_AVAILABLE:
            return
        self._apply(MotorOut(0.0, +1), MotorOut(0.0, +1))

    def cleanup(self):
        if not GPIO_AVAILABLE:
            return
        try:
            self.left_pwm.stop()
            self.right_pwm.stop()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass


class TB6612_4ChBackend(MotorBackendBase):
    """
    Two TB6612 boards (or any equivalent arrangement) providing 4 independent motor channels.
    We represent each wheel as:
      PWM pin + IN1 + IN2
    """
    def __init__(self, gpio: Dict, pwm_hz: int = 1000):
        self.pwm_hz = int(pwm_hz)

        self.stby_pin = int(gpio.get("stby_pin", -1))

        self.map = {
            "fl": (int(gpio["fl_pwm"]), int(gpio["fl_in1"]), int(gpio["fl_in2"])),
            "fr": (int(gpio["fr_pwm"]), int(gpio["fr_in1"]), int(gpio["fr_in2"])),
            "rl": (int(gpio["rl_pwm"]), int(gpio["rl_in1"]), int(gpio["rl_in2"])),
            "rr": (int(gpio["rr_pwm"]), int(gpio["rr_in1"]), int(gpio["rr_in2"])),
        }

        self.pwms: Dict[str, any] = {}

    def setup(self):
        if not GPIO_AVAILABLE:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Optional STBY control
        if self.stby_pin >= 0:
            GPIO.setup(self.stby_pin, GPIO.OUT)
            GPIO.output(self.stby_pin, GPIO.HIGH)

        # Setup pins and PWM per wheel
        for key, (pwm_pin, in1, in2) in self.map.items():
            GPIO.setup(in1, GPIO.OUT)
            GPIO.setup(in2, GPIO.OUT)
            GPIO.setup(pwm_pin, GPIO.OUT)
            pwm = GPIO.PWM(pwm_pin, self.pwm_hz)
            pwm.start(0)
            self.pwms[key] = pwm

        self.stop()

    def _apply_wheel(self, key: str, duty: float, direction: int):
        if not GPIO_AVAILABLE:
            return
        pwm_pin, in1, in2 = self.map[key]
        GPIO.output(in1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(in2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        self.pwms[key].ChangeDutyCycle(float(duty))

    def write_diff(self, left_norm: float, right_norm: float):
        raise RuntimeError("TB6612_4ChBackend expects mecanum wheels, not diff-drive")

    def write_mecanum(self, fl: float, fr: float, rl: float, rr: float):
        # Implemented in node; backend just applies outputs
        raise NotImplementedError

    def stop(self):
        if not GPIO_AVAILABLE:
            return
        for k in ["fl", "fr", "rl", "rr"]:
            self._apply_wheel(k, 0.0, +1)

    def cleanup(self):
        if not GPIO_AVAILABLE:
            return
        try:
            for pwm in self.pwms.values():
                pwm.stop()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Robot identity
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()

        # Profile registry path override (optional)
        self.declare_parameter("profiles_path", "")

        # Load profile
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        registry = load_profile_registry(profiles_path)
        prof = resolve_robot_profile(registry, self.robot_name)

        self.profile_name = prof["profile_name"]
        self.drive_type = prof["drive_type"]          # diff_drive | mecanum
        self.hardware = prof["hardware"]              # hbridge_2ch | tb6612_4ch
        self.profile_params = prof["params"]
        self.profile_gpio = prof["gpio"]

        # Parameters (can be overridden by ROS params if desired)
        # Diff-drive
        self.declare_parameter("wheel_separation", float(self.profile_params.get("wheel_separation", 0.18)))
        self.declare_parameter("max_linear_speed", float(self.profile_params.get("max_linear_speed", 0.4)))
        self.declare_parameter("max_angular_speed", float(self.profile_params.get("max_angular_speed", 2.0)))

        # Mecanum
        self.declare_parameter("max_lateral_speed", float(self.profile_params.get("max_lateral_speed", self.profile_params.get("max_linear_speed", 0.5))))
        self.declare_parameter("k_omega", float(self.profile_params.get("k_omega", 0.22)))

        # PWM
        self.declare_parameter("max_pwm", float(self.profile_params.get("max_pwm", 100)))
        self.declare_parameter("pwm_hz", 1000)

        # cmd_vel topic
        default_cmd_topic = prof["cmd_vel_topic_template"].format(robot=self.robot_name)
        self.declare_parameter("cmd_vel_topic", default_cmd_topic)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value).strip() or default_cmd_topic

        # Watchdog
        self.timeout_sec = 0.5
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self._watchdog)

        # Backend selection
        self.backend: Optional[MotorBackendBase] = None

        if not GPIO_AVAILABLE:
            self.get_logger().warn("RPi.GPIO not available. Motors will NOT move.")
        else:
            self.backend = self._make_backend()
            self.backend.setup()

        # Subscriber
        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info(f"[{self.robot_name}] motor_driver_node profile={self.profile_name} drive_type={self.drive_type} hardware={self.hardware}")
        self.get_logger().info(f"[{self.robot_name}] Listening on {self.cmd_vel_topic}")

    def _make_backend(self) -> MotorBackendBase:
        pwm_hz = int(self.get_parameter("pwm_hz").value)

        if self.hardware == "hbridge_2ch":
            required = ["en_a", "in1", "in2", "in3", "in4", "en_b"]
            for k in required:
                if k not in self.profile_gpio:
                    raise ValueError(f"hbridge_2ch missing gpio key: {k}")
            return HBridge2ChBackend(self.profile_gpio, pwm_hz=pwm_hz)

        if self.hardware == "tb6612_4ch":
            required = ["fl_pwm", "fl_in1", "fl_in2", "fr_pwm", "fr_in1", "fr_in2", "rl_pwm", "rl_in1", "rl_in2", "rr_pwm", "rr_in1", "rr_in2"]
            for k in required:
                if k not in self.profile_gpio:
                    raise ValueError(f"tb6612_4ch missing gpio key: {k}")
            return TB6612_4ChBackend(self.profile_gpio, pwm_hz=pwm_hz)

        raise ValueError(f"Unknown hardware backend: {self.hardware}")

    def _watchdog(self):
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self._stop_motors()

    def _stop_motors(self):
        if self.backend is None:
            return
        try:
            self.backend.stop()
        except Exception:
            pass

    # ---------- PWM conversion helpers ----------

    def _norm_to_motorout(self, norm: float) -> MotorOut:
        max_pwm = float(self.get_parameter("max_pwm").value)
        norm = clamp(norm, -1.0, 1.0)
        direction = +1 if norm >= 0.0 else -1
        duty = abs(norm) * max_pwm
        return MotorOut(duty=duty, direction=direction)

    # ---------- Drive mixing ----------

    def _handle_diff_drive(self, msg: Twist):
        wheel_sep = float(self.get_parameter("wheel_separation").value)
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)

        v = clamp(float(msg.linear.x), -max_lin, +max_lin)
        w = clamp(float(msg.angular.z), -max_ang, +max_ang)

        # Tank-spin override (keeps your prior behavior)
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            spin_speed = 0.7 * max_lin
            direction = 1.0 if w > 0.0 else -1.0
            v_left = -direction * spin_speed
            v_right = +direction * spin_speed
        else:
            v_left = v - (w * wheel_sep / 2.0)
            v_right = v + (w * wheel_sep / 2.0)

        # Convert to normalized [-1,1] based on max_lin
        left_norm = clamp(v_left / max_lin if max_lin > 1e-6 else 0.0, -1.0, 1.0)
        right_norm = clamp(v_right / max_lin if max_lin > 1e-6 else 0.0, -1.0, 1.0)

        if not GPIO_AVAILABLE or self.backend is None:
            return

        if not isinstance(self.backend, HBridge2ChBackend):
            self.get_logger().error("Profile mismatch: diff_drive requires hbridge_2ch")
            return

        left_out = self._norm_to_motorout(left_norm)
        right_out = self._norm_to_motorout(right_norm)
        self.backend._apply(left_out, right_out)

    def _handle_mecanum(self, msg: Twist):
        """
        Mecanum mixing in "command space" (not physical m/s):
          fl = vx + vy + k*omega
          fr = vx - vy - k*omega
          rl = vx - vy + k*omega
          rr = vx + vy - k*omega
        Then normalize so max(|wheel|) <= 1.0, then map to PWM.
        """
        max_vx = float(self.get_parameter("max_linear_speed").value)
        max_vy = float(self.get_parameter("max_lateral_speed").value)
        max_w = float(self.get_parameter("max_angular_speed").value)
        k = float(self.get_parameter("k_omega").value)

        vx = clamp(float(msg.linear.x), -max_vx, +max_vx)
        vy = clamp(float(msg.linear.y), -max_vy, +max_vy)
        w = clamp(float(msg.angular.z), -max_w, +max_w)

        # Normalize inputs
        nx = (vx / max_vx) if max_vx > 1e-6 else 0.0
        ny = (vy / max_vy) if max_vy > 1e-6 else 0.0
        nw = (w / max_w) if max_w > 1e-6 else 0.0

        fl = nx + ny + (k * nw)
        fr = nx - ny - (k * nw)
        rl = nx - ny + (k * nw)
        rr = nx + ny - (k * nw)

        # Normalize to prevent saturation
        m = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))
        fl /= m
        fr /= m
        rl /= m
        rr /= m

        if not GPIO_AVAILABLE or self.backend is None:
            return

        if not isinstance(self.backend, TB6612_4ChBackend):
            self.get_logger().error("Profile mismatch: mecanum requires tb6612_4ch")
            return

        # Apply per-wheel
        for key, val in [("fl", fl), ("fr", fr), ("rl", rl), ("rr", rr)]:
            out = self._norm_to_motorout(val)
            self.backend._apply_wheel(key, out.duty, out.direction)

    # ---------- Subscriber callback ----------

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        if self.drive_type == "diff_drive":
            self._handle_diff_drive(msg)
            return

        if self.drive_type == "mecanum":
            self._handle_mecanum(msg)
            return

        self.get_logger().warn(f"Unknown drive_type '{self.drive_type}', stopping")
        self._stop_motors()

    # ---------- Shutdown ----------

    def destroy_node(self):
        self._stop_motors()
        if self.backend is not None:
            try:
                self.backend.cleanup()
            except Exception:
                pass
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
