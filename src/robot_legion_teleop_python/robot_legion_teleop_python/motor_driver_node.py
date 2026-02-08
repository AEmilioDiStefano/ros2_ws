#!/usr/bin/env python3
import time
import getpass
import logging
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .drive_profiles import load_profile_registry, resolve_robot_profile
from .hardware_interface import HardwareInterface
from .drive_types import get_drive_type
from .audit_logger import AuditLogger

LOG = logging.getLogger("motor_driver_node")


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Robot naming
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()

        # Profile registry path (optional)
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        gpio_map = {}
        drive_params = {}
        hw_params = {}
        self.drive_type = "diff_drive"
        try:
            reg = load_profile_registry(profiles_path)
            prof = resolve_robot_profile(reg, self.robot_name)
            gpio_map = prof.get("gpio", {}) or {}
            drive_params = prof.get("drive_params", {}) or {}
            hw_params = prof.get("hardware_params", {}) or {}
            self.drive_type = str(prof.get("drive_type") or "diff_drive").lower()
            self.get_logger().info(f"[{self.robot_name}] Loaded GPIO map from profile: {list(gpio_map.keys())}")
        except Exception as ex:
            # Log the exception; drive_profiles.py should have already tried fallbacks.
            self.get_logger().warning(f"[{self.robot_name}] Failed to load profile registry: {ex}")
            self.get_logger().warning(f"[{self.robot_name}] Will run in mock hardware mode. To fix, pass profiles_path parameter or rebuild package.")

        # Parameters (YAML defaults, ROS params override)
        wheel_sep_default = (
            drive_params.get("wheel_separation_m")
            or drive_params.get("wheel_base_m")
            or 0.18
        )
        wheel_base_default = drive_params.get("wheel_base_m") or wheel_sep_default
        track_width_default = drive_params.get("track_width_m") or wheel_sep_default
        max_lin_default = drive_params.get("max_linear_mps") or 0.4
        max_ang_default = drive_params.get("max_angular_rps") or 2.0
        max_pwm_default = hw_params.get("max_pwm") or 100
        pwm_hz_default = hw_params.get("pwm_hz") or 1000
        pwm_ramp_ms_default = hw_params.get("pwm_ramp_ms") or 0
        pwm_deadband_default = hw_params.get("pwm_deadband_pct") or 0
        cmd_rate_hz_default = hw_params.get("cmd_rate_hz") or 0
        pwm_slew_default = hw_params.get("pwm_slew_pct_per_s") or 0
        timeout_default = drive_params.get("watchdog_timeout_s") or 0.5
        spin_mult_default = drive_params.get("spin_speed_mult") or 0.7
        stall_timeout_default = drive_params.get("stall_timeout_s") or 0.0
        stall_duty_default = drive_params.get("stall_duty_pct") or 0.0

        self.declare_parameter("wheel_separation", float(wheel_sep_default))   # meters
        self.declare_parameter("wheel_base", float(wheel_base_default))       # meters (mecanum)
        self.declare_parameter("track_width", float(track_width_default))     # meters (mecanum)
        self.declare_parameter("max_linear_speed", float(max_lin_default))    # m/s
        self.declare_parameter("max_angular_speed", float(max_ang_default))   # rad/s
        self.declare_parameter("max_pwm", int(max_pwm_default))               # percent
        self.declare_parameter("pwm_hz", int(pwm_hz_default))                 # Hz
        self.declare_parameter("pwm_ramp_ms", float(pwm_ramp_ms_default))
        self.declare_parameter("pwm_deadband_pct", float(pwm_deadband_default))
        self.declare_parameter("cmd_rate_hz", float(cmd_rate_hz_default))
        self.declare_parameter("pwm_slew_pct_per_s", float(pwm_slew_default))
        self.declare_parameter("watchdog_timeout_s", float(timeout_default))
        self.declare_parameter("spin_speed_mult", float(spin_mult_default))
        self.declare_parameter("stall_timeout_s", float(stall_timeout_default))
        self.declare_parameter("stall_duty_pct", float(stall_duty_default))

        # Topic this robot listens to
        self.declare_parameter("cmd_vel_topic", f"/{self.robot_name}/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value.strip() or f"/{self.robot_name}/cmd_vel"

        # Normalize common profile naming and initialize hardware interface.
        # For mecanum/omni we keep 4-channel names intact (fl/fr/rl/rr).
        gpio_map = self._normalize_gpio_map(gpio_map, drive_type=self.drive_type)
        # Pass PWM frequency through the gpio_map for HardwareInterface
        gpio_map["pwm_hz"] = int(self.get_parameter("pwm_hz").value)
        gpio_map["pwm_ramp_ms"] = float(self.get_parameter("pwm_ramp_ms").value)
        gpio_map["pwm_slew_pct_per_s"] = float(self.get_parameter("pwm_slew_pct_per_s").value)
        self.hw = HardwareInterface(gpio_map)
        self.drive = get_drive_type(self.drive_type)

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog
        self.last_cmd_time = time.time()
        self.last_output_time = 0.0
        self._last_commanded = None  # (left_duty, left_dir, right_duty, right_dir)
        # Stall protection: time-based limiter to avoid grinding motors under load.
        self._stall_start_time = None
        self._stall_active = False
        self._stall_timeout_s = float(self.get_parameter("stall_timeout_s").value)
        self._stall_duty_pct = float(self.get_parameter("stall_duty_pct").value)
        self.cmd_rate_hz = float(self.get_parameter("cmd_rate_hz").value)
        self.min_cmd_period = (1.0 / self.cmd_rate_hz) if self.cmd_rate_hz > 0 else 0.0
        self.timeout_sec = float(self.get_parameter("watchdog_timeout_s").value)
        self.create_timer(0.1, self._watchdog)

        # Audit logging for DIU compliance
        audit_log_path = os.environ.get("ROBOT_AUDIT_LOG_PATH") or f"/tmp/robot_{self.robot_name}_audit.jsonl"
        self.audit = AuditLogger(self, "motor_driver", audit_log_path)

        self.get_logger().info(f"[{self.robot_name}] Motor driver listening on {self.cmd_vel_topic}")
        self.get_logger().info(f"[{self.robot_name}] Audit log: {audit_log_path}")

    # --------------------------------------------------

    def _normalize_gpio_map(self, gpio_map: dict, drive_type: str = "diff_drive") -> dict:
        """Normalize different hardware profile key names into a common
        mapping consumed by HardwareInterface (en_left,in1_left,in2_left,en_right,...).

        This keeps motor mixing logic independent of exact profile naming.
        """
        if not gpio_map:
            return {}

        # If mecanum/omni, keep 4-channel map intact
        if str(drive_type).lower() in ("mecanum", "omni", "omnidirectional", "mecanum_drive"):
            return gpio_map

        # If profile already uses expected keys, return as-is
        expected = ("en_left", "in1_left", "in2_left", "en_right", "in1_right", "in2_right")
        if all(k in gpio_map for k in ("en_left", "in1_left", "in2_left")) or all(k in gpio_map for k in ("fl_pwm", "fr_pwm")):
            # Map TB6612 style (fl/fr -> left/right) to en_left/en_right
            mapped = {}
            if "en_left" in gpio_map:
                mapped.update(gpio_map)
                return mapped

        mapped = {}
        # TB6612 dual naming -> map front-left/front-right to left/right
        if "fl_pwm" in gpio_map and "fr_pwm" in gpio_map:
            mapped["en_left"] = gpio_map.get("fl_pwm")
            mapped["in1_left"] = gpio_map.get("fl_in1")
            mapped["in2_left"] = gpio_map.get("fl_in2")

            mapped["en_right"] = gpio_map.get("fr_pwm")
            mapped["in1_right"] = gpio_map.get("fr_in1")
            mapped["in2_right"] = gpio_map.get("fr_in2")
            return mapped

        # L298N_diff (hbridge_2ch) style mapping
        if "en_left" in gpio_map or "in1_left" in gpio_map:
            mapped.update(gpio_map)
            return mapped

        # older-style keys (en_left/en_right names may vary)
        # try common alternative keys
        if "ena" in gpio_map and "enb" in gpio_map:
            mapped["en_left"] = gpio_map.get("ena")
            mapped["en_right"] = gpio_map.get("enb")
            mapped["in1_left"] = gpio_map.get("in1_left")
            mapped["in2_left"] = gpio_map.get("in2_left")
            mapped["in1_right"] = gpio_map.get("in1_right")
            mapped["in2_right"] = gpio_map.get("in2_right")
            return mapped

        # fallback: return original map, HardwareInterface will mock if keys absent
        return gpio_map

    # --------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        start_time = time.time()

        wheel_sep = float(self.get_parameter("wheel_separation").value)
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)

        # Log motor command for audit trail
        self.audit.log_command(
            robot=self.robot_name,
            source="cmd_vel",
            command_id="twist",
            parameters={"linear_x": float(msg.linear.x), "angular_z": float(msg.angular.z)},
            status="received",
            duration_s=time.time() - start_time,
        )

        # Centralized drive-type mixing
        params = {
            "wheel_separation": wheel_sep,
            "wheel_base": float(self.get_parameter("wheel_base").value),
            "track_width": float(self.get_parameter("track_width").value),
            "max_linear_speed": max_lin,
            "max_angular_speed": max_ang,
            "spin_speed_mult": float(self.get_parameter("spin_speed_mult").value),
        }
        cmd = self.drive.mix(msg, params)
        if cmd.left is not None and cmd.right is not None:
            # Rotate commands bypass ramp/deadband for immediate response.
            is_rotate = abs(cmd.left) > 1e-6 and abs(cmd.right) > 1e-6 and (cmd.left * cmd.right) < 0
            self._set_motor_outputs(cmd.left, cmd.right, bypass_safety=is_rotate)
        else:
            self._set_mecanum_outputs(cmd)

    # --------------------------------------------------

    def _watchdog(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self._set_motor_outputs(0.0, 0.0, force=True)

    # --------------------------------------------------

    def _set_motor_outputs(self, v_left: float, v_right: float, force: bool = False, bypass_safety: bool = False):
        # Convert speeds to duty+direction and delegate to hardware interface
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_pwm = float(self.get_parameter("max_pwm").value)
        deadband = float(self.get_parameter("pwm_deadband_pct").value)
        if bypass_safety:
            deadband = 0.0
        deadband = max(0.0, min(deadband, max_pwm))

        def speed_to_pwm(v):
            ratio = max(-1.0, min(1.0, v / max_lin))
            direction = 1 if ratio >= 0 else -1
            duty = abs(ratio) * max_pwm
            if duty > 0.0 and duty < deadband:
                duty = 0.0
            return duty, direction

        left_duty, left_dir = speed_to_pwm(v_left)
        right_duty, right_dir = speed_to_pwm(v_right)

        now = time.time()
        if not force and self.min_cmd_period > 0 and (now - self.last_output_time) < self.min_cmd_period:
            # If a NEW command arrives quickly, we still apply it immediately
            # so the most recent command always wins (no ambiguity).
            last = self._last_commanded
            changed = (
                last is None
                or abs(left_duty - last[0]) > 0.5
                or abs(right_duty - last[2]) > 0.5
                or left_dir != last[1]
                or right_dir != last[3]
            )
            if not changed:
                return

        # --- Stall protection (time-based)
        # If duty is high for too long, cut output to avoid grinding motors or brownouts.
        if self._stall_timeout_s > 0 and self._stall_duty_pct > 0:
            high_duty = max(left_duty, right_duty) >= self._stall_duty_pct
            if high_duty:
                if self._stall_start_time is None:
                    self._stall_start_time = now
                elif (now - self._stall_start_time) >= self._stall_timeout_s:
                    self._stall_active = True
            else:
                # Reset once commanded duty drops below threshold
                self._stall_start_time = None
                self._stall_active = False

            if self._stall_active and not force:
                left_duty, right_duty = 0.0, 0.0

        # Use the hardware abstraction layer to actually set pins/PWM
        try:
            self.hw.set_motor(left_duty, left_dir, right_duty, right_dir, bypass_ramp=bypass_safety)
            self.last_output_time = now
            self._last_commanded = (left_duty, left_dir, right_duty, right_dir)
        except Exception as e:
            LOG.warning("Failed to set motor outputs: %s", e)

    def _set_mecanum_outputs(self, cmd):
        """Compute mecanum wheel commands and dispatch to hardware interface.

        This keeps drive-type logic contained in the motor driver, while the
        HardwareInterface handles the actual pin/PWM outputs.
        """
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_pwm = float(self.get_parameter("max_pwm").value)
        deadband = float(self.get_parameter("pwm_deadband_pct").value)
        deadband = max(0.0, min(deadband, max_pwm))

        def speed_to_pwm(v):
            ratio = max(-1.0, min(1.0, v / max_lin))
            direction = 1 if ratio >= 0 else -1
            duty = abs(ratio) * max_pwm
            if duty > 0.0 and duty < deadband:
                duty = 0.0
            return duty, direction

        fl_d, fl_dir = speed_to_pwm(cmd.fl)
        fr_d, fr_dir = speed_to_pwm(cmd.fr)
        rl_d, rl_dir = speed_to_pwm(cmd.rl)
        rr_d, rr_dir = speed_to_pwm(cmd.rr)

        try:
            self.hw.set_mecanum(fl_d, fl_dir, fr_d, fr_dir, rl_d, rl_dir, rr_d, rr_dir)
        except Exception as e:
            LOG.warning("Failed to set mecanum outputs: %s", e)

    # --------------------------------------------------

    def destroy_node(self):
        try:
            # Soft-stop on shutdown (honors ramp/slew for gentle hardware stop).
            self._set_motor_outputs(0.0, 0.0, force=True)
        except Exception:
            pass
        try:
            if hasattr(self, "hw"):
                self.hw.stop()
        except Exception:
            pass
        try:
            if hasattr(self, "audit"):
                self.audit.close()
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
