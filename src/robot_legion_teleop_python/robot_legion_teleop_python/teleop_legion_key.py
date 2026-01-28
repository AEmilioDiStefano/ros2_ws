#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
Drive-type aware keyboard teleop.

Diff-drive:
  8/UP forward, 2/DOWN backward
  4/LEFT rotate left, 6/RIGHT rotate right
  7/9/1/3 circle turns (one track only)

Mecanum:
  8/UP forward, 2/DOWN backward
  4/LEFT strafe left, 6/RIGHT strafe right
  7/9/1/3 diagonals
  0 rotate left, . rotate right
"""

import re
import sys
import select
import termios
import tty
import time
import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .drive_profiles import load_profile_registry, resolve_robot_profile

CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_raw_mode():
    tty.setraw(sys.stdin.fileno())


def read_key_raw(timeout_s: float = 0.1) -> str:
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not rlist:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":
        key += sys.stdin.read(2)  # arrow sequence tail
    return key


@dataclass
class TeleopState:
    available_robots: List[str]
    active_robot: Optional[str]
    cmd_vel_topic: Optional[str]
    drive_type: Optional[str]
    linear_speed: float
    angular_speed: float
    allow_cmd_vel_switching: bool


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)

        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        self.profile_registry = load_profile_registry(profiles_path)

        # Publishers
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/teleop/active_robot", 10)
        self.active_drive_pub = self.create_publisher(String, "/teleop/active_drive_type", 10)

        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None
        self.current_drive_type: Optional[str] = None

        # Optional heartbeat for selected robot (best-effort)
        self.hb_sub = None
        self._last_hb_json: Optional[Dict] = None

        # Speeds
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        self.last_twist = Twist()
        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_lat_mult = 0.0
        self.last_yaw_mult = 0.0

        self.move_bindings: Dict[str, Tuple[str, int, int, int]] = {}
        self._rebuild_keymap_for("diff_drive")

        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)

        self._terminal_settings = None

    # -------- printing helper (avoids raw-mode newline drift) --------

    def _tprint(self, text: str = ""):
        if not sys.stdin.isatty() or self._terminal_settings is None:
            print(text)
            sys.stdout.flush()
            return

        try:
            restore_terminal_settings(self._terminal_settings)
        except Exception:
            print(text)
            sys.stdout.flush()
            return

        print(text if text else "")
        sys.stdout.flush()

        try:
            set_raw_mode()
        except Exception:
            pass

    # -------- discovery --------

    def _discover_candidate_cmd_vel_topics(self) -> List[str]:
        topics_and_types = self.get_topic_names_and_types()
        topics = [t for (t, _types) in topics_and_types]
        return [t for t in topics if CMD_VEL_RE.match(t)]

    def list_available_robots(self) -> List[str]:
        robots: Set[str] = set()
        for t in self._discover_candidate_cmd_vel_topics():
            subs = self.get_subscriptions_info_by_topic(t)
            if subs and len(subs) > 0:
                m = CMD_VEL_RE.match(t)
                if m:
                    robots.add(m.group(1))
        return sorted(robots)

    def _validate_robot_name(self, name: str) -> Tuple[bool, str]:
        name = name.strip()
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        return (subs is not None and len(subs) > 0), topic

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            if self.list_available_robots():
                return

    def render_robot_list(self, robots: List[str]) -> str:
        lines = []
        lines.append("")
        lines.append("======= AVAILABLE ROBOTS =======")
        if not robots:
            lines.append("")
            lines.append("  (none found)")
            lines.append("")
            lines.append("  - Enter 'r' to refresh")
            lines.append("  - Ensure robots are on same ROS_DOMAIN_ID and network")
            lines.append("  - Ensure motor_driver_node is running on the robot")
            lines.append("")
        else:
            lines.append("")
            for r in robots:
                lines.append(f"  - {r}")
            lines.append("")
        lines.append("================================")
        lines.append("")
        return "\n".join(lines)

    # -------- heartbeat (optional best-effort) --------

    def _heartbeat_cb(self, msg: String):
        try:
            self._last_hb_json = json.loads(msg.data)
        except Exception:
            self._last_hb_json = None

    def _attach_heartbeat_sub(self, robot_name: str):
        self._last_hb_json = None
        if self.hb_sub is not None:
            try:
                self.destroy_subscription(self.hb_sub)
            except Exception:
                pass
            self.hb_sub = None

        topic = f"/{robot_name}/heartbeat"
        self.hb_sub = self.create_subscription(String, topic, self._heartbeat_cb, 10)

        # give it a moment to arrive
        deadline = time.monotonic() + 0.35
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_hb_json is not None:
                break

    def _resolve_drive_type_for_robot(self, robot_name: str) -> str:
        # YAML baseline
        prof = resolve_robot_profile(self.profile_registry, robot_name)
        drive_type = str(prof.get("drive_type", "diff_drive")).strip()

        # heartbeat override (if present)
        if isinstance(self._last_hb_json, dict):
            dt = self._last_hb_json.get("drive_type", None)
            if isinstance(dt, str) and dt.strip():
                drive_type = dt.strip()

        return "mecanum" if drive_type == "mecanum" else "diff_drive"

    # -------- keymap --------

    def _rebuild_keymap_for(self, drive_type: str):
        dt = drive_type

        if dt == "mecanum":
            # mode, x_mult, y_mult, yaw_mult
            self.move_bindings = {
                # arrows
                "\x1b[A": ("twist", +1, 0, 0),
                "\x1b[B": ("twist", -1, 0, 0),
                "\x1b[D": ("twist", 0, +1, 0),
                "\x1b[C": ("twist", 0, -1, 0),

                # numpad style
                "8": ("twist", +1, 0, 0),
                "2": ("twist", -1, 0, 0),
                "4": ("twist", 0, +1, 0),
                "6": ("twist", 0, -1, 0),

                "7": ("twist", +1, +1, 0),
                "9": ("twist", +1, -1, 0),
                "1": ("twist", -1, +1, 0),
                "3": ("twist", -1, -1, 0),

                "0": ("twist", 0, 0, +1),
                ".": ("twist", 0, 0, -1),
            }
        else:
            # diff drive
            self.move_bindings = {
                "\x1b[A": ("twist", +1, 0, 0),   # forward
                "\x1b[B": ("twist", -1, 0, 0),   # backward
                "\x1b[D": ("twist", 0, 0, +1),   # rotate left
                "\x1b[C": ("twist", 0, 0, -1),   # rotate right

                "8": ("twist", +1, 0, 0),
                "2": ("twist", -1, 0, 0),
                "4": ("twist", 0, 0, +1),
                "6": ("twist", 0, 0, -1),

                # circle turns keep legacy behavior
                "7": ("circle", 0, 0, 0),
                "9": ("circle", 0, 0, 0),
                "1": ("circle", 0, 0, 0),
                "3": ("circle", 0, 0, 0),
            }

    # -------- selection + topic switching --------

    def _publish_active(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

        dt = String()
        dt.data = self.current_drive_type or ""
        self.active_drive_pub.publish(dt)

    def _apply_cmd_vel_topic(self, topic: str):
        # destroy prior publisher if any
        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass
            self.publisher_ = None

        self.cmd_vel_topic = topic
        self.current_robot_name = topic.split("/")[1] if topic.startswith("/") else topic

        # attach heartbeat (best-effort), then resolve drive type
        self._attach_heartbeat_sub(self.current_robot_name)
        self.current_drive_type = self._resolve_drive_type_for_robot(self.current_robot_name)

        self._rebuild_keymap_for(self.current_drive_type)

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._publish_active()

        self._tprint(self.render_header())

    def render_header(self) -> str:
        dt = self.current_drive_type or "unknown"
        r = self.current_robot_name or "(none)"
        return (
            "\n-------------------------------\n"
            "      ROBOT LEGION TELEOP      \n"
            "-------------------------------\n"
            f"ACTIVE ROBOT: {r}\n"
            f"drive_type:   {dt}\n"
            f"cmd_vel:      {self.cmd_vel_topic}\n"
            "STOP: [SPACE] or 's' or 5\n"
            "SPEED: i slow | o medium | p fast | w/+ up | e/- down | q,/ lin+ | r,* lin-\n"
            "Robot select: m\n"
            "CTRL-C to quit.\n"
            "-------------------------------\n"
        )

    def _prompt_until_valid_robot(self):
        first_pass = True
        while rclpy.ok():
            if first_pass:
                self._warmup_discovery(timeout_sec=1.0)
                first_pass = False

            robots = self.list_available_robots()
            self._tprint(self.render_robot_list(robots))

            restore_terminal_settings(self._terminal_settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name (or 'r' to refresh): ").strip()
            except Exception:
                set_raw_mode()
                continue
            set_raw_mode()

            if user_input.lower() == "r":
                self._warmup_discovery(timeout_sec=0.5)
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                self._tprint(f"[INVALID] '{user_input}' is not available (no subscriber on {topic}).")
                continue

            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                self._tprint("[ROBOT SWITCH] Disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # -------- motion helpers --------

    def _publish_one_track_circle(self, v_left: float, v_right: float):
        # This keeps your legacy “one track only” circle behavior
        # Convert wheel linear speeds into Twist (very rough proxy)
        # v = (vl + vr)/2 ; w = (vr - vl)/wheel_sep
        wheel_sep = 0.18  # keep same as motor node default
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / wheel_sep if abs(wheel_sep) > 1e-6 else 0.0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True
        self.publisher_.publish(twist)

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
            return
        self.publisher_.publish(self.last_twist)

    def _offline_watchdog(self):
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return
        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # offline
        try:
            self.publisher_.publish(Twist())
        except Exception:
            pass

        self.is_moving = False
        self.last_twist = Twist()
        self.last_lin_mult = 0.0
        self.last_lat_mult = 0.0
        self.last_yaw_mult = 0.0

        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass
        self.publisher_ = None
        self.cmd_vel_topic = None
        self.current_robot_name = None
        self.current_drive_type = None

        self._tprint("[OFFLINE] Robot went offline. Press 'm' to select another robot.")

    # -------- speed controls --------

    def _set_slow(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _set_medium(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _set_fast(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _speed_up(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _speed_down(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _lin_up(self):
        self.linear_speed *= self.speed_step
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    def _lin_down(self):
        self.linear_speed /= self.speed_step
        self._tprint(f"[SPEED] linear={self.linear_speed:.2f} angular={self.angular_speed:.2f}")

    # -------- main loop --------

    def run(self):
        self._terminal_settings = termios.tcgetattr(sys.stdin)
        set_raw_mode()

        self._tprint(self.render_header())
        self._prompt_until_valid_robot()

        try:
            while rclpy.ok():
                key = read_key_raw(timeout_s=0.1)
                if key == "":
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue
                if key == "\x03":
                    break

                # selection / switching
                if key == "m":
                    self._prompt_until_valid_robot()
                    continue

                # if no robot selected, ignore movement
                if self.publisher_ is None:
                    if key in ("i", "o", "p", "w", "+", "e", "-", "q", "/", "r", "*"):
                        # allow speed changes even while unselected
                        pass
                    else:
                        self._tprint("[INFO] No robot selected. Press 'm'.")
                        continue

                # stop
                if key in (" ", "s", "5"):
                    twist = Twist()
                    self.last_twist = twist
                    self.is_moving = False
                    if self.publisher_ is not None:
                        self.publisher_.publish(twist)
                    self._tprint("[STOP]")
                    continue

                # speed profiles
                if key == "i":
                    self._set_slow()
                    continue
                if key == "o":
                    self._set_medium()
                    continue
                if key == "p":
                    self._set_fast()
                    continue

                # speed scaling
                if key in ("w", "+"):
                    self._speed_up()
                    continue
                if key in ("e", "-"):
                    self._speed_down()
                    continue
                if key in ("q", "/"):
                    self._lin_up()
                    continue
                if key in ("r", "*"):
                    self._lin_down()
                    continue

                # movement
                if key in self.move_bindings and self.publisher_ is not None:
                    mode, x_mult, y_mult, yaw_mult = self.move_bindings[key]

                    if mode == "circle":
                        S = self.linear_speed
                        if key == "7":
                            self._publish_one_track_circle(v_left=0.0, v_right=+S)
                        elif key == "9":
                            self._publish_one_track_circle(v_left=+S, v_right=0.0)
                        elif key == "1":
                            self._publish_one_track_circle(v_left=0.0, v_right=-S)
                        elif key == "3":
                            self._publish_one_track_circle(v_left=-S, v_right=0.0)
                        continue

                    twist = Twist()
                    twist.linear.x = self.linear_speed * float(x_mult)

                    if (self.current_drive_type or "diff_drive") == "mecanum":
                        twist.linear.y = self.linear_speed * float(y_mult)
                        twist.angular.z = self.angular_speed * float(yaw_mult)
                    else:
                        twist.angular.z = self.angular_speed * float(yaw_mult)

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

        finally:
            try:
                restore_terminal_settings(self._terminal_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
