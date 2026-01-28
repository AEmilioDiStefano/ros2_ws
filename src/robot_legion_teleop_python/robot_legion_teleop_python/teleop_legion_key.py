#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
teleop_legion_key.py (Drive-type aware + “old-style” operator-friendly terminal UX)

GOAL OF THIS FILE
-----------------
This node runs on the PILOT laptop and publishes geometry_msgs/Twist to a selected robot.

It supports a heterogeneous fleet:
- Differential drive robots (diff_drive)
- Omnidirectional mecanum robots (mecanum)

A robot is considered "available" if there is at least one subscriber on:
  /<robot_name>/cmd_vel
That typically means the robot is running motor_driver_node.py on that robot.

Drive type identification (what tells teleop “diff vs mecanum”?)
---------------------------------------------------------------
We resolve drive type in this priority order:

1) Heartbeat (best effort, only for the currently selected robot)
   If the robot runs heartbeat_node.py, it publishes JSON on:
     /<robot>/heartbeat
   That JSON includes drive_type, so the robot can “self-identify”.

2) YAML profile registry (config/robot_profiles.yaml)
   If heartbeat is not present (or not yet received), we fall back to the
   robot_profiles.yaml registry and resolve_robot_profile().

3) Final fallback = diff_drive
   If neither heartbeat nor YAML has useful info, we assume diff_drive
   for safety (prevents accidental strafe commands on diff robots).

IMPORTANT:
- Teleop does NOT need GPIO mappings (those only matter on the robot).
- Teleop only needs drive_type to interpret keys correctly.
- motor_driver_node.py on the robot uses the GPIO mapping.

Key semantics (per drive type)
------------------------------
DIFF-DRIVE:
  8/UP forward, 2/DOWN backward
  4/LEFT rotate left, 6/RIGHT rotate right
  7/9/1/3 circle turns (one track only; legacy behavior preserved)

MECANUM:
  8/UP forward, 2/DOWN backward
  4/LEFT strafe left, 6/RIGHT strafe right
  7/9/1/3 diagonals
  0 rotate left, . rotate right

Terminal reliability note (why we have _tprint)
----------------------------------------------
This program reads keys in RAW mode for instant keypress capture.
Printing while RAW is active can cause newline behavior to “drift”.
We fix that by printing in cooked mode (restore terminal settings),
printing, then returning to raw mode.

This file is intentionally heavily commented for newer ROS 2 users.
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

# Used to load + resolve robot drive profiles from config/robot_profiles.yaml
# (This is what teleop uses when heartbeat is not available.)
from .drive_profiles import load_profile_registry, resolve_robot_profile


# We detect robots by the existence of /<robot>/cmd_vel topics.
CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")


# ---------------------------
# Terminal helpers
# ---------------------------

def restore_terminal_settings(settings):
    """Restore terminal from raw mode to cooked mode."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_raw_mode():
    """Put terminal in raw mode so we can read single keypresses immediately."""
    tty.setraw(sys.stdin.fileno())


def read_key_raw(timeout_s: float = 0.1) -> str:
    """
    Read a single keypress (with arrow-key escape sequence support).
    Assumes terminal is already in raw mode.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not rlist:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":  # start of escape sequence
        # Typical arrow keys arrive as ESC [ A/B/C/D (3 chars total including ESC)
        key += sys.stdin.read(2)
    return key


# ---------------------------
# Teleop state (useful later for web UI)
# ---------------------------

@dataclass
class TeleopState:
    available_robots: List[str]
    active_robot: Optional[str]
    cmd_vel_topic: Optional[str]
    drive_type: Optional[str]
    linear_speed: float
    angular_speed: float
    allow_cmd_vel_switching: bool
    wheel_separation: float


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        # ---- Parameters (can be overridden via `ros2 run ... --ros-args -p ...:=...`) ----

        # If false: once you pick a robot, pressing 'm' will NOT switch.
        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)

        # Used only for diff-drive “circle turn” math (must match motor_driver_node defaults)
        self.declare_parameter("wheel_separation", 0.18)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        # Optional override path for robot_profiles.yaml
        # If empty: drive_profiles.py finds it in the installed share directory, or source fallback.
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        self.profile_registry = load_profile_registry(profiles_path)

        # ---- Publishers (these are “meta” topics used by other nodes like FPV mux) ----
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/teleop/active_robot", 10)
        self.active_drive_pub = self.create_publisher(String, "/teleop/active_drive_type", 10)

        # ---- Command publisher (created AFTER you select a robot) ----
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None
        self.current_drive_type: Optional[str] = None

        # ---- Heartbeat subscription (best effort) ----
        # If the selected robot is publishing /<robot>/heartbeat, we will parse it and
        # allow it to override YAML drive_type. This is how robots “self-identify”.
        self.hb_sub = None
        self._last_hb_json: Optional[Dict] = None

        # ---- Speed model ----
        # linear_speed affects Twist.linear.x (and Twist.linear.y for mecanum)
        # angular_speed affects Twist.angular.z
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        # “Profiles” are just preset speed scalars for pilots.
        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # ---- Republish state ----
        # We republish last twist periodically so movement continues while key is held.
        self.last_twist = Twist()
        self.is_moving = False

        # Last motion multipliers (useful when we want to re-emit based on current speed settings)
        self.last_lin_mult = 0.0
        self.last_lat_mult = 0.0
        self.last_yaw_mult = 0.0

        # ---- Keymap ----
        # move_bindings values are: (mode, x_mult, y_mult, yaw_mult)
        # mode is usually "twist"; diff-drive circle turns use mode "circle"
        self.move_bindings: Dict[str, Tuple[str, int, int, int]] = {}
        self._rebuild_keymap_for("diff_drive")  # safe default until robot selected

        # ---- Timers ----
        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)

        # Terminal settings will be captured when run() starts.
        self._terminal_settings = None

        # Initial UI (old-style)
        self._tprint(self.render_instructions(topic_name=None))
        self._tprint("[STARTUP] Discovering robots...")

    # ---------------- Terminal output (cooked mode only) ----------------

    def _tprint(self, text: str = ""):
        """
        Print reliably even while the program uses RAW mode for keypress reading.
        We temporarily restore cooked terminal settings for printing.
        """
        # If stdin isn't a tty, just print normally (useful for piping/logging)
        if not sys.stdin.isatty() or self._terminal_settings is None:
            print(text)
            sys.stdout.flush()
            return

        try:
            # Put terminal back to cooked for correct newline behavior
            restore_terminal_settings(self._terminal_settings)
        except Exception:
            print(text)
            sys.stdout.flush()
            return

        print(text if text else "")
        sys.stdout.flush()

        # Back to raw so key reading continues to work
        try:
            set_raw_mode()
        except Exception:
            pass

    # ---------------- ROS graph warm-up (startup discovery fix) ----------------

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        Warm up DDS/ROS graph discovery so get_topic_names_and_types() /
        get_subscriptions_info_by_topic() are correct on first display.

        Why this exists:
        - On startup, the ROS graph cache may not yet include remote peers.
        - If we query immediately, we may see (none found) even though robots exist.
        - A short spin_once loop processes discovery traffic and populates the graph.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            if self.list_available_robots():
                return

    # ---------------- State (helpful for future web UI) ----------------

    def get_state(self) -> TeleopState:
        robots = self.list_available_robots()
        return TeleopState(
            available_robots=robots,
            active_robot=self.current_robot_name,
            cmd_vel_topic=self.cmd_vel_topic,
            drive_type=self.current_drive_type,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            allow_cmd_vel_switching=self.allow_cmd_vel_switching,
            wheel_separation=self.wheel_separation,
        )

    # ---------------- Renderers (old-style UI, but drive-type aware) ----------------

    def render_controls_for_drive_type(self, drive_type: str) -> str:
        """
        Returns the “MOVEMENT:” section, but changes content based on drive type.
        """
        dt = (drive_type or "").strip()
        if dt == "mecanum":
            return "\n".join([
                "MOVEMENT (MECANUM / OMNI):",
                "  8 OR [UP]     forward",
                "  2 OR [DOWN]   backward",
                "  4 OR [LEFT]   strafe left",
                "  6 OR [RIGHT]  strafe right",
                "  7             forward-diagonal-left",
                "  9             forward-diagonal-right",
                "  1             backward-diagonal-left",
                "  3             backward-diagonal-right",
                "  0             rotate left",
                "  .             rotate right",
            ])
        else:
            return "\n".join([
                "MOVEMENT (DIFF-DRIVE):",
                "  8 OR [UP]     forward",
                "  2 OR [DOWN]   backward",
                "  4 OR [LEFT]   rotate left",
                "  6 OR [RIGHT]  rotate right",
                "  7             circle forward-left (right track only)",
                "  9             circle forward-right (left track only)",
                "  1             circle backward-left (right track only)",
                "  3             circle backward-right (left track only)",
            ])

    def render_current_speeds(self) -> str:
        return f"Linear Speed: {self.linear_speed:.2f}  Angular Speed: {self.angular_speed:.2f}"

    def render_instructions(self, topic_name: Optional[str]) -> str:
        """
        Old-style “big block” instructions, but now includes drive_type.
        """
        dt = self.current_drive_type or "unknown"

        lines = []
        lines.append("-------------------------------")
        lines.append("      ROBOT LEGION TELEOP      ")
        lines.append("-------------------------------")
        if topic_name:
            lines.append(f"Publishing Twist on: {topic_name}")
        else:
            lines.append("SELECT A ROBOT")

        lines.append(f"drive_type: {dt}")
        lines.append(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        lines.append(f"wheel_separation (circle turns): {self.wheel_separation:.2f} m")

        if self.current_robot_name:
            lines.append("")
            lines.append(f"ACTIVE ROBOT: {self.current_robot_name}")

        lines.append("")
        lines.append(self.render_controls_for_drive_type(dt))
        lines.append("")
        lines.append("STOP:")
        lines.append("  [SPACE] OR 's' OR 5")
        lines.append("")
        lines.append("SPEED PROFILES:")
        lines.append("  i slow, o medium, p fast")
        lines.append("")
        lines.append("Speed scaling:")
        lines.append("  q OR / increase linear")
        lines.append("  r OR * decrease linear")
        lines.append("  w OR + increase speed")
        lines.append("  e OR - decrease speed")
        lines.append("")
        lines.append("Robot selection:")
        lines.append("  m choose robot")
        lines.append("")
        lines.append("CTRL-C to quit.")
        lines.append("-------------------------------")
        lines.append(self.render_current_speeds())
        return "\n".join(lines)

    def render_robot_list(self, robots: List[str]) -> str:
        """
        Old-style robot list formatting (very readable in the field).
        """
        lines = []
        lines.append("")
        lines.append("======= AVAILABLE ROBOTS =======")
        if not robots:
            lines.append("")
            lines.append("  (none found)")
            lines.append("")
            lines.append("  - Try 'r' to refresh")
            lines.append("")
            lines.append("  - Make sure each robot is")
            lines.append("    on the same ROS_DOMAIN_ID")
            lines.append("    and network.")
            lines.append("")
            lines.append("  - Make sure your robots are")
            lines.append("    currently running at least")
            lines.append("    one node (motor_driver_node).")
            lines.append("")
        else:
            lines.append("")
            for r in robots:
                lines.append(f"  - {r}")
                lines.append("")
        lines.append("================================")
        lines.append("")
        return "\n".join(lines)

    # ---------------- Discovery helpers ----------------

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
        if not name:
            return False, ""
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        return (subs is not None and len(subs) > 0), topic

    # ---------------- Heartbeat (optional best-effort) ----------------

    def _heartbeat_cb(self, msg: String):
        """
        Parse heartbeat JSON payload published by heartbeat_node.py on the robot.
        """
        try:
            self._last_hb_json = json.loads(msg.data)
        except Exception:
            self._last_hb_json = None

    def _attach_heartbeat_sub(self, robot_name: str):
        """
        Subscribe to /<robot>/heartbeat. If nothing is publishing, this does no harm.
        """
        self._last_hb_json = None

        # Remove previous subscription when switching robots
        if self.hb_sub is not None:
            try:
                self.destroy_subscription(self.hb_sub)
            except Exception:
                pass
            self.hb_sub = None

        topic = f"/{robot_name}/heartbeat"
        self.hb_sub = self.create_subscription(String, topic, self._heartbeat_cb, 10)

        # Give it a moment to arrive (best effort)
        deadline = time.monotonic() + 0.35
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_hb_json is not None:
                break

    def _resolve_drive_type_for_robot(self, robot_name: str) -> str:
        """
        Decide whether the selected robot is diff_drive or mecanum.

        Priority:
          1) Heartbeat payload (if received)
          2) YAML registry mapping (robot_profiles.yaml)
          3) fallback diff_drive
        """
        # YAML baseline
        drive_type = "diff_drive"
        try:
            prof = resolve_robot_profile(self.profile_registry, robot_name)
            dt = str(prof.get("drive_type", "diff_drive")).strip()
            if dt:
                drive_type = dt
        except Exception:
            pass

        # Heartbeat override (robot “self-identifies”)
        if isinstance(self._last_hb_json, dict):
            dt = self._last_hb_json.get("drive_type", None)
            if isinstance(dt, str) and dt.strip():
                drive_type = dt.strip()

        return "mecanum" if drive_type == "mecanum" else "diff_drive"

    # ---------------- Keymap ----------------

    def _rebuild_keymap_for(self, drive_type: str):
        """
        Build movement bindings based on drive type.

        The rest of the code (main loop) reads from self.move_bindings and
        produces Twist commands accordingly.
        """
        dt = (drive_type or "").strip()

        if dt == "mecanum":
            # mode, x_mult, y_mult, yaw_mult
            self.move_bindings = {
                # arrows
                "\x1b[A": ("twist", +1, 0, 0),   # up -> forward
                "\x1b[B": ("twist", -1, 0, 0),   # down -> backward
                "\x1b[D": ("twist", 0, +1, 0),   # left -> strafe left
                "\x1b[C": ("twist", 0, -1, 0),   # right -> strafe right

                # numpad style
                "8": ("twist", +1, 0, 0),
                "2": ("twist", -1, 0, 0),
                "4": ("twist", 0, +1, 0),
                "6": ("twist", 0, -1, 0),

                # diagonals
                "7": ("twist", +1, +1, 0),
                "9": ("twist", +1, -1, 0),
                "1": ("twist", -1, +1, 0),
                "3": ("twist", -1, -1, 0),

                # rotate
                "0": ("twist", 0, 0, +1),
                ".": ("twist", 0, 0, -1),
            }
        else:
            # diff drive (legacy behavior preserved)
            self.move_bindings = {
                "\x1b[A": ("twist", +1, 0, 0),   # forward
                "\x1b[B": ("twist", -1, 0, 0),   # backward
                "\x1b[D": ("twist", 0, 0, +1),   # rotate left
                "\x1b[C": ("twist", 0, 0, -1),   # rotate right

                "8": ("twist", +1, 0, 0),
                "2": ("twist", -1, 0, 0),
                "4": ("twist", 0, 0, +1),
                "6": ("twist", 0, 0, -1),

                # circle turns keep legacy behavior (handled specially)
                "7": ("circle", 0, 0, 0),
                "9": ("circle", 0, 0, 0),
                "1": ("circle", 0, 0, 0),
                "3": ("circle", 0, 0, 0),
            }

    # ---------------- Active robot helpers ----------------

    def _publish_active(self):
        """
        Publish which robot is active so other nodes (like FPV mux / web UI) can follow.
        """
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
        """
        Switch the teleop publisher to a new /<robot>/cmd_vel topic.
        Also resolves drive type and rebuilds keymap accordingly.
        """
        # Destroy prior publisher if any
        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass
            self.publisher_ = None

        self.cmd_vel_topic = topic
        self.current_robot_name = topic.split("/")[1] if topic.startswith("/") else topic

        # Attach heartbeat, then resolve drive type
        self._attach_heartbeat_sub(self.current_robot_name)
        self.current_drive_type = self._resolve_drive_type_for_robot(self.current_robot_name)

        # Rebuild keymap now that we know the drive type
        self._rebuild_keymap_for(self.current_drive_type)

        # Create ROS publisher to the selected robot
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Publish meta-state
        self._publish_active()

        # Old-style: show full instructions every time robot changes
        self._tprint(self.render_instructions(self.cmd_vel_topic))
        self._tprint(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name} (drive_type={self.current_drive_type})")

    # ---------------- Circle-turn helper (diff-drive only) ----------------

    def _publish_one_track_circle(self, v_left: float, v_right: float):
        """
        Legacy “one track only” circle behavior.

        We force one-track-only motion by solving for Twist that yields desired track speeds:

            v_left  = v - w*L/2
            v_right = v + w*L/2

        Inverse:
            v = (v_left + v_right)/2
            w = (v_right - v_left)/L
        """
        L = self.wheel_separation if self.wheel_separation > 1e-6 else 0.18
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / L

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True

        if self.publisher_ is not None:
            self.publisher_.publish(twist)

    # ---------------- Republish + Offline watchdog ----------------

    def _republish_last_twist(self):
        """
        If we are moving, re-publish the last Twist periodically so motion continues smoothly.
        """
        if not self.is_moving or self.publisher_ is None:
            return
        self.publisher_.publish(self.last_twist)

    def _offline_watchdog(self):
        """
        If current cmd_vel topic has no subscribers, the robot is considered offline.
        This mirrors the old teleop behavior: stop and force re-selection.
        """
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return

        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # Robot went offline
        self._tprint("")
        self._tprint(f"[OFFLINE] Robot '{self.current_robot_name}' appears offline (no subscribers on {self.cmd_vel_topic}).")
        self._tprint("[OFFLINE] Stopping and returning to robot selection.")

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

        self._tprint("[OFFLINE] Press 'm' to select another robot.")

    # ---------------- Speed modifiers (old behavior) ----------------

    def _print_speeds(self):
        self._tprint(self.render_current_speeds())

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._print_speeds()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._print_speeds()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._print_speeds()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._print_speeds()

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._print_speeds()

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._print_speeds()

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._print_speeds()

    # ---------------- Robot selection prompt (old-style) ----------------

    def _prompt_until_valid_robot(self):
        """
        Print robots, prompt until user enters a valid robot name.
        Old-style UX: show list, then a simple prompt.
        """
        first_pass = True

        while rclpy.ok():
            # Warm up discovery so the initial list is correct on startup.
            if first_pass:
                self._warmup_discovery(timeout_sec=1.0)
                first_pass = False

            robots = self.list_available_robots()
            self._tprint(self.render_robot_list(robots))

            # Must be in cooked mode for input()
            restore_terminal_settings(self._terminal_settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name (or 'r' to refresh): ").strip()
            except Exception:
                set_raw_mode()
                continue

            # Back to raw for key reading after input()
            set_raw_mode()

            if user_input.lower() == "r":
                self._warmup_discovery(timeout_sec=0.5)
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                self._tprint(f"[INVALID] '{user_input}' is not available (no subscriber on {topic}). Try again.\n")
                continue

            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                self._tprint("[ROBOT SWITCH] Switching disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # ---------------- Main loop ----------------

    def run(self):
        # Save terminal settings once so all printing can temporarily restore them.
        self._terminal_settings = termios.tcgetattr(sys.stdin)

        # Enter raw mode for key reading
        set_raw_mode()

        # Initial selection prompt
        self._prompt_until_valid_robot()

        try:
            while rclpy.ok():
                key = read_key_raw(timeout_s=0.1)

                # Let ROS process discovery/heartbeat even if no key is pressed
                if key == "":
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                if key == "\x03":  # Ctrl-C
                    break

                # If no robot selected, only allow selection + speed controls
                if self.publisher_ is None:
                    if key == "m":
                        self._prompt_until_valid_robot()
                    elif key in ("i", "o", "p", "w", "+", "e", "-", "q", "/", "r", "*"):
                        # speed changes allowed even unselected
                        if key == "i":
                            self._set_slow_profile()
                        elif key == "o":
                            self._set_medium_profile()
                        elif key == "p":
                            self._set_fast_profile()
                        elif key in ("w", "+"):
                            self._increase_both_speeds()
                        elif key in ("e", "-"):
                            self._decrease_both_speeds()
                        elif key in ("q", "/"):
                            self._increase_linear_speed()
                        elif key in ("r", "*"):
                            self._decrease_linear_speed()
                    elif key in (" ", "5", "s"):
                        self._tprint("[STOP] (no robot selected)")
                    else:
                        self._tprint("[INFO] No robot selected. Press 'm' to select a robot.")
                    continue

                # Stop (works for all drive types)
                if key in (" ", "5", "s"):
                    try:
                        self.publisher_.publish(Twist())
                    except Exception:
                        pass
                    self.is_moving = False
                    self.last_twist = Twist()
                    self.last_lin_mult = 0.0
                    self.last_lat_mult = 0.0
                    self.last_yaw_mult = 0.0
                    self._tprint("[STOP]")
                    continue

                # Switch robot
                if key == "m":
                    self._prompt_until_valid_robot()
                    continue

                # Speed profiles
                if key == "i":
                    self._set_slow_profile()
                    continue
                if key == "o":
                    self._set_medium_profile()
                    continue
                if key == "p":
                    self._set_fast_profile()
                    continue

                # Speed scaling
                if key in ("w", "+"):
                    self._increase_both_speeds()
                    continue
                if key in ("e", "-"):
                    self._decrease_both_speeds()
                    continue
                if key in ("q", "/"):
                    self._increase_linear_speed()
                    continue
                if key in ("r", "*"):
                    self._decrease_linear_speed()
                    continue

                # Movement keys (drive-type-aware via self.move_bindings)
                if key in self.move_bindings:
                    mode, x_mult, y_mult, yaw_mult = self.move_bindings[key]

                    # Diff-drive circle turns (legacy)
                    if mode == "circle":
                        # Only do circles if we're actually controlling a diff-drive robot.
                        # If somehow mapped on mecanum, we just ignore.
                        if (self.current_drive_type or "diff_drive") != "diff_drive":
                            continue

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

                    # Normal Twist generation
                    twist = Twist()

                    # Forward/back always uses linear.x
                    twist.linear.x = self.linear_speed * float(x_mult)

                    # Mecanum: strafe uses linear.y, rotate uses angular.z
                    if (self.current_drive_type or "diff_drive") == "mecanum":
                        twist.linear.y = self.linear_speed * float(y_mult)
                        twist.angular.z = self.angular_speed * float(yaw_mult)
                    else:
                        # Diff: lateral ignored, rotate uses angular.z
                        twist.angular.z = self.angular_speed * float(yaw_mult)

                    self.last_twist = twist
                    self.is_moving = True

                    try:
                        self.publisher_.publish(twist)
                    except Exception:
                        pass

        finally:
            # Best effort stop + restore terminal
            try:
                if self.publisher_ is not None:
                    self.publisher_.publish(Twist())
            except Exception:
                pass
            try:
                restore_terminal_settings(self._terminal_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
