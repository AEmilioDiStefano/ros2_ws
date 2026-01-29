# teleop_legion_key.py
# Proprietary - Vitruvian Systems LLC
#
# PURPOSE
# -------
# Keyboard teleoperation for a heterogeneous fleet (diff drive + mecanum + future types).
#
# KEY DESIGN GOAL
# ---------------
# Switching robots must NOT accidentally change control schemes.
# The selected robot's drive scheme is determined *per robot*, primarily from heartbeat.
#
# IMPORTANT CONCEPTS (for ROS2 beginners)
# --------------------------------------
# - Each robot publishes Twist commands on its own namespace topic:
#       /<robot_name>/cmd_vel
# - Each robot also publishes a JSON heartbeat on:
#       /<robot_name>/heartbeat
#   which includes:
#       {"robot":"robot2","drive_type":"mecanum","profile":"mecanum_tb6612", ...}
#
# Teleop uses heartbeat to "self-identify" the drive type for the currently selected robot.
# This avoids hard-coding robot IDs and makes heterogeneous fleets scalable.

from __future__ import annotations

import json
import os
import sys
import time
import tty
import termios
import select
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_legion_teleop_python.drive_profiles import DriveProfileRegistry


# ----------------------------
# Small terminal helper utils
# ----------------------------

def _clear_screen():
    # ANSI clear screen + move cursor home
    print("\033[2J\033[H", end="")


def _now_ms() -> int:
    return int(time.time() * 1000)


def _fmt_robot(robot: str) -> str:
    return robot or "<none>"


# ----------------------------
# Teleop configuration
# ----------------------------

@dataclass
class Keymap:
    """
    Defines how keys map to motion for a given drive type.
    We compute a Keymap per selected robot based on drive_type.
    """
    # Each key maps to (vx, vy, wz) multipliers in normalized units (-1..1)
    bindings: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)

    # Key hint text shown in the help panel
    help_lines: List[str] = field(default_factory=list)


@dataclass
class TeleopState:
    # Discovered robots (derived from cmd_vel topics)
    available_robots: List[str] = field(default_factory=list)

    # Which robot is currently selected
    current_robot: Optional[str] = None

    # Current robot's drive type ("diff_drive", "mecanum", ...)
    current_drive_type: str = "diff_drive"

    # Speed scaling (teleop-side)
    linear_scale: float = 0.35
    angular_scale: float = 0.65

    # When we last printed the UI
    last_render_ms: int = 0


class TeleopLegionKey(Node):
    """
    Main teleop node.
    - Discovers robots by finding /<robot>/cmd_vel topics.
    - Allows selecting a robot (key 'm').
    - Publishes Twist to the selected robot.
    - Subscribes to that robot's heartbeat to learn drive_type dynamically.
    """

    def __init__(self):
        super().__init__("teleop_legion_key")

        # Registry loads robot_profiles.yaml (teleop-side) for convenience defaults.
        # Heartbeat remains the authoritative source whenever available.
        self.registry = DriveProfileRegistry()

        self.state = TeleopState()

        # Publisher is created dynamically when a robot is selected.
        self._cmd_pub = None

        # Heartbeat subscriber is also created dynamically for the selected robot.
        self._hb_sub = None

        # --- CRITICAL FIX ---
        # Store heartbeat by robot name (so we never apply robotA's drive_type to robotB).
        self._last_hb_by_robot: Dict[str, dict] = {}

        # Cache drive_type learned previously, so switching back is instant even
        # if heartbeat is delayed on WiFi.
        self._drive_type_cache: Dict[str, str] = {}

        # Current keymap for the selected robot drive type
        self._keymap = self._make_keymap(self.state.current_drive_type)

        # For UI responsiveness
        self._last_discovery_ms = 0

        # Tell users what to do
        self._print_status_banner(extra_line="Starting up... discovering robots.")

    # ----------------------------
    # ROS2 discovery / selection
    # ----------------------------

    def _discover_robots(self) -> List[str]:
        """
        Discover robots by scanning ROS graph topics and collecting
        anything matching /<robot_name>/cmd_vel
        """
        robots = set()
        for name, _types in self.get_topic_names_and_types():
            # We look for topics like: /robot1/cmd_vel
            if name.endswith("/cmd_vel"):
                parts = name.split("/")
                # ['', 'robot1', 'cmd_vel'] => robot1
                if len(parts) >= 3 and parts[-1] == "cmd_vel":
                    robots.add(parts[-2])

        return sorted(list(robots))

    def _ensure_robot_list_fresh(self, min_interval_s: float = 0.75):
        """
        Refresh robot list periodically, and also on-demand (key 'r').
        """
        now = time.time()
        if (now - (self._last_discovery_ms / 1000.0)) < min_interval_s and self.state.available_robots:
            return

        self.state.available_robots = self._discover_robots()
        self._last_discovery_ms = _now_ms()

        # If no robot selected yet, auto-select the first available robot.
        if self.state.current_robot is None and self.state.available_robots:
            self._select_robot(self.state.available_robots[0])

    def _select_robot(self, robot: str):
        """
        Switch teleop control to a different robot.

        This does NOT hard-code any robot IDs. We just switch the namespace.
        """
        if not robot:
            return

        self.state.current_robot = robot

        # Create publisher for the selected robot cmd_vel
        topic = f"/{robot}/cmd_vel"
        self._cmd_pub = self.create_publisher(Twist, topic, 10)

        # Subscribe to this robot's heartbeat so it can self-identify drive_type/profile
        self._attach_heartbeat_subscriber(robot)

        # Determine drive type using per-robot heartbeat/cache/yaml fallback
        self.state.current_drive_type = self._resolve_drive_type_for_robot(robot)

        # Rebuild keymap for the selected robot
        self._rebuild_keymap_for_current_robot()

        self._print_status_banner(extra_line=f"Selected robot: {robot}")

    def _attach_heartbeat_subscriber(self, robot: str):
        """
        Subscribe to /<robot>/heartbeat for the currently selected robot.

        IMPORTANT: Heartbeat is best-effort and may arrive after we switch.
        We store heartbeat per robot and can hot-swap keymaps when drive_type updates.
        """
        # Destroy old subscriber if it exists
        if self._hb_sub is not None:
            try:
                self.destroy_subscription(self._hb_sub)
            except Exception:
                pass
            self._hb_sub = None

        hb_topic = f"/{robot}/heartbeat"
        self._hb_sub = self.create_subscription(String, hb_topic, self._heartbeat_cb, 10)

        # Give heartbeat time to arrive on slower WiFi links.
        # (Even if it doesn't arrive here, our callback will update later.)
        time.sleep(1.0)

    def _heartbeat_cb(self, msg: String):
        """Heartbeat callback.

        Each robot runs `heartbeat_node.py` and publishes a small JSON blob on:
            /<robot_name>/heartbeat

        We store the decoded heartbeat *by robot name* so we never accidentally apply
        a different robot's drive_type/profile when switching robots.

        Expected heartbeat fields:
            - robot (string): robot identifier (namespace)
            - drive_type (string): 'diff_drive' or 'mecanum' (or future types)
            - profile (string): the selected profile name in robot_profiles.yaml
        """
        try:
            data = json.loads(msg.data)

            # The robot should include its own ID, but fall back to selected robot if missing.
            robot = str(data.get("robot", "")).strip() or (self.state.current_robot or "")
            if not robot:
                return

            self._last_hb_by_robot[robot] = data

            # Cache drive_type immediately so switching back later doesn't "inherit"
            # the previous robot's drive scheme if a heartbeat hasn't arrived yet.
            dt = data.get("drive_type")
            if isinstance(dt, str) and dt:
                self._drive_type_cache[robot] = dt

                # If this heartbeat is for the currently selected robot and the drive_type
                # differs from what we're currently using, hot-swap the keymap.
                if robot == self.state.current_robot and dt != self.state.current_drive_type:
                    self.state.current_drive_type = dt
                    self._rebuild_keymap_for_current_robot()
                    self._print_status_banner(extra_line=f"Drive type updated from heartbeat: {dt}")

        except Exception:
            # Heartbeat is strictly best-effort. Never crash teleop because of it.
            return

    def _resolve_drive_type_for_robot(self, robot_name: str) -> str:
        """Return the drive_type for a robot.

        Priority order (most reliable first):
          1) Heartbeat self-identification for *that same robot*
          2) Cached drive_type from a previous heartbeat (if heartbeat is momentarily missed)
          3) Local YAML mapping (robot_profiles.yaml) on THIS machine (teleop laptop)
          4) Fallback to diff_drive (safe default)

        Why this matters:
          - Without per-robot heartbeat storage, it's easy to accidentally apply the
            previous robot's drive scheme when you switch robots quickly.
          - Caching prevents a robot from being misclassified if we haven't yet
            received a fresh heartbeat after switching back to it.
        """
        hb = self._last_hb_by_robot.get(robot_name)
        if isinstance(hb, dict):
            dt = hb.get("drive_type")
            if isinstance(dt, str) and dt:
                return dt

        cached = self._drive_type_cache.get(robot_name)
        if isinstance(cached, str) and cached:
            return cached

        dt = self.registry.robot_drive_type(robot_name)
        if isinstance(dt, str) and dt:
            return dt

        return "diff_drive"

    def _rebuild_keymap_for_current_robot(self):
        """Build key bindings appropriate for the currently selected robot drive type."""
        self._keymap = self._make_keymap(self.state.current_drive_type)

    # ----------------------------
    # Keymaps (diff vs mecanum)
    # ----------------------------

    def _make_keymap(self, drive_type: str) -> Keymap:
        """
        Create a Keymap for a drive type.

        Diff-drive:
          - Arrow keys / WASD translate forward/back + rotate
        Mecanum:
          - Arrow keys / WASD translate (including strafe) + rotate
        """
        km = Keymap()

        if drive_type == "mecanum":
            km.bindings = {
                # Translation: (vx, vy, wz)
                "w": (1.0, 0.0, 0.0),
                "s": (-1.0, 0.0, 0.0),
                "a": (0.0, 1.0, 0.0),   # strafe left
                "d": (0.0, -1.0, 0.0),  # strafe right

                # Diagonals
                "q": (1.0, 1.0, 0.0),
                "e": (1.0, -1.0, 0.0),
                "z": (-1.0, 1.0, 0.0),
                "c": (-1.0, -1.0, 0.0),

                # Rotation
                "j": (0.0, 0.0, 1.0),
                "l": (0.0, 0.0, -1.0),

                # Stop
                " ": (0.0, 0.0, 0.0),
            }
            km.help_lines = [
                "Drive type: mecanum",
                "  w/s: forward/back",
                "  a/d: strafe left/right",
                "  q/e/z/c: diagonals",
                "  j/l: rotate left/right",
                "  space: STOP",
            ]
        else:
            # Default / safe fallback: diff drive
            km.bindings = {
                "w": (1.0, 0.0, 0.0),
                "s": (-1.0, 0.0, 0.0),
                "a": (0.0, 0.0, 1.0),
                "d": (0.0, 0.0, -1.0),
                " ": (0.0, 0.0, 0.0),
            }
            km.help_lines = [
                "Drive type: diff_drive",
                "  w/s: forward/back",
                "  a/d: rotate left/right",
                "  space: STOP",
            ]

        km.help_lines += [
            "",
            "Fleet keys:",
            "  m: switch robot",
            "  r: refresh robot list",
            "  +/-: change linear speed",
            "  [ / ]: change angular speed",
            "  x: quit",
        ]
        return km

    # ----------------------------
    # Terminal UI
    # ----------------------------

    def _print_status_banner(self, extra_line: str = ""):
        _clear_screen()
        print("Robot Legion Teleop (heterogeneous fleet)")
        print("----------------------------------------")
        print(f"Selected robot : {_fmt_robot(self.state.current_robot)}")
        print(f"Drive type     : {self.state.current_drive_type}")
        print(f"Lin scale      : {self.state.linear_scale:.2f}")
        print(f"Ang scale      : {self.state.angular_scale:.2f}")
        if extra_line:
            print("")
            print(extra_line)
        print("")
        print("Available robots:")
        if self.state.available_robots:
            for r in self.state.available_robots:
                marker = "*" if r == self.state.current_robot else " "
                print(f"  {marker} {r}")
        else:
            print("  (none discovered yet)")
        print("")
        for line in self._keymap.help_lines:
            print(line)
        print("")

    # ----------------------------
    # Key reading (raw terminal)
    # ----------------------------

    def _get_key(self, timeout_s: float = 0.10) -> Optional[str]:
        """
        Non-blocking key read from terminal.
        Returns:
          - a single character (like 'w')
          - None if no key pressed
        """
        dr, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not dr:
            return None
        return sys.stdin.read(1)

    # ----------------------------
    # Publishing Twist
    # ----------------------------

    def _publish_twist(self, vx: float, vy: float, wz: float):
        if self._cmd_pub is None or self.state.current_robot is None:
            return

        msg = Twist()
        msg.linear.x = vx * self.state.linear_scale
        msg.linear.y = vy * self.state.linear_scale
        msg.angular.z = wz * self.state.angular_scale
        self._cmd_pub.publish(msg)

    # ----------------------------
    # Robot switching prompt
    # ----------------------------

    def _prompt_switch_robot(self):
        """
        Switch robot without hard-coding IDs:
        - Print list
        - User types index
        """
        if not self.state.available_robots:
            self._print_status_banner(extra_line="No robots available to switch to.")
            return

        _clear_screen()
        print("Switch Robot")
        print("------------")
        for i, r in enumerate(self.state.available_robots):
            print(f"  [{i}] {r}")

        print("")
        print("Enter index and press ENTER (or blank to cancel): ", end="")
        sys.stdout.flush()

        # Restore cooked mode briefly so ENTER works normally.
        # We'll do a simple line read here.
        line = sys.stdin.readline().strip()
        if not line:
            self._print_status_banner(extra_line="Switch cancelled.")
            return

        try:
            idx = int(line)
            if idx < 0 or idx >= len(self.state.available_robots):
                raise ValueError()
        except Exception:
            self._print_status_banner(extra_line="Invalid index.")
            return

        self._select_robot(self.state.available_robots[idx])

    # ----------------------------
    # Main loop
    # ----------------------------

    def spin_teleop(self):
        """
        Main teleop loop:
          - periodic discovery
          - raw keyboard read
          - publish Twist
        """
        # Put terminal into raw mode
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                # Keep robot list reasonably fresh
                self._ensure_robot_list_fresh()

                # Process ROS callbacks (heartbeat updates, graph changes, etc.)
                rclpy.spin_once(self, timeout_sec=0.0)

                key = self._get_key(timeout_s=0.10)
                if key is None:
                    continue

                key = key.lower()

                # Quit
                if key == "x":
                    self._publish_twist(0.0, 0.0, 0.0)
                    break

                # Refresh robot list
                if key == "r":
                    self.state.available_robots = self._discover_robots()
                    self._print_status_banner(extra_line="Refreshed robot list.")
                    continue

                # Switch robot
                if key == "m":
                    self._publish_twist(0.0, 0.0, 0.0)
                    # IMPORTANT: temporarily exit raw mode for readline
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
                    try:
                        self._prompt_switch_robot()
                    finally:
                        tty.setcbreak(sys.stdin.fileno())
                    continue

                # Speed adjustments
                if key == "+":
                    self.state.linear_scale = min(2.00, self.state.linear_scale + 0.05)
                    self._print_status_banner(extra_line="Linear speed increased.")
                    continue
                if key == "-":
                    self.state.linear_scale = max(0.05, self.state.linear_scale - 0.05)
                    self._print_status_banner(extra_line="Linear speed decreased.")
                    continue
                if key == "]":
                    self.state.angular_scale = min(2.00, self.state.angular_scale + 0.05)
                    self._print_status_banner(extra_line="Angular speed increased.")
                    continue
                if key == "[":
                    self.state.angular_scale = max(0.05, self.state.angular_scale - 0.05)
                    self._print_status_banner(extra_line="Angular speed decreased.")
                    continue

                # Motion bindings
                if key in self._keymap.bindings:
                    vx, vy, wz = self._keymap.bindings[key]
                    self._publish_twist(vx, vy, wz)
                else:
                    # Unknown key -> ignore
                    pass

        finally:
            # Restore terminal mode
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = TeleopLegionKey()
    try:
        node.spin_teleop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
