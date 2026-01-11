#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for swarm robotics applications (robot-agnostic)

Core behaviors:
- Robots are identified by "robot name" (default = Linux username on robot)
- A robot is considered "available" if there is at least one subscriber on:
    /<robot_name>/cmd_vel
  (i.e., motor_driver_node is running on that robot)

Startup:
- Prints list of available robots, prompts user to pick one.
- Keeps prompting until a valid robot name is entered.

Switching:
- Press 'm' any time to switch robot (same validation loop).

Offline detection:
- While driving, if the current robot stops subscribing to /<robot>/cmd_vel,
  teleop stops and returns to selection prompt.

Tracked robot turning:
- '4' rotates LEFT  (left track backward, right track forward)  -> angular.z positive
- '6' rotates RIGHT (left track forward, right track backward)  -> angular.z negative

Circle turns (smallest radius; ONE TRACK ONLY):
- 7 forward-left  : RIGHT track forward, LEFT track stopped
- 9 forward-right : LEFT track forward, RIGHT track stopped
- 1 backward-left : RIGHT track backward, LEFT track stopped
- 3 backward-right: LEFT track backward, RIGHT track stopped
"""

import re
import sys
import select
import termios
import tty
from typing import List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")


def get_key(settings):
    """Read a single keypress (with arrow-key escape sequence support)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == "\x1b":  # start of escape sequence
            key += sys.stdin.read(2)
        return key
    return ""


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        # Optional parameters (kept, but now robot selection is interactive by default)
        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)

        # Used for circle-turn math (must match motor_driver_node)
        self.declare_parameter("wheel_separation", 0.18)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        # Active robot publishers (used by fpv_camera_mux)
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/teleop/active_robot", 10)

        # Publisher created after robot selection
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None

        # Speed profile
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Republish state
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self.is_moving = False

        # KEYMAP (7/9/1/3 handled specially for one-track-only circles)
        self.move_bindings = {
            # Arrow keys
            "\x1b[A": ("twist", 1, 0),     # up
            "\x1b[B": ("twist", -1, 0),    # down
            "\x1b[D": ("twist", 0, 1),     # left  (rotate left)
            "\x1b[C": ("twist", 0, -1),    # right (rotate right)

            # Numpad
            "8": ("twist", 1, 0),          # forward
            "2": ("twist", -1, 0),         # backward

            # Rotate in place
            "4": ("twist", 0, 1),          # rotate LEFT
            "6": ("twist", 0, -1),         # rotate RIGHT

            # One-track-only circle turns (handled specially)
            "7": ("circle", None, None),   # forward-left  (right track only)
            "9": ("circle", None, None),   # forward-right (left track only)
            "1": ("circle", None, None),   # backward-left (right track only)
            "3": ("circle", None, None),   # backward-right(left track only)
        }

        self.speed_bindings = {
            "w": self._increase_both_speeds,
            "+": self._increase_both_speeds,
            "e": self._decrease_both_speeds,
            "-": self._decrease_both_speeds,
            "q": self._increase_linear_speed,
            "/": self._increase_linear_speed,
            "r": self._decrease_linear_speed,
            "*": self._decrease_linear_speed,
            "i": self._set_slow_profile,
            "o": self._set_medium_profile,
            "p": self._set_fast_profile,
        }

        # Timers
        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)

        # Start with instructions + robot picker
        self._print_instructions(None)
        print("[STARTUP] Discovering robots...")
        # Robot selection will happen inside run() so we can safely use input()

    # ---------------- Printing ----------------

    def _print_instructions(self, topic_name: Optional[str]):
        print("-------------------------------")
        print("      ROBOT LEGION TELEOP      ")
        print("-------------------------------")
        if topic_name:
            print(f"Publishing Twist on: {topic_name}")
        else:
            print("SELECT A ROBOT")
        print(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        print(f"wheel_separation (for circle turns): {self.wheel_separation:.2f} m")
        if self.current_robot_name:
            print(f"ACTIVE ROBOT: {self.current_robot_name}")
        print("")
        print("MOVEMENT:")
        print("  8 forward")
        print("  2 backward")
        print("  4 rotate-left")
        print("  6 rotate-right")
        print("  7 circle forward-left")
        print("  9 circle forward-right")
        print("  1 circle backward-left")
        print("  3 circle backward-right")
        print("")
        print("ARROW KEYS ALSO WORK")
        print(" [UP] forward")
        print(" [DOWN] backward")
        print(" [LEFT] rotate left")
        print(" [RIGHT] rotate right")
        print("")
        print("STOP:")
        print("  [SPACE] OR 's' OR 5")
        print("")
        print("SPEED PROFILES:")
        print("  i slow, o medium, p fast")
        print("")
        print("Speed scaling:")
        print("  q OR / increase linear")
        print("  r OR * decrease linear")
        print("  w OR + increase speed")
        print("  e OR - decrease speed")

        print("")
        print("Robot selection:")
        print("  m choose robot")
        print("")
        print("CTRL-C to quit.")
        print("-------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        print(f"Linear Speed: {self.linear_speed:.2f}  Angular Speed: {self.angular_speed:.2f}")

    # ---------------- Discovery helpers ----------------

    def _topic_to_robot(self, topic: str) -> Optional[str]:
        m = CMD_VEL_RE.match(topic)
        return m.group(1) if m else None

    def _discover_candidate_cmd_vel_topics(self) -> List[str]:
        """
        Return topics that look like /<robot>/cmd_vel.
        """
        topics_and_types = self.get_topic_names_and_types()
        topics = [t for (t, _types) in topics_and_types]
        return [t for t in topics if CMD_VEL_RE.match(t)]

    def list_available_robots(self) -> List[str]:
        """
        Available robots are those with at least one subscriber on /<robot>/cmd_vel.
        That means the robot's motor driver node is up and listening.
        """
        robots: Set[str] = set()
        for t in self._discover_candidate_cmd_vel_topics():
            subs = self.get_subscriptions_info_by_topic(t)
            if subs and len(subs) > 0:
                robot = self._topic_to_robot(t)
                if robot:
                    robots.add(robot)
        return sorted(robots)

    def _print_robot_list(self, robots: List[str]):
        print("")
        print("======= AVAILABLE ROBOTS =======")
        if not robots:
            print("")
            print("  (none found)")
            print("")
            print("  - Try 'r' to refresh")
            print("")
            print("  - Make sure each robot  is")
            print("    on the same ROS_DOMAIN_ID")
            print("    and network.")
            print("")
            print("  - Make sure your robots are ")
            print("    currently running at least")
            print("    one node.")
            print("")
        else:
            for r in robots:
                print("")
                print(f"  - {r}")
                print("")
        print("================================")
        print("")

    def _validate_robot_name(self, name: str) -> Tuple[bool, str]:
        """
        Returns (is_valid, cmd_vel_topic).
        """
        name = name.strip()
        if not name:
            return False, ""
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        if subs and len(subs) > 0:
            return True, topic
        return False, topic

    # ---------------- Active robot helpers ----------------

    def _publish_active_robot(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

    # ---------------- Topic application ----------------

    def _apply_cmd_vel_topic(self, new_topic: str):
        if not new_topic.startswith("/"):
            new_topic = "/" + new_topic

        if new_topic == "/cmd_vel":
            print("[ERROR] Refusing to use /cmd_vel. Use /<robot>/cmd_vel for multi-robot architecture.")
            return

        robot = self._topic_to_robot(new_topic)
        if robot:
            self.current_robot_name = robot

        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        self._publish_active_robot()
        self._print_instructions(new_topic)
        print(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

    # ---------------- Circle-turn helper ----------------

    def _publish_one_track_circle(self, v_left: float, v_right: float):
        """
        Force one-track-only motion by solving for Twist that yields the desired track speeds:

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

        self.publisher_.publish(twist)

    # ---------------- Republish + Offline watchdog ----------------

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
            return

        if abs(self.last_twist.linear.x) > 1e-9 or abs(self.last_twist.angular.z) > 1e-9:
            self.publisher_.publish(self.last_twist)
            return

        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return

        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self.publisher_.publish(twist)

    def _offline_watchdog(self):
        """
        If current cmd_vel topic has no subscribers, the robot is "offline".
        Stop and force re-selection.
        """
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return

        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # Robot went offline
        print("")
        print(f"[OFFLINE] Robot '{self.current_robot_name}' appears offline (no subscribers on {self.cmd_vel_topic}).")
        print("[OFFLINE] Stopping and returning to robot selection.")
        try:
            self.publisher_.publish(Twist())
        except Exception:
            pass

        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()

        # Destroy publisher so main loop forces selection again
        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass
        self.publisher_ = None
        self.cmd_vel_topic = None
        self.current_robot_name = None

    # ---------------- Speed modifiers ----------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._print_current_speeds()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._print_current_speeds()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._print_current_speeds()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._print_current_speeds()

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._print_current_speeds()

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._print_current_speeds()

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._print_current_speeds()

    # ---------------- Robot selection prompt ----------------

    def _prompt_until_valid_robot(self, settings):
        """
        Print robots, prompt until user enters a valid robot name.
        """
        while rclpy.ok():
            robots = self.list_available_robots()
            self._print_robot_list(robots)

            restore_terminal_settings(settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name (or 'r' to refresh): ").strip()
            except Exception:
                tty.setraw(sys.stdin.fileno())
                continue
            tty.setraw(sys.stdin.fileno())

            if user_input.lower() == "r":
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                print(f"[INVALID] '{user_input}' is not an available robot (no subscriber on {topic}). Try again.\n")
                continue

            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                print("[ROBOT SWITCH] Switching disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # ---------------- Main loop ----------------

    def run(self):
        settings = termios.tcgetattr(sys.stdin)

        # initial selection
        self._prompt_until_valid_robot(settings)

        try:
            while rclpy.ok():
                key = get_key(settings)
                if key == "":
                    continue
                if key == "\x03":  # Ctrl-C
                    break

                # If we currently have no robot selected (offline or startup), force selection
                if self.publisher_ is None:
                    if key == "m":
                        self._prompt_until_valid_robot(settings)
                    elif key in self.speed_bindings:
                        self.speed_bindings[key]()
                    elif key in (" ", "5", "s"):
                        print("[STOP] (no robot selected)")
                    else:
                        print("[INFO] No robot selected. Press 'm' to select a robot.")
                    continue

                # Movement keys
                if key in self.move_bindings:
                    mode, lin_mult, ang_mult = self.move_bindings[key]

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

                    # Normal twist
                    self.last_lin_mult = float(lin_mult)
                    self.last_ang_mult = float(ang_mult)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult
                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

                # Stop
                if key in (" ", "5", "s"):
                    self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    print("[STOP]")
                    continue

                # Switch robot
                if key == "m":
                    self._prompt_until_valid_robot(settings)
                    continue

                # Speed controls
                if key in self.speed_bindings:
                    self.speed_bindings[key]()
                    continue

        finally:
            try:
                if self.publisher_ is not None:
                    self.publisher_.publish(Twist())
            except Exception:
                pass
            restore_terminal_settings(settings)


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