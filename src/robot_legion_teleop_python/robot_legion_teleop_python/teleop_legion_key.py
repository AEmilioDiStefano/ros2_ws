#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for swarm robotics applications

- Multi-robot-first: publish to /<robot_name>/cmd_vel (namespaced)
- If startup_robot is provided, then immediately target /<startup_robot>/cmd_vel.
- If no startup target is provided, then wait for 'm' to choose a robot.
- Publish active robot on:
    - /active_robot
    - /teleop/active_robot  (used by fpv_camera_mux)

Tracked robot turning:
- '4' rotates LEFT  (left track backward, right track forward)  -> angular.z positive
- '6' rotates RIGHT (left track forward, right track backward)  -> angular.z negative

Circle turns (smallest radius; ONE TRACK ONLY):
- 7 forward-left  : RIGHT track forward, LEFT track stopped
- 9 forward-right : LEFT track forward, RIGHT track stopped
- 1 backward-left : RIGHT track backward, LEFT track stopped
- 3 backward-right: LEFT track backward, RIGHT track stopped

Also: swap 1 and 3 compared to prior diagonal behavior (done by defining them explicitly here).
"""

import sys
import select
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def get_key(settings):
    """Read a single keypress (with arrow-key escape sequence support)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # start of escape sequence
            key += sys.stdin.read(2)
        return key
    return ''


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__('robot_legion_teleop_python')

        # Multi-robot-first parameters
        self.declare_parameter('startup_robot', '')               # e.g. "emiliobot"
        self.declare_parameter('startup_cmd_vel_topic', '')       # e.g. "/emiliobot/cmd_vel" (optional override)

        # Safety: if False, 'm' will NOT change cmd_vel topic (real robot mode).
        self.declare_parameter('allow_cmd_vel_switching', True)
        self.allow_cmd_vel_switching = bool(self.get_parameter('allow_cmd_vel_switching').value)

        # Match motor_driver_node default wheel_separation (used for circle-turn math)
        self.declare_parameter('wheel_separation', 0.18)  # meters between tracks
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)

        # Active robot publishers
        self.active_robot_pub = self.create_publisher(String, '/active_robot', 10)
        self.active_robot_teleop_pub = self.create_publisher(String, '/teleop/active_robot', 10)

        # Teleop publisher (created once we have a target)
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None

        # Base speed profile (SLOW)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        # Derived profiles
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Last commanded direction (for republish robustness)
        # For normal twist keys, these store multipliers.
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0

        # For circle-turn keys (one-track-only), we store the actual Twist values to republish.
        self.last_twist = Twist()

        self.is_moving = False

        # KEYMAP (normal twist keys only)
        # NOTE: 7/9/1/3 are handled specially for one-track-only circles.
        self.move_bindings = {
            # Arrow keys
            '\x1b[A': ('twist', 1, 0),     # up
            '\x1b[B': ('twist', -1, 0),    # down
            '\x1b[D': ('twist', 0, 1),     # left  (rotate left)
            '\x1b[C': ('twist', 0, -1),    # right (rotate right)

            # Numpad
            '8': ('twist', 1, 0),          # forward
            '2': ('twist', -1, 0),         # backward

            # Rotate in place
            '4': ('twist', 0, 1),          # rotate LEFT
            '6': ('twist', 0, -1),         # rotate RIGHT

            # One-track-only circle turns (handled specially)
            '7': ('circle', None, None),   # forward-left  (right track only)
            '9': ('circle', None, None),   # forward-right (left track only)
            '1': ('circle', None, None),   # backward-left (right track only)
            '3': ('circle', None, None),   # backward-right(left track only)

            # Letter diagonals (kept as-is, but they will still do blended twist)
            'a': ('twist', 1, 1),
            'd': ('twist', 1, -1),
            '<': ('twist', -1, -1),
            'c': ('twist', 1, -1),
        }

        self.speed_bindings = {
            'w': self._increase_both_speeds,
            '+': self._increase_both_speeds,
            'e': self._decrease_both_speeds,
            '-': self._decrease_both_speeds,
            'q': self._increase_linear_speed,
            '/': self._increase_linear_speed,
            'r': self._decrease_linear_speed,
            '*': self._decrease_linear_speed,
            'i': self._set_slow_profile,
            'o': self._set_medium_profile,
            'p': self._set_fast_profile,
        }

        # Republish timer for robustness across Wi-Fi/DDS.
        self.create_timer(0.1, self._republish_last_twist)

        # Apply startup target (if any)
        startup_robot = self.get_parameter('startup_robot').get_parameter_value().string_value.strip()
        startup_topic = self.get_parameter('startup_cmd_vel_topic').get_parameter_value().string_value.strip()

        if startup_topic:
            self._apply_cmd_vel_topic(startup_topic, announce_startup=True)
        elif startup_robot:
            self._apply_cmd_vel_topic(f'/{startup_robot}/cmd_vel', announce_startup=True)
        else:
            self._print_instructions(None)
            print("[STARTUP] No startup robot set. Press 'm' to choose a robot.")

    # ---------------- Printing ----------------

    def _print_instructions(self, topic_name: Optional[str]):
        print("--------------------------------------------------")
        print(" Robot Legion Teleop (Python) with FPV support")
        print("--------------------------------------------------")
        if topic_name:
            print(f"Publishing Twist on: {topic_name}")
        else:
            print("Publishing Twist on: (none yet) - press 'm' to select a robot")
        print(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        print(f"wheel_separation (for circle turns): {self.wheel_separation:.3f} m")
        if self.current_robot_name:
            print(f"Active robot: {self.current_robot_name}")
        print("")
        print("Movement:")
        print("  8 forward, 2 backward")
        print("  4 rotate-left (in-place), 6 rotate-right (in-place)")
        print("  7 forward-left circle  (RIGHT track only)")
        print("  9 forward-right circle (LEFT  track only)")
        print("  1 backward-left circle (RIGHT track only)")
        print("  3 backward-right circle(LEFT  track only)")
        print("  Arrow keys also work")
        print("")
        print("Stop:")
        print("  [SPACE], 's', or 5")
        print("")
        print("Speed profiles:")
        print("  i slow, o medium, p fast")
        print("")
        print("Speed scaling:")
        print("  w/+ increase both, e/- decrease both")
        print("  q// increase linear, r/* decrease linear")
        print("")
        print("Robot selection:")
        print("  m choose robot (namespaced /<robot>/cmd_vel)")
        print("")
        print("CTRL-C to quit.")
        print("--------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        print(f"Linear Speed: {self.linear_speed:.3f}  Angular Speed: {self.angular_speed:.3f}")

    # ---------------- Active robot helpers ----------------

    def _extract_robot_name_from_topic(self, topic: str) -> Optional[str]:
        if not topic:
            return None
        if not topic.startswith('/'):
            topic = '/' + topic
        parts = topic.split('/')
        if len(parts) >= 3 and parts[2] == 'cmd_vel' and parts[1]:
            return parts[1]
        if 'cmd_vel' in parts:
            idx = parts.index('cmd_vel')
            if idx > 1 and parts[idx - 1]:
                return parts[idx - 1]
        return None

    def _publish_active_robot(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

    # ---------------- Topic application ----------------

    def _apply_cmd_vel_topic(self, new_topic: str, announce_startup: bool = False):
        if not new_topic.startswith('/'):
            new_topic = '/' + new_topic

        if new_topic == '/cmd_vel':
            print("[ERROR] Refusing to use /cmd_vel. Use /<robot_name>/cmd_vel for multi-robot architecture.")
            return

        new_robot_name = self._extract_robot_name_from_topic(new_topic)
        if new_robot_name:
            self.current_robot_name = new_robot_name
            self._publish_active_robot()

        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        if announce_startup:
            self._print_instructions(new_topic)
            print(f"[STARTUP] Target locked to: {new_topic}")
        else:
            print(f"[ROBOT SWITCH] Now publishing Twist to: {new_topic}")
            if self.current_robot_name:
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

        # Store for republish
        self.last_twist = twist
        self.is_moving = True

        self.publisher_.publish(twist)

    # ---------------- Republish (robust driving) ----------------

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
            return

        # If last_twist is non-zero, republish it (covers circle-turn mode).
        if abs(self.last_twist.linear.x) > 1e-9 or abs(self.last_twist.angular.z) > 1e-9:
            self.publisher_.publish(self.last_twist)
            return

        # Otherwise fall back to multiplier-based twist
        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return

        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self.publisher_.publish(twist)

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

    # ---------------- Robot switching ----------------

    def _switch_robot_prompt(self, settings):
        restore_terminal_settings(settings)
        try:
            print("\n[ROBOT SWITCH] Enter robot name OR full /<robot>/cmd_vel topic.")
            print("  Example name: emiliobot")
            print("  Example topic: /emiliobot/cmd_vel")
            user_input = input("[ROBOT SWITCH] Target: ").strip()
        except Exception:
            tty.setraw(sys.stdin.fileno())
            return
        tty.setraw(sys.stdin.fileno())

        if not user_input:
            return

        if user_input.startswith('/') or '/' in user_input:
            new_topic = user_input
        else:
            new_topic = f'/{user_input}/cmd_vel'

        inferred_robot = self._extract_robot_name_from_topic(new_topic)
        if inferred_robot:
            self.current_robot_name = inferred_robot
            self._publish_active_robot()
            print(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

        if not self.allow_cmd_vel_switching:
            print("[ROBOT SWITCH] cmd_vel switching disabled (allow_cmd_vel_switching:=False).")
            print(f"              Staying on: {self.cmd_vel_topic}")
            return

        self._apply_cmd_vel_topic(new_topic)

    # ---------------- Main loop ----------------

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                if key == '':
                    continue
                if key == '\x03':  # Ctrl-C
                    break

                if self.publisher_ is None:
                    if key == 'm':
                        self._switch_robot_prompt(settings)
                    elif key in self.speed_bindings:
                        self.speed_bindings[key]()
                    elif key in (' ', '5', 's'):
                        print("[STOP] (no robot selected yet)")
                    else:
                        print("[INFO] No robot selected yet. Press 'm' to choose a robot.")
                    continue

                if key in self.move_bindings:
                    mode, lin_mult, ang_mult = self.move_bindings[key]

                    if mode == 'circle':
                        S = self.linear_speed

                        if key == '7':
                            # forward-left circle: RIGHT forward, LEFT stopped
                            self._publish_one_track_circle(v_left=0.0, v_right=+S)
                        elif key == '9':
                            # forward-right circle: LEFT forward, RIGHT stopped
                            self._publish_one_track_circle(v_left=+S, v_right=0.0)
                        elif key == '1':
                            # backward-left circle: RIGHT backward, LEFT stopped
                            self._publish_one_track_circle(v_left=0.0, v_right=-S)
                        elif key == '3':
                            # backward-right circle: LEFT backward, RIGHT stopped
                            self._publish_one_track_circle(v_left=-S, v_right=0.0)
                        continue

                    # Normal twist behavior
                    self.last_lin_mult = float(lin_mult)
                    self.last_ang_mult = float(ang_mult)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult
                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

                elif key in (' ', '5', 's'):
                    self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    print("[STOP]")

                elif key == 'm':
                    self._switch_robot_prompt(settings)

                elif key in self.speed_bindings:
                    self.speed_bindings[key]()

        finally:
            if self.publisher_ is not None:
                self.publisher_.publish(Twist())
            restore_terminal_settings(settings)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()