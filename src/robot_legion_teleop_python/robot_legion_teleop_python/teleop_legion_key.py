#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for multiple robots with FPV integration:

- Publishes geometry_msgs/Twist to a cmd_vel topic (per-robot).
- Lets you switch robots at runtime with 'm'.
- Publishes the currently controlled robot on:
    - /active_robot
    - /teleop/active_robot   (for fpv_camera_mux)

Keymap preserved:
  7 fwd-left, 9 fwd-right, 8 fwd, 5 stop, 4 rotate-left, 6 rotate-right,
  1 back-left, 3 back-right, 2 back, i/o/p speed profiles, etc.
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
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        return key
    return ''


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__('robot_legion_teleop_python')

        # Default cmd_vel topic (IMPORTANT for real robot: set to /emiliobot/cmd_vel)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # If False, 'm' will NOT change cmd_vel topic (safe for real robot).
        # You can still publish active robot updates for FPV.
        self.declare_parameter('allow_cmd_vel_switching', True)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.allow_cmd_vel_switching = bool(self.get_parameter('allow_cmd_vel_switching').value)

        self.cmd_vel_topic = cmd_vel_topic
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Active robot publishers (both topics)
        self.active_robot_pub = self.create_publisher(String, '/active_robot', 10)
        self.active_robot_teleop_pub = self.create_publisher(String, '/teleop/active_robot', 10)

        self.current_robot_name: Optional[str] = self._extract_robot_name_from_topic(cmd_vel_topic)
        if self.current_robot_name:
            self._publish_active_robot()

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

        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.is_moving = False

        # MOVEMENT BINDINGS (unchanged from your current file) :contentReference[oaicite:5]{index=5}
        self.move_bindings = {
            '\x1b[A': (1, 0),
            '\x1b[B': (-1, 0),
            '\x1b[D': (0, 1),
            '\x1b[C': (0, -1),

            '8': (1, 0),
            '2': (-1, 0),
            '4': (0, 1),
            '6': (0, -1),

            'a': (1, 1),
            'd': (1, -1),
            '<': (-1, -1),
            'c': (1, -1),

            '7': (1, 1),
            '9': (1, -1),
            '1': (-1, 1),
            '3': (-1, -1),
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

        self._print_instructions(cmd_vel_topic)

    def _print_instructions(self, topic_name):
        print("--------------------------------------------------")
        print(" Robot Legion Teleop (Python) with FPV support")
        print("--------------------------------------------------")
        print(f"Publishing Twist on: {topic_name}")
        print(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        if self.current_robot_name:
            print(f"Initial active robot: {self.current_robot_name}")
        print("")
        print("Movement: 8/2/4/6, 7/9/1/3, arrows also work")
        print("Stop: [SPACE], 's', or 5")
        print("Profiles: i=slow, o=medium, p=fast")
        print("Robot selection: m (only changes cmd_vel topic if allow_cmd_vel_switching=True)")
        print("CTRL-C to quit.")
        print("--------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        print(f"Linear Speed: {self.linear_speed:.3f}  Angular Speed: {self.angular_speed:.3f}")

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
        print(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

    def _republish_last_twist(self):
        if not self.is_moving:
            return
        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return
        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self.publisher_.publish(twist)

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._print_current_speeds()
        self._republish_last_twist()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._print_current_speeds()
        self._republish_last_twist()

    def _switch_robot_prompt(self, settings):
        restore_terminal_settings(settings)
        try:
            print("\n[ROBOT SWITCH] Enter robot name or full cmd_vel topic.")
            user_input = input("[ROBOT SWITCH] Target: ").strip()
        except Exception:
            tty.setraw(sys.stdin.fileno())
            return
        tty.setraw(sys.stdin.fileno())

        if not user_input:
            return

        if user_input.startswith('/') or '/' in user_input:
            new_topic = user_input
            new_robot_name = self._extract_robot_name_from_topic(new_topic)
        else:
            new_topic = f"/{user_input}/cmd_vel"
            new_robot_name = user_input

        if not new_topic.startswith('/'):
            new_topic = '/' + new_topic

        # Always publish active robot change (for FPV), even if we don't switch cmd_vel.
        self.current_robot_name = new_robot_name
        self._publish_active_robot()

        if not self.allow_cmd_vel_switching:
            print(f"[ROBOT SWITCH] cmd_vel switching disabled; staying on {self.cmd_vel_topic}")
            return

        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass
        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic
        print(f"[ROBOT SWITCH] Now publishing Twist to: {new_topic}")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                if key == '':
                    continue
                if key == '\x03':
                    break

                if key in self.move_bindings:
                    lin_mult, ang_mult = self.move_bindings[key]
                    self.last_lin_mult = lin_mult
                    self.last_ang_mult = ang_mult
                    self.is_moving = True

                    twist = Twist()
                    twist.linear.x = self.linear_speed * lin_mult
                    twist.angular.z = self.angular_speed * ang_mult
                    self.publisher_.publish(twist)

                elif key in (' ', '5', 's'):
                    self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    print("[STOP]")

                elif key == 'm':
                    self._switch_robot_prompt(settings)

                elif key in self.speed_bindings:
                    self.speed_bindings[key]()

        finally:
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
