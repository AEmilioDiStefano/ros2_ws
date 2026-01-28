#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
Universal unit executor (Action server)

Implements:
  /<robot_name>/execute_playbook   ExecutePlaybook.action

Publishes:
  /<robot_name>/cmd_vel            geometry_msgs/Twist

Demo autonomy: timed Twist behaviors + (best-effort) feedback.
Supports:
- diff_drive
- mecanum
"""

import json
import getpass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist

from fleet_orchestrator_interfaces.action import ExecutePlaybook
from .playbook_helpers import TimedTwistPlan, run_timed_twist
from .drive_profiles import load_profile_registry, resolve_robot_profile


class UnitExecutor(Node):
    def __init__(self):
        super().__init__("unit_executor")

        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("profiles_path", "")

        self.declare_parameter("default_transit_speed_mps", 0.40)
        self.declare_parameter("default_strafe_speed_mps", 0.35)
        self.declare_parameter("default_rotate_speed_rps", 2.00)
        self.declare_parameter("default_turn_linear_mps", 0.18)

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()

        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        reg = load_profile_registry(profiles_path)
        prof = resolve_robot_profile(reg, self.robot)
        self.drive_type = prof["drive_type"]

        self.v = float(self.get_parameter("default_transit_speed_mps").value)
        self.vy = float(self.get_parameter("default_strafe_speed_mps").value)
        self.w = float(self.get_parameter("default_rotate_speed_rps").value)
        self.turn_v = float(self.get_parameter("default_turn_linear_mps").value)

        self.cmd_vel_topic = f"/{self.robot}/cmd_vel"
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.action_name = f"/{self.robot}/execute_playbook"
        self.server = ActionServer(
            self,
            ExecutePlaybook,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(f"[{self.robot}] UnitExecutor ready on {self.action_name}")
        self.get_logger().info(f"[{self.robot}] drive_type={self.drive_type} cmd_vel={self.cmd_vel_topic}")

    def goal_cb(self, goal_request):
        cid = (goal_request.command_id or "").strip()
        allowed = {"transit", "rotate", "turn", "hold", "strafe", "diagonal"}
        if cid not in allowed:
            self.get_logger().warn(f"Rejecting unknown command_id: {cid}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _publish(self, twist: Twist):
        self.cmd_pub.publish(twist)

    def _publish_feedback_safe(self, goal_handle, feedback_msg, percent: float, text: str):
        # Make feedback robust across action definition variants
        if hasattr(feedback_msg, "percent_complete"):
            feedback_msg.percent_complete = float(percent)
        if hasattr(feedback_msg, "status_text"):
            feedback_msg.status_text = str(text)
        if hasattr(feedback_msg, "status"):
            feedback_msg.status = str(text)
        if hasattr(feedback_msg, "message"):
            feedback_msg.message = str(text)

        try:
            goal_handle.publish_feedback(feedback_msg)
        except Exception:
            pass

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = ExecutePlaybook.Feedback()

        def fb(percent: float, text: str):
            self._publish_feedback_safe(goal_handle, feedback, percent, text)

        try:
            params = json.loads(goal.parameters_json or "{}")
            if not isinstance(params, dict):
                params = {}
        except Exception:
            params = {}

        command_id = (goal.command_id or "").strip()
        duration_s = float(params.get("duration_s", 1.0))
        speed = params.get("speed", None)

        v = self.v
        vy = self.vy
        w = self.w
        turn_v = self.turn_v

        try:
            if speed is not None:
                s = float(speed)
                s = max(0.0, min(1.5, s))
                v *= s
                vy *= s
                w *= s
                turn_v *= s
        except Exception:
            pass

        if command_id == "hold":
            plan = TimedTwistPlan(twist=Twist(), duration_s=duration_s, status_text="holding")
            run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True)

        elif command_id == "rotate":
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            twist.angular.z = w if direction in ("left", "ccw") else -w
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"rotating {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "transit":
            direction = str(params.get("direction", "forward")).lower()
            twist = Twist()
            twist.linear.x = -v if direction in ("backward", "back", "-x") else +v
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"transit {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "strafe":
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            if self.drive_type == "mecanum":
                twist.linear.y = +vy if direction in ("left", "+y") else -vy
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"strafe {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "diagonal":
            direction = str(params.get("direction", "fwd_left")).lower()
            twist = Twist()
            if self.drive_type == "mecanum":
                if direction == "fwd_left":
                    twist.linear.x, twist.linear.y = +v, +vy
                elif direction == "fwd_right":
                    twist.linear.x, twist.linear.y = +v, -vy
                elif direction == "back_left":
                    twist.linear.x, twist.linear.y = -v, +vy
                else:
                    twist.linear.x, twist.linear.y = -v, -vy
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"diagonal {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "turn":
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            if self.drive_type == "diff_drive":
                twist.linear.x = turn_v
                twist.angular.z = w if direction in ("left", "ccw") else -w
                status = f"turning {direction}"
            else:
                twist.angular.z = w if direction in ("left", "ccw") else -w
                status = f"rotating {direction}"
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=status)
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        goal_handle.succeed()
        result = ExecutePlaybook.Result()
        result.success = True
        result.result_text = f"Executed {command_id}"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = UnitExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
