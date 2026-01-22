#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
Diff-drive unit executor (Action server)

Implements:
  /<robot_name>/execute_playbook   ExecutePlaybook.action

Publishes:
  /<robot_name>/cmd_vel            geometry_msgs/Twist

This is a demo "autonomy stand-in":
- No planning, no localization. Just timed Twist behaviors + feedback.
"""

import json
import getpass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist

from fleet_orchestrator_interfaces.action import ExecutePlaybook
from .playbook_helpers import TimedTwistPlan, run_timed_twist


class DiffDriveUnitExecutor(Node):
    def __init__(self):
        super().__init__("unit_executor_diffdrive")

        # Must match motor_driver_node.py conventions (robot name drives topic names)
        self.declare_parameter("robot_name", getpass.getuser())

        # Match your motor_driver_node defaults for "strong" motion
        self.declare_parameter("default_transit_speed_mps", 0.40)
        self.declare_parameter("default_rotate_speed_rps", 2.00)

        # For circle-turn (one track ~0, other ~0.36 when wheel_separation=0.18 and w=2.0)
        self.declare_parameter("default_turn_linear_mps", 0.18)

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()
        self.v = float(self.get_parameter("default_transit_speed_mps").value)
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

        self.get_logger().info(f"[{self.robot}] DiffDriveUnitExecutor ready on {self.action_name}")
        self.get_logger().info(f"[{self.robot}] Publishing cmd_vel on {self.cmd_vel_topic}")

    def goal_cb(self, goal_request):
        cid = (goal_request.command_id or "").strip()
        allowed = {"transit", "rotate", "turn", "hold"}
        if cid not in allowed:
            self.get_logger().warn(f"Rejecting unknown command_id: {cid}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _publish(self, twist: Twist):
        self.cmd_pub.publish(twist)

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = ExecutePlaybook.Feedback()

        def fb(percent: float, text: str):
            feedback.percent_complete = float(percent)
            feedback.status_text = str(text)
            goal_handle.publish_feedback(feedback)

        try:
            params = json.loads(goal.parameters_json or "{}")
            if not isinstance(params, dict):
                params = {}
        except Exception:
            params = {}

        command_id = (goal.command_id or "").strip()

        duration_s = float(params.get("duration_s", 1.0))
        speed = params.get("speed", None)

        # Optional speed override for transit/rotate/turn
        v = self.v
        w = self.w
        turn_v = self.turn_v
        try:
            if speed is not None:
                s = float(speed)
                # Interpret s as a multiplier in (0..1.5] for safety.
                s = max(0.0, min(1.5, s))
                v = self.v * s
                w = self.w * s
                turn_v = self.turn_v * s
        except Exception:
            pass

        if command_id == "hold":
            twist = Twist()
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text="holding")
            run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True)

        elif command_id == "rotate":
            # Teleop 4/6 semantics: in-place spin -> motor driver tank-spin override
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = w if direction in ("left", "ccw") else -w
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"rotating {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "turn":
            # Circle turn: one track ~0, the other moving (via v + w combination)
            # With motor_driver_node wheel_separation=0.18 and w≈2.0:
            #   v=0.18, w=2.0 -> left~0, right~0.36 (left turn)
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            twist.linear.x = turn_v
            twist.angular.z = w if direction in ("left", "ccw") else -w
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"turning {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "transit":
            # Teleop 8/2 semantics: forward/back strong
            direction = str(params.get("direction", "forward")).lower()
            twist = Twist()
            if direction in ("backward", "back", "-x"):
                twist.linear.x = -v
            else:
                twist.linear.x = +v
            twist.angular.z = 0.0
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"transit {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        goal_handle.succeed()
        result = ExecutePlaybook.Result()
        result.success = True
        result.result_text = f"Executed {command_id}"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveUnitExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
