#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
Omni (holonomic) unit executor (Action server)

Implements:
  /<robot_name>/execute_playbook   ExecutePlaybook.action

Publishes:
  /<robot_name>/cmd_vel            geometry_msgs/Twist

This assumes the omni base controller consumes:
  linear.x (forward/back), linear.y (left/right strafe), angular.z (yaw)
Typical for holonomic ROS controllers.
"""

import json
import getpass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist

from fleet_orchestrator_interfaces.action import ExecutePlaybook
from .playbook_helpers import TimedTwistPlan, run_timed_twist


class OmniUnitExecutor(Node):
    def __init__(self):
        super().__init__("unit_executor_omni")

        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("default_transit_speed_mps", 0.20)
        self.declare_parameter("default_rotate_speed_rps", 0.80)

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()
        self.v = float(self.get_parameter("default_transit_speed_mps").value)
        self.w = float(self.get_parameter("default_rotate_speed_rps").value)

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

        self.get_logger().info(f"[{self.robot}] OmniUnitExecutor ready on {self.action_name}")
        self.get_logger().info(f"[{self.robot}] Publishing cmd_vel on {self.cmd_vel_topic}")

    def goal_cb(self, goal_request):
        cid = (goal_request.command_id or "").strip()
        allowed = {"transit", "rotate", "hold"}
        return GoalResponse.ACCEPT if cid in allowed else GoalResponse.REJECT

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
        except Exception:
            params = {}

        command_id = (goal.command_id or "").strip()
        duration_s = float(params.get("duration_s", 2.0))

        if command_id == "hold":
            plan = TimedTwistPlan(twist=Twist(), duration_s=duration_s, status_text="holding")
            run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True)

        elif command_id == "rotate":
            direction = str(params.get("direction", "left")).lower()
            tw = Twist()
            tw.angular.z = self.w if direction in ("left", "ccw") else -self.w
            plan = TimedTwistPlan(twist=tw, duration_s=duration_s, status_text=f"rotating {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "transit":
            direction = str(params.get("direction", "forward")).lower()
            tw = Twist()

            # Omni can move in X and Y directly.
            if direction in ("forward", "ahead", "+x"):
                tw.linear.x = +self.v
            elif direction in ("backward", "back", "-x"):
                tw.linear.x = -self.v
            elif direction in ("left", "+y"):
                tw.linear.y = +self.v
            elif direction in ("right", "-y"):
                tw.linear.y = -self.v
            else:
                tw.linear.x = +self.v

            plan = TimedTwistPlan(twist=tw, duration_s=duration_s, status_text=f"transit {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        goal_handle.succeed()
        result = ExecutePlaybook.Result()
        result.success = True
        result.result_text = f"Executed {command_id}"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OmniUnitExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
