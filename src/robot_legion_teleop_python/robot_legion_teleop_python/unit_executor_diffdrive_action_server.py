#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
Diff-drive unit executor (Action server)

Implements:
  /<robot_name>/execute_playbook   ExecutePlaybook.action

Publishes:
  /<robot_name>/cmd_vel            geometry_msgs/Twist

This is a demo "autonomy stand-in":
- It does not plan around obstacles.
- It does not do localization.
- It just executes a small playbook safely and reports feedback.
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

        self.get_logger().info(f"[{self.robot}] DiffDriveUnitExecutor ready on {self.action_name}")
        self.get_logger().info(f"[{self.robot}] Publishing cmd_vel on {self.cmd_vel_topic}")

    def goal_cb(self, goal_request):
        # Accept only commands we understand (translation barrier)
        cid = (goal_request.command_id or "").strip()
        allowed = {"transit", "rotate", "hold"}
        if cid not in allowed:
            self.get_logger().warn(f"Rejecting unknown command_id: {cid}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        # This demo executor doesn't support preemption cleanly yet; accept cancel to stop
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

        params = {}
        try:
            params = json.loads(goal.parameters_json or "{}")
        except Exception:
            params = {}

        command_id = (goal.command_id or "").strip()

        # Simple demo mapping:
        # transit: direction + duration
        # rotate: direction + duration
        # hold: duration
        duration_s = float(params.get("duration_s", 2.0))

        if command_id == "hold":
            twist = Twist()
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text="holding")
            run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True)

        elif command_id == "rotate":
            direction = str(params.get("direction", "left")).lower()
            twist = Twist()
            twist.angular.z = self.w if direction in ("left", "ccw") else -self.w
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"rotating {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        elif command_id == "transit":
            direction = str(params.get("direction", "forward")).lower()
            twist = Twist()
            # Diff-drive: forward/back = linear.x; left/right handled as rotate for demo
            if direction in ("forward", "ahead", "+x"):
                twist.linear.x = +self.v
            elif direction in ("backward", "back", "-x"):
                twist.linear.x = -self.v
            elif direction in ("left", "-y", "+y", "right"):
                # Keep demo honest: diff-drive cannot translate sideways.
                # Interpret as rotate+transit (still "autonomy stand-in" behavior).
                # left => rotate left then forward; right => rotate right then forward
                rotate_dir = "left" if direction == "left" else "right"
                tw_rot = Twist()
                tw_rot.angular.z = self.w if rotate_dir == "left" else -self.w
                run_timed_twist(self._publish, fb,
                                TimedTwistPlan(twist=tw_rot, duration_s=1.0, status_text=f"turning {rotate_dir}"),
                                rate_hz=20.0)
                twist = Twist()
                twist.linear.x = +self.v
            else:
                twist.linear.x = +self.v

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
