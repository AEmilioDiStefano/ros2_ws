#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
unit_executor_action_server.py

ROLE
----
Robot-side ActionServer:
  /<robot_name>/execute_playbook   fleet_orchestrator_interfaces/action/ExecutePlaybook

It executes playbook primitives by publishing Twist to:
  /<robot_name>/cmd_vel

WHY THIS IS THE MOST IMPORTANT "FLEET" NODE
-------------------------------------------
This is the robot-side boundary for your entire DIU-style demo pipeline:

Voice -> STT -> LLM -> ordered playbooks -> fleet_orchestrator -> Action goals -> robots

Teleop is "manual mode".
This executor is "orchestrated mode".
Both converge on cmd_vel.

That means:
- Your drive-type differences are hidden behind motor_driver_node.
- Your fleet-level logic doesn't fork per drive type.
"""

from __future__ import annotations

import getpass
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
import threading

from fleet_orchestrator_interfaces.action import ExecutePlaybook

from .drive_profiles import load_profile_registry, resolve_robot_profile
from .playbook_helpers import TimedTwistPlan, run_timed_twist
from .playbook_contract import validate_and_normalize
from .audit_logger import AuditLogger


class UnitExecutor(Node):
    def __init__(self):
        super().__init__("unit_executor")

        # Robot identity follows your convention.
        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("profiles_path", "")

        # Default speeds (baseline). Orchestrator can scale using speed= in JSON.
        self.declare_parameter("default_transit_speed_mps", 0.40)
        self.declare_parameter("default_strafe_speed_mps", 0.35)
        self.declare_parameter("default_rotate_speed_rps", 2.00)
        self.declare_parameter("default_turn_linear_mps", 0.18)

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()

        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        prof = None
        try:
            reg = load_profile_registry(profiles_path)
            prof = resolve_robot_profile(reg, self.robot)
            self.drive_type = prof["drive_type"]
        except Exception:
            # Fallback: accept explicit parameters or defaults when registry doesn't contain this robot.
            self.declare_parameter("drive_type", "diff_drive")
            self.declare_parameter("hardware", "L298N_diff")
            self.drive_type = str(self.get_parameter("drive_type").value).strip() or "diff_drive"
            self.hardware = str(self.get_parameter("hardware").value).strip() or "L298N_diff"
            self.profile_name = None
            self.get_logger().warning(f"[{self.robot}] profile not found in registry; falling back to drive_type={self.drive_type} hardware={self.hardware}")
        else:
            self.hardware = prof.get("hardware")
            self.profile_name = prof.get("profile_name")

        # Load baseline speeds
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

        # Active goals mapping: id(goal_handle) -> threading.Event
        self._active_goals = {}

        # Audit logging for DIU compliance
        audit_log_path = os.environ.get("ROBOT_AUDIT_LOG_PATH") or f"/tmp/robot_{self.robot}_audit.jsonl"
        self.audit = AuditLogger(self, "unit_executor", audit_log_path)

        self.get_logger().info(f"[{self.robot}] UnitExecutor ready")
        self.get_logger().info(f"[{self.robot}] action={self.action_name}")
        self.get_logger().info(f"[{self.robot}] drive_type={self.drive_type} cmd_vel={self.cmd_vel_topic}")
        self.get_logger().info(f"[{self.robot}] Audit log: {audit_log_path}")

    # ---------------- action callbacks ----------------
    def goal_cb(self, goal_request):
        """
        Validate early (fast reject) so orchestrator gets immediate feedback.
        """
        ok, err, _parsed = validate_and_normalize(goal_request.command_id, goal_request.parameters_json)
        intent_id = goal_request.intent_id if hasattr(goal_request, "intent_id") else "unknown"

        if not ok:
            self.get_logger().warn(f"[{self.robot}] reject goal: {err}")
            self.audit.log_command(
                robot=self.robot,
                source="orchestrator",
                command_id=goal_request.command_id,
                parameters=json.loads(goal_request.parameters_json or "{}"),
                status="rejected",
                source_id=intent_id,
                details=err,
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        """
        Accept cancels.

        Note: run_timed_twist is a blocking loop. For full cancel correctness,
        we'd refactor run_timed_twist into a timer-driven state machine.
        For Sprint-1 demos, we keep it simple and accept cancel requests.
        """
        # Signal the running executor loop (if any) to stop
        ev = self._active_goals.get(id(goal_handle))
        if ev is not None:
            try:
                ev.set()
            except Exception:
                pass
        return CancelResponse.ACCEPT

    # ---------------- helpers ----------------
    def _publish(self, twist: Twist):
        self.cmd_pub.publish(twist)

    def _publish_feedback_safe(self, goal_handle, feedback_msg, percent: float, text: str):
        """
        Your earlier logs showed an AttributeError around percent_complete.

        That happens when the action definition differs between builds.
        So we write feedback defensively:
          - if field exists, set it
          - if not, ignore

        This makes the executor resilient across interface revisions.
        """
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

    # ---------------- executor ----------------
    def execute_cb(self, goal_handle):
        """
        Execute a validated goal by publishing a timed Twist.
        """
        goal = goal_handle.request
        feedback = ExecutePlaybook.Feedback()
        intent_id = goal.intent_id if hasattr(goal, "intent_id") else "unknown"
        goal_start_time = time.time()

        ok, err, parsed = validate_and_normalize(goal.command_id, goal.parameters_json)
        if not ok or parsed is None:
            self.get_logger().warn(f"[{self.robot}] execute rejected late: {err}")
            self.audit.log_command(
                robot=self.robot,
                source="orchestrator",
                command_id=goal.command_id,
                parameters=json.loads(goal.parameters_json or "{}"),
                status="failed",
                source_id=intent_id,
                details=err,
                duration_s=time.time() - goal_start_time,
            )
            goal_handle.abort()
            result = ExecutePlaybook.Result()
            result.success = False
            result.result_text = err
            return result

        def fb(percent: float, text: str):
            self._publish_feedback_safe(goal_handle, feedback, percent, text)

        # Create a stop event so cancel requests can interrupt run_timed_twist
        stop_event = threading.Event()
        self._active_goals[id(goal_handle)] = stop_event

        # Log goal execution start
        self.audit.log_command(
            robot=self.robot,
            source="orchestrator",
            command_id=goal.command_id,
            parameters=parsed.raw_params,
            status="started",
            source_id=intent_id,
        )

        # Apply speed scaling
        v = self.v * parsed.speed_scale
        vy = self.vy * parsed.speed_scale
        w = self.w * parsed.speed_scale
        turn_v = self.turn_v * parsed.speed_scale

        cid = parsed.command_id
        direction = parsed.direction
        duration_s = parsed.duration_s

        # Implement primitives.
        if cid == "hold":
            plan = TimedTwistPlan(twist=Twist(), duration_s=duration_s, status_text="holding")
            run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True, stop_event=stop_event)

        elif cid == "rotate":
            twist = Twist()
            twist.angular.z = w if direction in ("left", "ccw") else -w
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"rotate {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=stop_event)

        elif cid == "transit":
            twist = Twist()
            twist.linear.x = -v if direction in ("backward", "back", "-x") else +v
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"transit {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=stop_event)

        elif cid == "strafe":
            # Safe behavior:
            # - mecanum: publish y velocity
            # - diff: ignore strafe (publish zero Twist unless you decide otherwise)
            twist = Twist()
            if self.drive_type == "mecanum":
                twist.linear.y = +vy if direction in ("left", "+y") else -vy
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"strafe {direction}")
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=stop_event)

        elif cid == "diagonal":
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
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=stop_event)

        elif cid == "turn":
            # "turn" is curved on diff, rotate on mecanum (safe + predictable).
            twist = Twist()
            if self.drive_type == "diff_drive":
                twist.linear.x = turn_v
                twist.angular.z = w if direction in ("left", "ccw") else -w
                status = f"turn {direction}"
            else:
                twist.angular.z = w if direction in ("left", "ccw") else -w
                status = f"rotate {direction}"
            plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=status)
            run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True)

        # Mark success
        goal_handle.succeed()
        result = ExecutePlaybook.Result()
        result.success = True
        result.result_text = "ok"
        
        # Log successful goal execution
        goal_duration_s = time.time() - goal_start_time
        self.audit.log_command(
            robot=self.robot,
            source="orchestrator",
            command_id=goal_request.command_id,
            parameters=json.loads(goal_request.parameters_json or "{}"),
            status="succeeded",
            source_id=intent_id,
            duration_s=goal_duration_s,
        )
        
        # Clean up the active goal tracking
        if intent_id in self._active_goals:
            del self._active_goals[intent_id]
        
        return result

    def destroy_node(self) -> None:
        """Clean up resources and close audit log."""
        if hasattr(self, 'audit') and self.audit:
            self.audit.close()
        super().destroy_node()


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
