#!/usr/bin/env python3
"""
control_lock_manager.py

Central per-robot control lock arbiter (runs on the laptop).

Protocol (all std_msgs/String JSON):

Request topic:
  /control_lock/request

Request JSON:
  {
    "action": "acquire" | "release" | "heartbeat",
    "robot": "<robot_name>",
    "client_id": "<unique_client_id>"
  }

Response topic:
  /control_lock/response/<client_id>

Response JSON:
  {
    "ok": true|false,
    "action": "...",
    "robot": "...",
    "owner": "<current_owner_or_empty>",
    "reason": "<optional>"
  }

State broadcast:
  /control_lock/state
State JSON:
  {
    "locks": { "<robot>": "<owner_client_id>", ... }
  }

Locks expire if no heartbeat is received for lease_sec seconds.
"""

import json
import time
import os
from typing import Dict, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audit_logger import AuditLogger


def now_s() -> float:
    return time.time()


class ControlLockManager(Node):
    def __init__(self):
        super().__init__("control_lock_manager")

        # Initialize audit logger
        self.audit = AuditLogger(self, "control_lock_manager", log_file_path="/tmp/control_lock_audit.jsonl")

        self.declare_parameter("lease_sec", 5.0)
        self.lease_sec = float(self.get_parameter("lease_sec").value)

        # robot -> (owner_client_id, last_seen_timestamp)
        self._locks: Dict[str, Tuple[str, float]] = {}

        self.req_sub = self.create_subscription(String, "/control_lock/request", self._on_request, 50)
        self.state_pub = self.create_publisher(String, "/control_lock/state", 10)

        # publish state periodically
        self.create_timer(0.5, self._publish_state)
        self.create_timer(1.0, self._expire_old)

        self.get_logger().info(f"ControlLockManager started (lease_sec={self.lease_sec}).")

    def _expire_old(self):
        t = now_s()
        expired = []
        for robot, (owner, last) in list(self._locks.items()):
            if (t - last) > self.lease_sec:
                expired.append(robot)
        for robot in expired:
            self.get_logger().warn(f"Lock expired for robot '{robot}' (owner='{self._locks[robot][0]}').")
            self._locks.pop(robot, None)

    def _publish_state(self):
        self._expire_old()
        msg = String()
        msg.data = json.dumps({"locks": {r: owner for r, (owner, _ts) in self._locks.items()}})
        self.state_pub.publish(msg)

    def _get_owner(self, robot: str) -> Optional[str]:
        self._expire_old()
        cur = self._locks.get(robot)
        return cur[0] if cur else None

    def _respond(self, client_id: str, payload: Dict):
        topic = f"/control_lock/response/{client_id}"
        pub = self.create_publisher(String, topic, 10)
        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)

    def _on_request(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        action = (data.get("action") or "").strip()
        robot = (data.get("robot") or "").strip()
        client_id = (data.get("client_id") or "").strip()

        if not action or not robot or not client_id:
            return

        owner = self._get_owner(robot)

        if action == "acquire":
            if owner is None:
                self._locks[robot] = (client_id, now_s())
                self.audit.log_command(
                    robot=robot,
                    source="control_lock_manager",
                    command_id=f"acquire_{client_id}",
                    parameters={"client_id": client_id},
                    status="granted",
                )
                self._respond(client_id, {"ok": True, "action": "acquire", "robot": robot, "owner": client_id})
                return

            if owner == client_id:
                # re-acquire refresh
                self._locks[robot] = (client_id, now_s())
                self.audit.log_command(
                    robot=robot,
                    source="control_lock_manager",
                    command_id=f"acquire_{client_id}",
                    parameters={"client_id": client_id},
                    status="refreshed",
                )
                self._respond(client_id, {"ok": True, "action": "acquire", "robot": robot, "owner": client_id})
                return

            self.audit.log_command(
                robot=robot,
                source="control_lock_manager",
                command_id=f"acquire_{client_id}",
                parameters={"client_id": client_id, "owner": owner},
                status="denied",
                details=f"Robot locked by {owner}",
            )
            self._respond(client_id, {"ok": False, "action": "acquire", "robot": robot, "owner": owner, "reason": "locked"})
            return

        if action == "release":
            if owner == client_id:
                self._locks.pop(robot, None)
                self.audit.log_command(
                    robot=robot,
                    source="control_lock_manager",
                    command_id=f"release_{client_id}",
                    parameters={"client_id": client_id},
                    status="released",
                )
                self._respond(client_id, {"ok": True, "action": "release", "robot": robot, "owner": ""})
                return
            self.audit.log_command(
                robot=robot,
                source="control_lock_manager",
                command_id=f"release_{client_id}",
                parameters={"client_id": client_id, "owner": owner or ""},
                status="denied",
                details="Not the lock owner",
            )
            self._respond(client_id, {"ok": False, "action": "release", "robot": robot, "owner": owner or "", "reason": "not_owner"})
            return

        if action == "heartbeat":
            if owner == client_id:
                self._locks[robot] = (client_id, now_s())
                # heartbeat response optional (kept quiet)
                return
            # ignore heartbeats from non-owner
            return

    def destroy_node(self) -> None:
        """Clean up resources and close audit log."""
        if hasattr(self, 'audit') and self.audit:
            self.audit.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlLockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
