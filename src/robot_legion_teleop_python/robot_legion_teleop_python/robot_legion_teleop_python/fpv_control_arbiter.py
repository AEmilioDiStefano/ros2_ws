#!/usr/bin/env python3
"""
fpv_control_arbiter.py

ROS-native exclusive-control + status node for browser FPV control.

Browser talks to ROS 2 via rosbridge_server (websocket JSON).
Video is served by web_video_server (MJPEG).

Topics (std_msgs/String JSON):
  /fpv/claim_req      {"client_id","display_name","robot"}
  /fpv/release_req    {"client_id","robot"}
  /fpv/heartbeat      {"client_id","robot"}  (refreshes claim TTL)
  /fpv/cmd_req        {"client_id","robot","linear","angular"}  (clamped)

Publishes:
  /fpv/status         JSON status blob for UI polling/subscription

Why JSON over custom msgs?
- Works trivially with rosbridge + roslibjs with no extra message generation.
"""

import json
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


CMD_VEL_TOPIC = "/{robot}/cmd_vel"
CAMERA_TOPIC = "/{robot}/camera/image_raw"


@dataclass
class Claim:
    client_id: str
    display_name: str
    expires_at: float  # monotonic time


class FpvControlArbiter(Node):
    def __init__(self):
        super().__init__("fpv_control_arbiter")

        # ---- Parameters
        self.declare_parameter("lease_ttl_sec", 3.0)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 2.0)

        self.lease_ttl = float(self.get_parameter("lease_ttl_sec").value)
        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.max_ang = float(self.get_parameter("max_angular_speed").value)

        # ---- State
        self._known_robots: Set[str] = set()
        self._claims: Dict[str, Claim] = {}  # robot -> claim
        self._cmd_vel_pubs: Dict[str, rclpy.publisher.Publisher] = {}

        # ---- Pub/Sub
        self.status_pub = self.create_publisher(String, "/fpv/status", 10)

        self.create_subscription(String, "/fpv/claim_req", self._on_claim_req, 10)
        self.create_subscription(String, "/fpv/release_req", self._on_release_req, 10)
        self.create_subscription(String, "/fpv/heartbeat", self._on_heartbeat, 10)
        self.create_subscription(String, "/fpv/cmd_req", self._on_cmd_req, 10)

        # ---- Timers
        self.create_timer(1.0, self._refresh_discovery)
        self.create_timer(0.25, self._expire_claims)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info("FPV Control Arbiter started (ROS-native).")

    # ---------------- Utilities ----------------

    def _now(self) -> float:
        return time.monotonic()

    def _safe_json(self, msg: String) -> Optional[dict]:
        try:
            return json.loads(msg.data)
        except Exception:
            return None

    def _get_or_make_cmd_vel_pub(self, robot: str):
        if robot in self._cmd_vel_pubs:
            return self._cmd_vel_pubs[robot]
        topic = CMD_VEL_TOPIC.format(robot=robot)
        pub = self.create_publisher(Twist, topic, 10)
        self._cmd_vel_pubs[robot] = pub
        return pub

    def _is_controllable(self, robot: str) -> bool:
        # "controllable" means motor driver node is subscribed to /<robot>/cmd_vel
        topic = CMD_VEL_TOPIC.format(robot=robot)
        subs = self.get_subscriptions_info_by_topic(topic)
        return bool(subs and len(subs) > 0)

    def _has_video(self, robot: str) -> bool:
        topic = CAMERA_TOPIC.format(robot=robot)
        pubs = self.get_publishers_info_by_topic(topic)
        return bool(pubs and len(pubs) > 0)

    # ---------------- Discovery ----------------

    def _refresh_discovery(self):
        # We infer robots by scanning topics and matching /<robot>/(cmd_vel|camera/image_raw)
        topics = self.get_topic_names_and_types()
        robots: Set[str] = set()

        for name, _types in topics:
            if name.endswith("/cmd_vel"):
                parts = name.split("/")
                if len(parts) >= 3 and parts[1]:
                    robots.add(parts[1])
            if name.endswith("/camera/image_raw"):
                parts = name.split("/")
                if len(parts) >= 5 and parts[1]:
                    robots.add(parts[1])

        if robots != self._known_robots:
            self._known_robots = robots

    # ---------------- Claim lifecycle ----------------

    def _expire_claims(self):
        now = self._now()
        expired = [r for r, c in self._claims.items() if c.expires_at < now]
        for r in expired:
            del self._claims[r]

    def _claim(self, robot: str, client_id: str, display_name: str) -> (bool, str):
        if robot not in self._known_robots:
            return False, "unknown_robot"
        if not self._is_controllable(robot):
            return False, "not_controllable"

        now = self._now()
        existing = self._claims.get(robot)
        if existing and existing.expires_at >= now and existing.client_id != client_id:
            return False, "already_claimed"

        self._claims[robot] = Claim(
            client_id=client_id,
            display_name=display_name,
            expires_at=now + self.lease_ttl
        )
        return True, "ok"

    # ---------------- Handlers ----------------

    def _on_claim_req(self, msg: String):
        d = self._safe_json(msg)
        if not d:
            return
        robot = str(d.get("robot", "")).strip()
        client_id = str(d.get("client_id", "")).strip()
        display_name = str(d.get("display_name", "user")).strip()

        if not robot or not client_id:
            return

        ok, reason = self._claim(robot, client_id, display_name)
        # push status quickly so UI updates
        if ok:
            self.get_logger().info(f"[CLAIM] {robot} -> {display_name} ({client_id})")
        self._publish_status()

    def _on_release_req(self, msg: String):
        d = self._safe_json(msg)
        if not d:
            return
        robot = str(d.get("robot", "")).strip()
        client_id = str(d.get("client_id", "")).strip()
        if not robot or not client_id:
            return
        existing = self._claims.get(robot)
        if existing and existing.client_id == client_id:
            del self._claims[robot]
            self.get_logger().info(f"[RELEASE] {robot} by {client_id}")
            self._publish_status()

    def _on_heartbeat(self, msg: String):
        d = self._safe_json(msg)
        if not d:
            return
        robot = str(d.get("robot", "")).strip()
        client_id = str(d.get("client_id", "")).strip()
        if not robot or not client_id:
            return
        existing = self._claims.get(robot)
        if existing and existing.client_id == client_id:
            existing.expires_at = self._now() + self.lease_ttl

    def _on_cmd_req(self, msg: String):
        d = self._safe_json(msg)
        if not d:
            return

        robot = str(d.get("robot", "")).strip()
        client_id = str(d.get("client_id", "")).strip()
        if not robot or not client_id:
            return

        existing = self._claims.get(robot)
        if not existing or existing.client_id != client_id or existing.expires_at < self._now():
            # ignore commands from non-claimants
            return

        try:
            lin = float(d.get("linear", 0.0))
            ang = float(d.get("angular", 0.0))
        except Exception:
            return

        # clamp
        lin = max(-self.max_lin, min(self.max_lin, lin))
        ang = max(-self.max_ang, min(self.max_ang, ang))

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang

        pub = self._get_or_make_cmd_vel_pub(robot)
        pub.publish(twist)

    # ---------------- Status ----------------

    def _publish_status(self):
        now = self._now()
        robots: List[dict] = []

        for r in sorted(self._known_robots):
            claim = self._claims.get(r)
            claimed_by = None
            if claim and claim.expires_at >= now:
                claimed_by = {"display_name": claim.display_name, "client_id": claim.client_id}

            robots.append({
                "robot": r,
                "controllable": self._is_controllable(r),
                "has_video": self._has_video(r),
                "claimed_by": claimed_by,
            })

        payload = {
            "type": "status",
            "robots": robots,
            "lease_ttl_sec": self.lease_ttl,
        }

        out = String()
        out.data = json.dumps(payload)
        self.status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FpvControlArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
