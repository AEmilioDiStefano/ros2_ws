#!/usr/bin/env python3
"""
drive_types.py

Minimal drive-type interface to keep motor_driver_node clean and extensible.
New drive modes (e.g., underwater/flight) can be added by registering a
DriveType implementation without touching motor_driver_node.py.

⭐ Relationship to robot_profiles.yaml:
  - robot_profiles.yaml selects drive_profile + hardware_profile per robot
  - drive_types.py decides how a Twist is mixed for a given drive_profile.type

How to add a new drive type (tiny README):
  1) Create a class that implements DriveType.mix(...)
     class HoverDrive(DriveType):
         name = "hover"
         def mix(self, msg, params): ...
  2) Return it from get_drive_type() when the name matches
     if name in ("hover", "flight"): return HoverDrive()
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

from geometry_msgs.msg import Twist


@dataclass
class DriveCommand:
    """Normalized command container for drive types."""
    left: Optional[float] = None
    right: Optional[float] = None
    fl: Optional[float] = None
    fr: Optional[float] = None
    rl: Optional[float] = None
    rr: Optional[float] = None


class DriveType:
    """Interface for drive-type mixers."""
    name: str = "base"

    def mix(self, msg: Twist, params: Dict[str, float]) -> DriveCommand:
        raise NotImplementedError


class DiffDrive(DriveType):
    name = "diff_drive"

    def mix(self, msg: Twist, params: Dict[str, float]) -> DriveCommand:
        max_lin = float(params.get("max_linear_speed", 0.4))
        max_ang = float(params.get("max_angular_speed", 2.0))
        wheel_sep = float(params.get("wheel_separation", 0.18))
        spin_mult = float(params.get("spin_speed_mult", 0.7))

        v = max(-max_lin, min(max_lin, float(msg.linear.x)))
        w = max(-max_ang, min(max_ang, float(msg.angular.z)))

        # Tank-spin override
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            spin_speed = spin_mult * max_lin
            direction = 1.0 if w > 0.0 else -1.0
            v_left = -direction * spin_speed
            v_right = +direction * spin_speed
            return DriveCommand(left=v_left, right=v_right)

        v_left = v - (w * wheel_sep / 2.0)
        v_right = v + (w * wheel_sep / 2.0)
        return DriveCommand(left=v_left, right=v_right)


class MecanumDrive(DriveType):
    name = "mecanum"

    def mix(self, msg: Twist, params: Dict[str, float]) -> DriveCommand:
        max_lin = float(params.get("max_linear_speed", 0.4))
        max_ang = float(params.get("max_angular_speed", 2.0))
        wheel_base = float(params.get("wheel_base", 0.18))
        track_width = float(params.get("track_width", 0.18))

        vx = max(-max_lin, min(max_lin, float(msg.linear.x)))
        vy = max(-max_lin, min(max_lin, float(msg.linear.y)))
        w = max(-max_ang, min(max_ang, float(msg.angular.z)))

        k = (wheel_base + track_width) / 2.0 if (wheel_base + track_width) > 1e-6 else 0.18

        fl = vx - vy - w * k
        fr = vx + vy + w * k
        rl = vx + vy - w * k
        rr = vx - vy + w * k

        max_w = max(abs(fl), abs(fr), abs(rl), abs(rr), 1e-6)
        if max_w > max_lin:
            scale = max_lin / max_w
            fl *= scale
            fr *= scale
            rl *= scale
            rr *= scale

        return DriveCommand(fl=fl, fr=fr, rl=rl, rr=rr)


def get_drive_type(name: str) -> DriveType:
    name = (name or "diff_drive").lower()
    if name in ("mecanum", "omni", "omnidirectional"):
        return MecanumDrive()
    return DiffDrive()
