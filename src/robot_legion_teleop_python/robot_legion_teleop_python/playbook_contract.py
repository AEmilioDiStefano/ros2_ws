#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
playbook_contract.py

WHY THIS FILE EXISTS
--------------------
We want robot_legion_teleop_python to be a stable "execution substrate" for:
  - keyboard teleop
  - fleet_orchestrator (playbook sequencing)
  - an LLM layer that maps voice/text intent -> ordered playbooks

To make that scalable, we need ONE authoritative definition of:
  - which playbook command_ids exist
  - what parameters each expects (JSON keys)
  - what defaults are assumed when keys are missing
  - what is legal vs rejected

This is intentionally small and boring: it is a contract boundary.

IMPORTANT DESIGN CHOICE
-----------------------
We keep the command set *minimal* and *composable*.

Example:
  Voice: "move left 2 seconds and rotate 5 seconds"
  LLM -> ordered playbooks:
     1) strafe(left, 2s)
     2) rotate(left, 5s)

This matches DIU's public framing of converting natural language intent into
machine execution across heterogeneous platforms. :contentReference[oaicite:11]{index=11}
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Set, Tuple
import json


# ---------------------------------------------------------------------
# The canonical set of playbook primitives your robots agree to support.
# ---------------------------------------------------------------------
ALLOWED_COMMAND_IDS: Set[str] = {
    # "do nothing but keep control alive"
    "hold",

    # simple 1-axis motions
    "transit",   # forward/back along robot X
    "rotate",    # yaw left/right

    # drive-type-dependent motions (no-op on diff)
    "strafe",    # left/right along robot Y (only meaningful for mecanum)
    "diagonal",  # 4 diagonals (only meaningful for mecanum)

    # "turn" is an intentionally fuzzy primitive:
    # - on diff: forward + yaw (a curved path)
    # - on mecanum: treat as rotate (keeps it safe + predictable)
    "turn",
}


# ---------------------------------------------------------------------
# JSON keys we expect in ExecutePlaybook.parameters_json
# ---------------------------------------------------------------------
# Note: We keep this small. The orchestrator/LLM can enrich later.
#
# duration_s: float seconds
# speed: scalar multiplier (0..1.5)
# direction: string (depends on command)
#
# Examples:
#   {"direction":"forward","duration_s":2.0,"speed":1.0}
#   {"direction":"left","duration_s":5.0}
#
COMMON_KEYS = {"duration_s", "speed", "direction"}


@dataclass(frozen=True)
class ParsedPlaybook:
    """
    A normalized, validated representation of a playbook request.

    We parse JSON once, apply defaults, clamp values, and hand a clean object
    to the executor logic.
    """
    command_id: str
    duration_s: float
    speed_scale: float
    direction: str
    raw_params: Dict[str, Any]


def _safe_json_dict(s: str) -> Dict[str, Any]:
    """
    Parse JSON and guarantee we return a dict (or empty dict on error).

    IMPORTANT:
    - We never throw from this function.
    - Orchestrator inputs are external inputs.
    """
    try:
        data = json.loads(s or "{}")
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def validate_and_normalize(command_id: str, parameters_json: Optional[str]) -> Tuple[bool, str, Optional[ParsedPlaybook]]:
    """
    Validate the request and normalize into ParsedPlaybook.

    Returns:
      (ok, error_text, parsed_or_none)
    """
    cid = (command_id or "").strip()

    # Reject unknown command IDs: this keeps the contract stable.
    if cid not in ALLOWED_COMMAND_IDS:
        return False, f"Unknown command_id '{cid}'", None

    params = _safe_json_dict(parameters_json or "{}")

    # duration_s default: 1 second (safe and obvious for demos)
    try:
        duration_s = float(params.get("duration_s", 1.0))
    except Exception:
        duration_s = 1.0
    duration_s = max(0.0, min(60.0, duration_s))  # clamp to prevent runaway

    # speed scaling default: 1.0 (identity)
    try:
        speed_scale = float(params.get("speed", 1.0))
    except Exception:
        speed_scale = 1.0
    speed_scale = max(0.0, min(1.5, speed_scale))

    # direction default: depends on command, but we keep it simple:
    # - transit: forward
    # - rotate/turn: left
    # - strafe: left
    # - diagonal: fwd_left
    # - hold: none
    default_dir = "none"
    if cid == "transit":
        default_dir = "forward"
    elif cid in ("rotate", "turn"):
        default_dir = "left"
    elif cid == "strafe":
        default_dir = "left"
    elif cid == "diagonal":
        default_dir = "fwd_left"

    direction = str(params.get("direction", default_dir)).strip().lower() or default_dir

    # Minimal sanity checks for direction values.
    # We do NOT over-restrict: orchestrator/LLM is allowed to experiment.
    # But we keep obvious typos from becoming silent weirdness.
    if cid == "transit" and direction not in ("forward", "backward", "back", "-x", "+x"):
        return False, f"Invalid direction '{direction}' for command '{cid}'", None
    if cid in ("rotate", "turn") and direction not in ("left", "right", "ccw", "cw"):
        return False, f"Invalid direction '{direction}' for command '{cid}'", None
    if cid == "strafe" and direction not in ("left", "right", "+y", "-y"):
        return False, f"Invalid direction '{direction}' for command '{cid}'", None
    if cid == "diagonal" and direction not in ("fwd_left", "fwd_right", "back_left", "back_right"):
        return False, f"Invalid direction '{direction}' for command '{cid}'", None

    return True, "", ParsedPlaybook(
        command_id=cid,
        duration_s=duration_s,
        speed_scale=speed_scale,
        direction=direction,
        raw_params=params,
    )
