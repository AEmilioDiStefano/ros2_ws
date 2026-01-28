#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

import os
import pathlib
from typing import Any, Dict, Optional

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
    AMENT_AVAILABLE = True
except Exception:
    AMENT_AVAILABLE = False


def _default_profiles_path() -> pathlib.Path:
    """
    Locate config/robot_profiles.yaml both in:
      - installed share directory (preferred)
      - source tree fallback (when running from src)
    """
    if AMENT_AVAILABLE:
        try:
            share_dir = pathlib.Path(get_package_share_directory("robot_legion_teleop_python"))
            p = share_dir / "config" / "robot_profiles.yaml"
            if p.exists():
                return p
        except Exception:
            pass

    # Source-tree fallback: <pkg>/robot_legion_teleop_python/drive_profiles.py -> <pkg>/../config/robot_profiles.yaml
    here = pathlib.Path(__file__).resolve()
    pkg_root = here.parents[1]  # .../robot_legion_teleop_python/
    p = pkg_root / "config" / "robot_profiles.yaml"
    return p


def load_profile_registry(path: Optional[str] = None) -> Dict[str, Any]:
    p = pathlib.Path(path) if path else _default_profiles_path()
    if not p.exists():
        raise FileNotFoundError(f"robot_profiles.yaml not found at: {str(p)}")

    data = yaml.safe_load(p.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError("robot_profiles.yaml must be a YAML mapping at top-level")
    return data


def resolve_robot_profile(
    registry: Dict[str, Any],
    robot_name: str,
) -> Dict[str, Any]:
    """
    Resolve a robot's profile dict:
      - If registry['robots'][robot_name] is set, use that profile name
      - Else use registry['defaults']['profile']
    Returns the profile dict (deep-ish merged with defaults.params if present).
    """
    defaults = registry.get("defaults", {}) if isinstance(registry.get("defaults", {}), dict) else {}
    profiles = registry.get("profiles", {}) if isinstance(registry.get("profiles", {}), dict) else {}
    robots = registry.get("robots", {}) if isinstance(registry.get("robots", {}), dict) else {}

    # Priority 0: local per-machine selection via top-level 'drive_profile'.
    #
    # This deployment is optimized for heterogeneous fleet scalability:
    # - Each robot has its own copy of robot_profiles.yaml, and you set
    #   'drive_profile:' on that robot to pick the correct drive/hardware/gpio.
    # - Teleop learns drive_type from the robot's heartbeat (see heartbeat_node.py),
    #   so the pilot does not need to maintain a central mapping table.
    local_drive_profile = registry.get("drive_profile", None)
    if isinstance(local_drive_profile, str) and local_drive_profile.strip():
        profile_name = local_drive_profile.strip()
    else:
        # Priority 1: optional central mapping table `robots:`
        profile_name = robots.get(robot_name, defaults.get("profile", None))
    if not profile_name:
        # Hard fallback
        profile_name = "diff_hbridge"

    profile = profiles.get(profile_name, None)
    if not isinstance(profile, dict):
        raise ValueError(f"Profile '{profile_name}' not found in registry")

    # Normalize expected keys
    drive_type = str(profile.get("drive_type", "diff_drive")).strip()
    hardware = str(profile.get("hardware", "hbridge_2ch")).strip()
    params = profile.get("params", {}) if isinstance(profile.get("params", {}), dict) else {}
    gpio = profile.get("gpio", {}) if isinstance(profile.get("gpio", {}), dict) else {}
    cmd_tmpl = str(profile.get("cmd_vel_topic_template", "/{robot}/cmd_vel")).strip()

    return {
        "profile_name": profile_name,
        "drive_type": drive_type,
        "hardware": hardware,
        "params": params,
        "gpio": gpio,
        "cmd_vel_topic_template": cmd_tmpl,
    }