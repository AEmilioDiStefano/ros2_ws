#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
"""
drive_profiles.py

This module loads a YAML registry and resolves
robot profiles. Keeping it separate makes the rest of the code (teleop, motor driver,
heartbeat) easier to read and test.

Compatibility note:
Some earlier iterations of this workspace imported a function named
`load_robot_profiles_yaml()`. Newer code uses `load_profile_registry()`.
We provide BOTH so older/newer nodes don't crash due to an import rename.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional
import sys

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
    AMENT_AVAILABLE = True
except ImportError:
    AMENT_AVAILABLE = False


def _default_profiles_path() -> Path:
    """
    Default location to find robot_profiles.yaml.

    Search order:
      1. Explicit parameter (caller provides full path)
      2. Installed via ament (colcon install): share/<package>/config/robot_profiles.yaml
      3. Source tree (development): robot_legion_teleop_python/config/robot_profiles.yaml

    In your repo, this file is typically located at:
      robot_legion_teleop_python/config/robot_profiles.yaml
    """
    # Try installed ament package location first (colcon install mode)
    if AMENT_AVAILABLE:
        try:
            pkg_share = get_package_share_directory("robot_legion_teleop_python")
            ament_path = Path(pkg_share) / "config" / "robot_profiles.yaml"
            if ament_path.exists():
                return ament_path
        except Exception:
            pass

    # Try source tree (development mode)
    source_tree_path = Path(__file__).resolve().parents[1] / "config" / "robot_profiles.yaml"
    if source_tree_path.exists():
        return source_tree_path

    # Fallback to source tree path (will fail with helpful error if not found)
    return source_tree_path


def load_profile_registry(profiles_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load the full robot profile registry from YAML.

    Args:
        profiles_path:
            Optional explicit path to robot_profiles.yaml.
            If None/empty, we use a reasonable default.

    Returns:
        A dict with keys like:
            - defaults
            - robots
            - drive_profiles
            - hardware_profiles
    """
    path = Path(profiles_path).expanduser() if profiles_path else _default_profiles_path()
    if not path.exists():
        raise FileNotFoundError(
            f"robot profile registry not found: {path}\n"
            "Fix by passing `profiles_path:=/full/path/to/robot_profiles.yaml` "
            "or ensure the file exists in robot_legion_teleop_python/config/."
        )

    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"robot profile registry YAML must be a mapping (dict). Got: {type(data)}")

    # Light validation with helpful messages for beginners
    for required in ("defaults", "robots", "drive_profiles", "hardware_profiles"):
        if required not in data:
            raise ValueError(
                f"robot profile registry missing required key '{required}'. "
                f"Keys present: {list(data.keys())}"
            )

    return data


# --- Backwards-compatible alias (older code imports this name) -----------------
def load_robot_profiles_yaml(profiles_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Backwards-compatible alias of load_profile_registry().
    """
    return load_profile_registry(profiles_path)


def resolve_robot_profile(reg: Dict[str, Any], robot_name: str) -> Dict[str, Any]:
    """
    Resolve a robot into a concrete set of configuration fields.

    This takes:
      robots[robot_name] -> {drive_profile: ..., hardware_profile: ...}
    then expands those names into actual config blocks.

    Returns a dict with:
        robot_name, drive_type, profile_name, hardware, drive, gpio, etc.
    """
    robot_name = str(robot_name).strip()
    if not robot_name:
        raise ValueError("resolve_robot_profile() requires a non-empty robot_name")

    robots = reg.get("robots", {})
    if robot_name not in robots:
        # Student-friendly error: show known robots
        known = ", ".join(sorted(robots.keys())) if robots else "<none>"
        raise KeyError(f"Unknown robot '{robot_name}'. Known robots: {known}")

    robot_entry = robots[robot_name] or {}
    drive_profile_name = robot_entry.get("drive_profile") or reg["defaults"].get("drive_profile")
    hw_profile_name = robot_entry.get("hardware_profile") or reg["defaults"].get("hardware_profile")

    if not drive_profile_name:
        raise ValueError(f"No drive_profile resolved for robot '{robot_name}' (check defaults + robots section)")
    if not hw_profile_name:
        raise ValueError(f"No hardware_profile resolved for robot '{robot_name}' (check defaults + robots section)")

    drive_profiles = reg.get("drive_profiles", {})
    hardware_profiles = reg.get("hardware_profiles", {})

    if drive_profile_name not in drive_profiles:
        raise KeyError(f"drive_profile '{drive_profile_name}' not found in drive_profiles")
    if hw_profile_name not in hardware_profiles:
        raise KeyError(f"hardware_profile '{hw_profile_name}' not found in hardware_profiles")

    drive = drive_profiles[drive_profile_name] or {}
    hw = hardware_profiles[hw_profile_name] or {}

    # Merge params with optional per-robot overrides (if present).
    drive_params = dict(drive.get("params", {}) or {})
    hw_params = dict(hw.get("params", {}) or {})

    robot_params = robot_entry.get("params") or {}
    if isinstance(robot_params, dict):
        drive_params.update(robot_params.get("drive", {}) or {})
        hw_params.update(robot_params.get("hardware", {}) or {})

    _validate_params(drive_params, hw_params, drive_profile_name, hw_profile_name)

    drive_type = drive.get("type")
    if not drive_type:
        raise ValueError(f"drive_profiles.{drive_profile_name} missing required 'type' field")

    return {
        "robot_name": robot_name,
        "profile_name": drive_profile_name,
        "hardware": hw_profile_name,
        "drive_type": drive_type,
        "drive": drive,
        "hw": hw,
        "gpio": hw.get("gpio", {}),
        "drive_params": drive_params,
        "hardware_params": hw_params,
        "robot_params": robot_params if isinstance(robot_params, dict) else {},
    }


def _validate_params(drive_params: Dict[str, Any], hw_params: Dict[str, Any], drive_name: str, hw_name: str) -> None:
    """Basic range checks for common tuning parameters."""
    def _as_float(v):
        try:
            return float(v)
        except Exception:
            return None

    def _as_int(v):
        try:
            return int(v)
        except Exception:
            return None

    # Drive params
    positive_keys = ("wheel_base_m", "wheel_separation_m", "track_width_m", "watchdog_timeout_s", "fpv_lease_ttl_sec")
    nonneg_keys = ("max_linear_mps", "max_angular_rps", "teleop_linear_mps", "teleop_angular_rps", "teleop_omni_turn_gain")
    int_nonneg_keys = ("teleop_medium_steps", "teleop_fast_linear_steps", "teleop_fast_angular_steps")

    for k in positive_keys:
        if k in drive_params:
            v = _as_float(drive_params.get(k))
            if v is None or v <= 0:
                raise ValueError(f"drive_profiles.{drive_name}.params.{k} must be > 0 (got {drive_params.get(k)})")

    for k in nonneg_keys:
        if k in drive_params:
            v = _as_float(drive_params.get(k))
            if v is None or v < 0:
                raise ValueError(f"drive_profiles.{drive_name}.params.{k} must be >= 0 (got {drive_params.get(k)})")

    if "spin_speed_mult" in drive_params:
        v = _as_float(drive_params.get("spin_speed_mult"))
        if v is None or v < 0 or v > 1.0:
            raise ValueError(f"drive_profiles.{drive_name}.params.spin_speed_mult must be in [0, 1] (got {drive_params.get('spin_speed_mult')})")

    if "stall_timeout_s" in drive_params:
        v = _as_float(drive_params.get("stall_timeout_s"))
        if v is None or v < 0:
            raise ValueError(f"drive_profiles.{drive_name}.params.stall_timeout_s must be >= 0 (got {drive_params.get('stall_timeout_s')})")

    if "stall_duty_pct" in drive_params:
        v = _as_float(drive_params.get("stall_duty_pct"))
        if v is None or v < 0 or v > 100:
            raise ValueError(f"drive_profiles.{drive_name}.params.stall_duty_pct must be in [0, 100] (got {drive_params.get('stall_duty_pct')})")

    if "teleop_speed_step" in drive_params:
        v = _as_float(drive_params.get("teleop_speed_step"))
        if v is None or v <= 1.0:
            raise ValueError(f"drive_profiles.{drive_name}.params.teleop_speed_step must be > 1.0 (got {drive_params.get('teleop_speed_step')})")

    if "teleop_smoothing_alpha" in drive_params:
        v = _as_float(drive_params.get("teleop_smoothing_alpha"))
        if v is None or v < 0 or v > 1.0:
            raise ValueError(f"drive_profiles.{drive_name}.params.teleop_smoothing_alpha must be in [0, 1] (got {drive_params.get('teleop_smoothing_alpha')})")

    for k in int_nonneg_keys:
        if k in drive_params:
            v = _as_int(drive_params.get(k))
            if v is None or v < 0:
                raise ValueError(f"drive_profiles.{drive_name}.params.{k} must be >= 0 (got {drive_params.get(k)})")

    # Hardware params
    if "pwm_hz" in hw_params:
        v = _as_int(hw_params.get("pwm_hz"))
        if v is None or v <= 0:
            raise ValueError(f"hardware_profiles.{hw_name}.params.pwm_hz must be > 0 (got {hw_params.get('pwm_hz')})")

    if "max_pwm" in hw_params:
        v = _as_int(hw_params.get("max_pwm"))
        if v is None or v < 0 or v > 100:
            raise ValueError(f"hardware_profiles.{hw_name}.params.max_pwm must be in [0, 100] (got {hw_params.get('max_pwm')})")

    if "pwm_ramp_ms" in hw_params:
        v = _as_float(hw_params.get("pwm_ramp_ms"))
        if v is None or v < 0:
            raise ValueError(f"hardware_profiles.{hw_name}.params.pwm_ramp_ms must be >= 0 (got {hw_params.get('pwm_ramp_ms')})")

    if "pwm_deadband_pct" in hw_params:
        v = _as_float(hw_params.get("pwm_deadband_pct"))
        if v is None or v < 0 or v > 100:
            raise ValueError(f"hardware_profiles.{hw_name}.params.pwm_deadband_pct must be in [0, 100] (got {hw_params.get('pwm_deadband_pct')})")

    if "cmd_rate_hz" in hw_params:
        v = _as_float(hw_params.get("cmd_rate_hz"))
        if v is None or v < 0:
            raise ValueError(f"hardware_profiles.{hw_name}.params.cmd_rate_hz must be >= 0 (got {hw_params.get('cmd_rate_hz')})")

    if "pwm_slew_pct_per_s" in hw_params:
        v = _as_float(hw_params.get("pwm_slew_pct_per_s"))
        if v is None or v < 0:
            raise ValueError(f"hardware_profiles.{hw_name}.params.pwm_slew_pct_per_s must be >= 0 (got {hw_params.get('pwm_slew_pct_per_s')})")
