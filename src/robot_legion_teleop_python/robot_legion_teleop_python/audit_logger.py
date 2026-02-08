#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
audit_logger.py

Structured audit logging for command traceability and compliance.

This module provides a centralized audit trail for:
- Who issued a command (client_id, source: teleop/orchestrator/etc)
- What command was issued (playbook command_id, parameters)
- When it was issued (timestamp)
- To which robot (robot_name)
- What the outcome was (success/failure/cancelled)

This is critical for:
- DIU Sprint 1 requirement: "Correct logging and traceability for validation"
- Security and compliance (who did what and when)
- Debugging fleet behavior
- After-action review

Usage:
  from robot_legion_teleop_python.audit_logger import AuditLogger
  
  logger = AuditLogger(node, "motor_driver")
  logger.log_command(
    robot="robot1",
    source="teleop",
    command_id="transit",
    parameters={"direction": "forward", "duration_s": 2.0},
    status="started"
  )

The logger writes to both ROS logs and an optional file sink.
"""

import json
import logging
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional

try:
    from rclpy.node import Node
except ImportError:
    Node = None


@dataclass
class AuditEvent:
    """Structured audit event for serialization and traceability."""
    timestamp: float  # Unix timestamp
    robot: str  # Target robot name
    source: str  # Command source (teleop, orchestrator, api, etc)
    source_id: Optional[str]  # Client ID or user
    command_id: str  # Playbook command name
    parameters: Dict[str, Any]  # Command parameters as dict
    status: str  # started, succeeded, failed, cancelled
    details: Optional[str] = None  # Error message or context
    duration_s: Optional[float] = None  # Execution time in seconds


class AuditLogger:
    """
    Centralized audit logging for fleet commands.

    Logs to both ROS logger and optionally to a file for long-term retention.
    """

    def __init__(
        self,
        node: Optional[Node] = None,
        component_name: str = "component",
        log_file_path: Optional[str] = None,
    ):
        """
        Initialize audit logger.

        Args:
            node: ROS node (used for ros logs; optional if you just want file logging)
            component_name: Name of component (motor_driver, unit_executor, teleop, etc)
            log_file_path: Path to append JSON audit events; if None, no file logging
        """
        self.node = node
        self.component_name = component_name
        self.log_file_path = log_file_path
        self.logger = logging.getLogger(f"audit.{component_name}")

        if log_file_path:
            try:
                self.log_file = open(log_file_path, "a")
            except Exception as e:
                if node:
                    node.get_logger().warning(f"Could not open audit log file {log_file_path}: {e}")
                else:
                    self.logger.warning(f"Could not open audit log file {log_file_path}: {e}")
                self.log_file = None
        else:
            self.log_file = None

    def log_command(
        self,
        robot: str,
        source: str,
        command_id: str,
        parameters: Optional[Dict[str, Any]] = None,
        status: str = "issued",
        source_id: Optional[str] = None,
        details: Optional[str] = None,
        duration_s: Optional[float] = None,
    ):
        """
        Log a command execution event.

        Args:
            robot: Target robot name
            source: Command source (teleop, orchestrator, api, etc)
            command_id: Playbook command (transit, rotate, etc)
            parameters: Command parameters dict
            status: Command state (issued, started, succeeded, failed, cancelled)
            source_id: Client ID or user identifier
            details: Optional context (error message, reason for failure, etc)
            duration_s: Execution duration in seconds (for succeeded/failed events)
        """
        event = AuditEvent(
            timestamp=time.time(),
            robot=robot,
            source=source,
            source_id=source_id,
            command_id=command_id,
            parameters=parameters or {},
            status=status,
            details=details,
            duration_s=duration_s,
        )

        # Format human-readable log message
        msg = (
            f"[AUDIT] {self.component_name} | robot={robot} source={source} "
            f"cmd={command_id} status={status}"
        )
        if source_id:
            msg += f" client={source_id}"
        if duration_s is not None:
            msg += f" duration_s={duration_s:.2f}"
        if details:
            msg += f" | {details}"

        # Write to ROS logger if available
        if self.node:
            self.node.get_logger().info(msg)
        else:
            self.logger.info(msg)

        # Write JSON to file for persistence
        if self.log_file:
            try:
                json_line = json.dumps(asdict(event), separators=(",", ":")) + "\n"
                self.log_file.write(json_line)
                self.log_file.flush()
            except Exception as e:
                if self.node:
                    self.node.get_logger().warning(f"Failed to write audit log: {e}")
                else:
                    self.logger.warning(f"Failed to write audit log: {e}")

    def close(self):
        """Close file handle if open."""
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
