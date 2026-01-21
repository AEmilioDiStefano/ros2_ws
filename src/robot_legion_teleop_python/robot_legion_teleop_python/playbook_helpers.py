#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

import time
from dataclasses import dataclass
from typing import Callable, Optional

from geometry_msgs.msg import Twist


@dataclass
class TimedTwistPlan:
    twist: Twist
    duration_s: float
    status_text: str


def run_timed_twist(
    publish_fn: Callable[[Twist], None],
    feedback_fn: Callable[[float, str], None],
    plan: TimedTwistPlan,
    rate_hz: float = 20.0,
    stop_at_end: bool = True,
):
    """
    Publish a Twist for plan.duration_s with periodic feedback.
    This is intentionally simple: it's a demo "autonomy stand-in".
    """
    start = time.monotonic()
    end = start + max(0.0, float(plan.duration_s))
    period = 1.0 / max(1.0, float(rate_hz))

    last_fb = 0.0
    while True:
        now = time.monotonic()
        if now >= end:
            break

        publish_fn(plan.twist)

        # Feedback at ~2 Hz
        frac = (now - start) / (end - start) if end > start else 1.0
        if (now - last_fb) >= 0.5:
            feedback_fn(max(0.0, min(1.0, frac)) * 100.0, plan.status_text)
            last_fb = now

        time.sleep(period)

    if stop_at_end:
        publish_fn(Twist())
        feedback_fn(100.0, "completed")
