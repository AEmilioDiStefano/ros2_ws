#!/usr/bin/env python3
"""
hardware_interface.py

Thin hardware abstraction layer for motor outputs.

Goals:
- Provide a consistent API for setting motor directions and PWM duty
- Allow a safe mock implementation when RPi.GPIO is not available (desktop)
- Keep behavior simple and well-documented for new engineers

Usage:
  from robot_legion_teleop_python.hardware_interface import HardwareInterface
  hw = HardwareInterface(gpio_map)
  hw.set_motor(left_duty, left_dir, right_duty, right_dir)
  hw.stop()

The gpio_map is a mapping of expected keys (names are flexible, see
drive_profiles.yaml) such as:
  {
    "en_left": 12,
    "in1_left": 23,
    "in2_left": 22,
    "en_right": 13,
    "in1_right": 27,
    "in2_right": 17,
  }

This module intentionally keeps hardware logic separated from motor mixing
so drive-type code can remain device-agnostic.
"""

from typing import Dict, Optional
import logging
import time

try:
    import RPi.GPIO as GPIO  # type: ignore
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False

LOG = logging.getLogger("hardware_interface")


class HardwareInterface:
    """Abstraction for simple 2-channel differential motor outputs.

    This implementation focuses on the common H-bridge configuration used
    in our profiles. It exposes a minimal API so other modules need not
    import or depend on RPi.GPIO directly.
    """

    def __init__(self, gpio_map: Optional[Dict[str, int]] = None):
        self.gpio_map = gpio_map or {}
        self.left_pwm = None
        self.right_pwm = None
        self.pwm_hz = int(self.gpio_map.get("pwm_hz") or 1000)
        self.pwm_ramp_ms = float(self.gpio_map.get("pwm_ramp_ms") or 0.0)
        self.pwm_slew_pct_per_s = float(self.gpio_map.get("pwm_slew_pct_per_s") or 0.0)
        self._cur_left_duty = 0.0
        self._cur_right_duty = 0.0
        self._cur_fl_duty = 0.0
        self._cur_fr_duty = 0.0
        self._cur_rl_duty = 0.0
        self._cur_rr_duty = 0.0
        self._last_update = time.monotonic()
        # Allow per-motor polarity inversion via profile flags. Accept several key names.
        self.invert_left = bool(
            self.gpio_map.get("invert_left")
            or self.gpio_map.get("left_inverted")
            or self.gpio_map.get("left_polarity_invert")
        )
        self.invert_right = bool(
            self.gpio_map.get("invert_right")
            or self.gpio_map.get("right_inverted")
            or self.gpio_map.get("right_polarity_invert")
        )

        if GPIO_AVAILABLE and self.gpio_map:
            try:
                self._setup_gpio()
                LOG.info("GPIO hardware interface initialized")
                LOG.info("GPIO map keys: %s; invert_left=%s invert_right=%s", list(self.gpio_map.keys()), self.invert_left, self.invert_right)
            except Exception as e:
                LOG.warning("GPIO init failed: %s", e)
                self._mock_mode()
        else:
            self._mock_mode()

    def _mock_mode(self):
        """Fallback that doesn't touch hardware; logs instead.
        This makes the package runnable on a laptop for development/testing.
        """
        self._mock = True
        if not GPIO_AVAILABLE:
            LOG.warning("RPi.GPIO not available - using mock hardware interface")
        else:
            LOG.warning("GPIO map empty - using mock hardware interface")

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # collect pins we expect for a 2-channel H-bridge
        pins = [
            self.gpio_map.get("in1_left"),
            self.gpio_map.get("in2_left"),
            self.gpio_map.get("in1_right"),
            self.gpio_map.get("in2_right"),
            self.gpio_map.get("en_left"),
            self.gpio_map.get("en_right"),
            self.gpio_map.get("fl_in1"),
            self.gpio_map.get("fl_in2"),
            self.gpio_map.get("fr_in1"),
            self.gpio_map.get("fr_in2"),
            self.gpio_map.get("rl_in1"),
            self.gpio_map.get("rl_in2"),
            self.gpio_map.get("rr_in1"),
            self.gpio_map.get("rr_in2"),
            self.gpio_map.get("fl_pwm"),
            self.gpio_map.get("fr_pwm"),
            self.gpio_map.get("rl_pwm"),
            self.gpio_map.get("rr_pwm"),
        ]
        pins = [p for p in pins if p is not None]

        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

        # PWM channels
        if self.gpio_map.get("en_left") is not None and self.gpio_map.get("en_right") is not None:
            self.left_pwm = GPIO.PWM(self.gpio_map.get("en_left"), self.pwm_hz)
            self.right_pwm = GPIO.PWM(self.gpio_map.get("en_right"), self.pwm_hz)
            self.left_pwm.start(0)
            self.right_pwm.start(0)
        else:
            self.left_pwm = None
            self.right_pwm = None

        if self.gpio_map.get("fl_pwm") is not None:
            self.fl_pwm = GPIO.PWM(self.gpio_map.get("fl_pwm"), self.pwm_hz)
            self.fl_pwm.start(0)
        else:
            self.fl_pwm = None
        if self.gpio_map.get("fr_pwm") is not None:
            self.fr_pwm = GPIO.PWM(self.gpio_map.get("fr_pwm"), self.pwm_hz)
            self.fr_pwm.start(0)
        else:
            self.fr_pwm = None
        if self.gpio_map.get("rl_pwm") is not None:
            self.rl_pwm = GPIO.PWM(self.gpio_map.get("rl_pwm"), self.pwm_hz)
            self.rl_pwm.start(0)
        else:
            self.rl_pwm = None
        if self.gpio_map.get("rr_pwm") is not None:
            self.rr_pwm = GPIO.PWM(self.gpio_map.get("rr_pwm"), self.pwm_hz)
            self.rr_pwm.start(0)
        else:
            self.rr_pwm = None
        self._mock = False

    def set_motor(self, left_duty: float, left_dir: int, right_duty: float, right_dir: int, bypass_ramp: bool = False):
        """Set motor outputs.

        left_dir/right_dir: 1 for forward, -1 for reverse
        duty: 0..100
        """
        if getattr(self, "_mock", True):
            LOG.debug("MOCK set_motor: L(duty=%s,dir=%s) R(duty=%s,dir=%s)", left_duty, left_dir, right_duty, right_dir)
            return

        # Apply configured polarity inversion if requested by the profile
        if self.invert_left:
            left_dir = -left_dir
        if self.invert_right:
            right_dir = -right_dir

        # Soft-start ramp / slew limiter (limits duty change per call)
        if not bypass_ramp and ((self.pwm_ramp_ms and self.pwm_ramp_ms > 0) or (self.pwm_slew_pct_per_s and self.pwm_slew_pct_per_s > 0)):
            now = time.monotonic()
            dt = max(0.0, now - self._last_update)
            self._last_update = now
            ramp_rate = 0.0
            if self.pwm_ramp_ms and self.pwm_ramp_ms > 0:
                ramp_rate = 100.0 / (self.pwm_ramp_ms / 1000.0)
            slew_rate = float(self.pwm_slew_pct_per_s or 0.0)
            if ramp_rate and slew_rate:
                rate_per_sec = min(ramp_rate, slew_rate)
            else:
                rate_per_sec = ramp_rate or slew_rate

            max_delta = rate_per_sec * dt if rate_per_sec > 0 else 100.0

            def _ramp(cur, target):
                if target > cur:
                    return min(target, cur + max_delta)
                if target < cur:
                    return max(target, cur - max_delta)
                return cur

            self._cur_left_duty = _ramp(self._cur_left_duty, float(left_duty))
            self._cur_right_duty = _ramp(self._cur_right_duty, float(right_duty))
        else:
            # Bypass ramp/slew (immediate response)
            if bypass_ramp:
                self._last_update = time.monotonic()
            self._cur_left_duty = float(left_duty)
            self._cur_right_duty = float(right_duty)

        # left motor pins
        in1 = self.gpio_map.get("in1_left")
        in2 = self.gpio_map.get("in2_left")
        if in1 is not None and in2 is not None:
            GPIO.output(in1, GPIO.HIGH if left_dir > 0 else GPIO.LOW)
            GPIO.output(in2, GPIO.LOW if left_dir > 0 else GPIO.HIGH)

        # right motor pins
        in3 = self.gpio_map.get("in1_right")
        in4 = self.gpio_map.get("in2_right")
        if in3 is not None and in4 is not None:
            GPIO.output(in3, GPIO.HIGH if right_dir > 0 else GPIO.LOW)
            GPIO.output(in4, GPIO.LOW if right_dir > 0 else GPIO.HIGH)

        # set PWM duty
        if self.left_pwm is not None:
            self.left_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_left_duty))))
        if self.right_pwm is not None:
            self.right_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_right_duty))))

    def set_mecanum(
        self,
        fl_duty: float, fl_dir: int,
        fr_duty: float, fr_dir: int,
        rl_duty: float, rl_dir: int,
        rr_duty: float, rr_dir: int,
    ):
        """Set outputs for a 4-channel mecanum/omni drivetrain."""
        if getattr(self, "_mock", True):
            LOG.debug(
                "MOCK set_mecanum: FL(duty=%s,dir=%s) FR(duty=%s,dir=%s) RL(duty=%s,dir=%s) RR(duty=%s,dir=%s)",
                fl_duty, fl_dir, fr_duty, fr_dir, rl_duty, rl_dir, rr_duty, rr_dir,
            )
            return

        # Apply configured polarity inversion if requested by the profile
        if self.invert_left:
            fl_dir = -fl_dir
            rl_dir = -rl_dir
        if self.invert_right:
            fr_dir = -fr_dir
            rr_dir = -rr_dir

        # Soft-start ramp / slew limiter (limits duty change per call)
        if (self.pwm_ramp_ms and self.pwm_ramp_ms > 0) or (self.pwm_slew_pct_per_s and self.pwm_slew_pct_per_s > 0):
            now = time.monotonic()
            dt = max(0.0, now - self._last_update)
            self._last_update = now
            ramp_rate = 0.0
            if self.pwm_ramp_ms and self.pwm_ramp_ms > 0:
                ramp_rate = 100.0 / (self.pwm_ramp_ms / 1000.0)
            slew_rate = float(self.pwm_slew_pct_per_s or 0.0)
            if ramp_rate and slew_rate:
                rate_per_sec = min(ramp_rate, slew_rate)
            else:
                rate_per_sec = ramp_rate or slew_rate

            max_delta = rate_per_sec * dt if rate_per_sec > 0 else 100.0

            def _ramp(cur, target):
                if target > cur:
                    return min(target, cur + max_delta)
                if target < cur:
                    return max(target, cur - max_delta)
                return cur

            self._cur_fl_duty = _ramp(self._cur_fl_duty, float(fl_duty))
            self._cur_fr_duty = _ramp(self._cur_fr_duty, float(fr_duty))
            self._cur_rl_duty = _ramp(self._cur_rl_duty, float(rl_duty))
            self._cur_rr_duty = _ramp(self._cur_rr_duty, float(rr_duty))
        else:
            self._cur_fl_duty = float(fl_duty)
            self._cur_fr_duty = float(fr_duty)
            self._cur_rl_duty = float(rl_duty)
            self._cur_rr_duty = float(rr_duty)

        # Direction pins
        def _set_dir(pin1, pin2, direction):
            if pin1 is None or pin2 is None:
                return
            GPIO.output(pin1, GPIO.HIGH if direction > 0 else GPIO.LOW)
            GPIO.output(pin2, GPIO.LOW if direction > 0 else GPIO.HIGH)

        _set_dir(self.gpio_map.get("fl_in1"), self.gpio_map.get("fl_in2"), fl_dir)
        _set_dir(self.gpio_map.get("fr_in1"), self.gpio_map.get("fr_in2"), fr_dir)
        _set_dir(self.gpio_map.get("rl_in1"), self.gpio_map.get("rl_in2"), rl_dir)
        _set_dir(self.gpio_map.get("rr_in1"), self.gpio_map.get("rr_in2"), rr_dir)

        # PWM duty
        if self.fl_pwm is not None:
            self.fl_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_fl_duty))))
        if self.fr_pwm is not None:
            self.fr_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_fr_duty))))
        if self.rl_pwm is not None:
            self.rl_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_rl_duty))))
        if self.rr_pwm is not None:
            self.rr_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(self._cur_rr_duty))))
    def stop(self):
        """Stop motors and cleanup if using real GPIO"""
        if getattr(self, "_mock", True):
            LOG.debug("MOCK stop called")
            return
        try:
            if self.left_pwm is not None:
                self.left_pwm.stop()
            if self.right_pwm is not None:
                self.right_pwm.stop()
            if getattr(self, "fl_pwm", None) is not None:
                self.fl_pwm.stop()
            if getattr(self, "fr_pwm", None) is not None:
                self.fr_pwm.stop()
            if getattr(self, "rl_pwm", None) is not None:
                self.rl_pwm.stop()
            if getattr(self, "rr_pwm", None) is not None:
                self.rr_pwm.stop()
            GPIO.cleanup()
        except Exception as e:
            LOG.warning("Error cleaning up GPIO: %s", e)
