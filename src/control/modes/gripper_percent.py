#!/usr/bin/env python3
# gripper_percent.py
# Set gripper opening by percent.
# Author: Daniel Würmli

"""Set gripper opening by percent."""

from .arm_common import call_method


def run(arm, percent: float, duration_ms: int = 400):
    call_method(arm, "set_gripper_percent", percent, duration_ms=duration_ms)
