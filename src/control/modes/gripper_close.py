#!/usr/bin/env python3
# gripper_close.py
# Close gripper.
# Author: Daniel Würmli

"""Close gripper."""

from .arm_common import call_method


def run(arm, duration_ms: int = 400):
    call_method(arm, "close_gripper", duration_ms=duration_ms)
