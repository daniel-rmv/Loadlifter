#!/usr/bin/env python3
# gripper_open.py
# Open gripper.
# Author: Daniel Würmli

"""Open gripper."""

from .arm_common import call_method


def run(arm, duration_ms: int = 400):
    call_method(arm, "open_gripper", duration_ms=duration_ms)
