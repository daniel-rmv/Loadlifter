#!/usr/bin/env python3
# move_deg.py
# Move a joint to absolute degrees.
# Author: Daniel Würmli

"""Move a joint to absolute degrees."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str, degrees: float, duration_ms: int = 800):
    joint_name = ensure_joint(joint)
    call_method(arm, "move_joint_deg", joint_name, degrees, duration_ms=duration_ms)
