#!/usr/bin/env python3
# move_pulse.py
# Move a joint to an absolute pulse value.
# Author: Daniel Würmli

"""Move a joint to an absolute pulse value."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str, pulse: int, duration_ms: int = 800):
    joint_name = ensure_joint(joint)
    call_method(arm, "move_joint_pulse", joint_name, pulse, duration_ms=duration_ms)
