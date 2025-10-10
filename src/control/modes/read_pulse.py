#!/usr/bin/env python3
# read_pulse.py
# Read joint pulse value.
# Author: Daniel Würmli

"""Read joint pulse value."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str):
    joint_name = ensure_joint(joint)
    value = call_method(arm, "read_joint_pulse", joint_name)
    print(f"{joint_name} pulse = {value}")
