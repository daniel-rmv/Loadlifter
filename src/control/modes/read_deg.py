#!/usr/bin/env python3
# read_deg.py
# Read joint angle in degrees.
# Author: Daniel Würmli

"""Read joint angle in degrees."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str):
    joint_name = ensure_joint(joint)
    value = call_method(arm, "read_joint_deg", joint_name)
    print(f"{joint_name} deg = {value}")
