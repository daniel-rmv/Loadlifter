#!/usr/bin/env python3
# load_joint.py
# Load (enable torque) on a single joint.
# Author: Daniel Würmli

"""Load (enable torque) on a single joint."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str):
    joint_name = ensure_joint(joint)
    call_method(arm, "load_joint", joint_name)
