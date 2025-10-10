#!/usr/bin/env python3
# gripper_object.py
# Grip object preset.
# Author: Daniel Würmli

"""Grip object preset."""

from .arm_common import call_method


def run(arm, duration_ms: int = 400):
    call_method(arm, "gripper_object", duration_ms=duration_ms)
