#!/usr/bin/env python3
# wrist_roll_left.py
# Relative wrist roll left.
# Author: Daniel Würmli

"""Relative wrist roll left."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "wrist_roll_left", degrees, duration_ms=duration_ms)
