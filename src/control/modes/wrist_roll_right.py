#!/usr/bin/env python3
# wrist_roll_right.py
# Relative wrist roll right.
# Author: Daniel Würmli

"""Relative wrist roll right."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "wrist_roll_right", degrees, duration_ms=duration_ms)
