#!/usr/bin/env python3
# roll_percent.py
# Set wrist roll by percentage.
# Author: Daniel Würmli

"""Set wrist roll by percentage."""

from .arm_common import call_method


def run(arm, percent: float, duration_ms: int = 500):
    call_method(arm, "roll_percent", percent, duration_ms=duration_ms)
