#!/usr/bin/env python3
# elbow_percent.py
# Set elbow position by percent.
# Author: Daniel Würmli

"""Set elbow position by percent."""

from .arm_common import call_method


def run(arm, percent: float, duration_ms: int = 500):
    call_method(arm, "elbow_percent", percent, duration_ms=duration_ms)
