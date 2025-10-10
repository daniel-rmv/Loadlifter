#!/usr/bin/env python3
# shoulder_percent.py
# Set shoulder position by percent.
# Author: Daniel Würmli

"""Set shoulder position by percent."""

from .arm_common import call_method


def run(arm, percent: float, duration_ms: int = 500):
    call_method(arm, "shoulder_percent", percent, duration_ms=duration_ms)
