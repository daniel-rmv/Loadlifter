#!/usr/bin/env python3
# pitch_percent.py
# Set wrist pitch by percent.
# Author: Daniel Würmli

"""Set wrist pitch by percent."""

from .arm_common import call_method


def run(arm, percent: float, duration_ms: int = 500):
    call_method(arm, "pitch_percent", percent, duration_ms=duration_ms)
