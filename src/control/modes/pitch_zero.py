#!/usr/bin/env python3
# pitch_zero.py
# Set wrist pitch to neutral.
# Author: Daniel Würmli

"""Set wrist pitch to neutral."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "pitch_zero", duration_ms=duration_ms)
