#!/usr/bin/env python3
# roll_zero.py
# Set wrist roll to zero.
# Author: Daniel Würmli

"""Set wrist roll to zero."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "roll_zero", duration_ms=duration_ms)
