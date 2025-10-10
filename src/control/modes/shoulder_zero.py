#!/usr/bin/env python3
# shoulder_zero.py
# Set shoulder to neutral.
# Author: Daniel Würmli

"""Set shoulder to neutral."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "shoulder_zero", duration_ms=duration_ms)
