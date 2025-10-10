#!/usr/bin/env python3
# elbow_zero.py
# Set elbow to neutral.
# Author: Daniel Würmli

"""Set elbow to neutral."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "elbow_zero", duration_ms=duration_ms)
