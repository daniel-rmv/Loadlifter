#!/usr/bin/env python3
# roll_right_max.py
# Roll wrist right.
# Author: Daniel Würmli

"""Roll wrist right."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "roll_right_max", duration_ms=duration_ms)
