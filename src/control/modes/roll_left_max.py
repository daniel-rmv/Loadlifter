#!/usr/bin/env python3
# roll_left_max.py
# Roll wrist left.
# Author: Daniel Würmli

"""Roll wrist left."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "roll_left_max", duration_ms=duration_ms)
