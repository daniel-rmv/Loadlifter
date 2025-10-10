#!/usr/bin/env python3
# turn_left.py
# Relative base turn left.
# Author: Daniel Würmli

"""Relative base turn left."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, degrees: float, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "turn_left", degrees, duration_ms=duration_ms)
