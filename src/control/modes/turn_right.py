#!/usr/bin/env python3
# turn_right.py
# Relative base turn right.
# Author: Daniel Würmli

"""Relative base turn right."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, degrees: float, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "turn_right", degrees, duration_ms=duration_ms)
