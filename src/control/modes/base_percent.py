#!/usr/bin/env python3
# base_percent.py
# Set base rotation by percent.
# Author: Daniel Würmli

"""Set base rotation by percent."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, percent: float, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_percent", percent, duration_ms=duration_ms)
