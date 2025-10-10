#!/usr/bin/env python3
# base_zero.py
# Set base to zero.
# Author: Daniel Würmli

"""Set base to zero."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_zero", duration_ms=duration_ms)
