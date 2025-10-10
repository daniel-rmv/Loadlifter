#!/usr/bin/env python3
# base_left_max.py
# Rotate base to left maximum.
# Author: Daniel Würmli

"""Rotate base to left maximum."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_left_max", duration_ms=duration_ms)
