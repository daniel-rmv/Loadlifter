#!/usr/bin/env python3
# base_right_90.py
# Rotate base 90 degrees right.
# Author: Daniel Würmli

"""Rotate base 90 degrees right."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_right_90", duration_ms=duration_ms)
