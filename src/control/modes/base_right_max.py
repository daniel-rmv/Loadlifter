#!/usr/bin/env python3
# base_right_max.py
# Rotate base to right maximum.
# Author: Daniel Würmli

"""Rotate base to right maximum."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_right_max", duration_ms=duration_ms)
