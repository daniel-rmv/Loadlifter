#!/usr/bin/env python3
# base_left_45.py
# Rotate base 45 degrees left.
# Author: Daniel Würmli

"""Rotate base 45 degrees left."""

from .arm_common import call_method, DEFAULT_BASE_MS


def run(arm, duration_ms: int = DEFAULT_BASE_MS):
    call_method(arm, "base_left_45", duration_ms=duration_ms)
