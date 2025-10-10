#!/usr/bin/env python3
# shoulder_down.py
# Relative shoulder lower.
# Author: Daniel Würmli

"""Relative shoulder lower."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "shoulder_down", degrees, duration_ms=duration_ms)
