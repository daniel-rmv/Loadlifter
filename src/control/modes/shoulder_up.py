#!/usr/bin/env python3
# shoulder_up.py
# Relative shoulder raise.
# Author: Daniel Würmli

"""Relative shoulder raise."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "shoulder_up", degrees, duration_ms=duration_ms)
