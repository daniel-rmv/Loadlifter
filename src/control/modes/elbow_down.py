#!/usr/bin/env python3
# elbow_down.py
# Relative elbow move down.
# Author: Daniel Würmli

"""Relative elbow move down."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "elbow_down", degrees, duration_ms=duration_ms)
