#!/usr/bin/env python3
# elbow_up.py
# Relative elbow move up.
# Author: Daniel Würmli

"""Relative elbow move up."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "elbow_up", degrees, duration_ms=duration_ms)
