#!/usr/bin/env python3
# wrist_pitch_down.py
# Relative wrist pitch down.
# Author: Daniel Würmli

"""Relative wrist pitch down."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "wrist_pitch_down", degrees, duration_ms=duration_ms)
