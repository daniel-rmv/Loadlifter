#!/usr/bin/env python3
# wrist_pitch_up.py
# Relative wrist pitch up.
# Author: Daniel Würmli

"""Relative wrist pitch up."""

from .arm_common import call_method


def run(arm, degrees: float, duration_ms: int = 500):
    call_method(arm, "wrist_pitch_up", degrees, duration_ms=duration_ms)
