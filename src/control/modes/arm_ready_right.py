#!/usr/bin/env python3
# arm_ready_right.py
# Move arm into ready posture with base 90 deg right.
# Author: Daniel Würmli

"""Move arm into ready posture with base 90 deg right."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "ready_right")
