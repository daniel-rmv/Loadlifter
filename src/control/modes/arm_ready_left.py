#!/usr/bin/env python3
# arm_ready_left.py
# Move arm into ready posture with base 90 deg left.
# Author: Daniel Würmli

"""Move arm into ready posture with base 90 deg left."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "ready_left")
