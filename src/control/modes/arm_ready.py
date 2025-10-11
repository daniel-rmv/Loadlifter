#!/usr/bin/env python3
# arm_ready.py
# Move arm into ready posture facing forward (alias of ready_front).
# Author: Daniel Würmli

"""Move arm into ready posture facing forward (alias of ready_front)."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "ready")
