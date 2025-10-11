#!/usr/bin/env python3
# arm_ready_front.py
# Move arm into ready posture facing forward.
# Author: Daniel Würmli

"""Move arm into ready posture facing forward."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "ready_front")
