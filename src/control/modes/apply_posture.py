#!/usr/bin/env python3
# apply_posture.py
# Apply a named stored posture.
# Author: Daniel Würmli

"""Apply a named stored posture."""

from .arm_common import call_method


def run(arm, pose: str):
    call_method(arm, "apply_posture", pose)
