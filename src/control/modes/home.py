#!/usr/bin/env python3
# home.py
# Arm home position mode with gentle motion.
# Author: Daniel Würmli

"""Arm home position mode with gentle motion."""

import time

from .arm_common import call_method

HOME_MS = 1900
SETTLE_S = 0.4


def run(arm):
    """Move arm to stored home position."""
    call_method(arm, "home", duration_ms=HOME_MS)
    time.sleep(HOME_MS / 1000.0 + SETTLE_S)
