#!/usr/bin/env python3
# pickup.py
# Automated pickup routine.
# Author: Daniel Würmli

"""Automated pickup routine."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "pickup")
