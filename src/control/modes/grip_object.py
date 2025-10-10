#!/usr/bin/env python3
# grip_object.py
# Grip object helper.
# Author: Daniel Würmli

"""Grip object helper."""

from .arm_common import call_method


def run(arm, duration_ms: int = 500):
    call_method(arm, "grip_object", duration_ms=duration_ms)
