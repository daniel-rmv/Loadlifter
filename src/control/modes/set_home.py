#!/usr/bin/env python3
# set_home.py
# Set current pose as home.
# Author: Daniel Würmli

"""Set current pose as home."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "set_home_from_current")
