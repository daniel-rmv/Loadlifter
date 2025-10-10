#!/usr/bin/env python3
# load_all.py
# Load all joints.
# Author: Daniel Würmli

"""Load all joints."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "load_all")
