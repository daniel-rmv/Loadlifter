#!/usr/bin/env python3
# unload_all.py
# Unload all joints.
# Author: Daniel Würmli

"""Unload all joints."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "unload_all")
