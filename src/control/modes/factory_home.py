#!/usr/bin/env python3
# factory_home.py
# Restore factory arm home.
# Author: Daniel Würmli

"""Restore factory arm home."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "factory_home")
