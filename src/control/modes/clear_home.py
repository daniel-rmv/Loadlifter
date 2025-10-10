#!/usr/bin/env python3
# clear_home.py
# Clear persisted home pose.
# Author: Daniel Würmli

"""Clear persisted home pose."""

from .arm_common import call_method


def run(arm):
    call_method(arm, "clear_home_file")
    print("Home-File geloescht.")
