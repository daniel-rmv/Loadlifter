#!/usr/bin/env python3
# list_poses.py
# List available stored arm poses.
# Author: Daniel Würmli

"""List available stored arm poses."""

from .arm_common import call_method


def run(arm):
    poses = call_method(arm, "list_poses")
    print("Posen:", ", ".join(poses))
