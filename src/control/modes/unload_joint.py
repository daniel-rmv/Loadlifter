#!/usr/bin/env python3
# unload_joint.py
# Unload a single joint.
# Author: Daniel Würmli

"""Unload a single joint."""

from .arm_common import call_method, ensure_joint


def run(arm, joint: str):
    joint_name = ensure_joint(joint)
    call_method(arm, "unload_joint", joint_name)
