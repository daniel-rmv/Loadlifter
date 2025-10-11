#!/usr/bin/env python3
# arm_common.py
# Shared helpers for arm-related modes.
# Author: Daniel Würmli

"""Shared helpers for arm-related modes."""

VALID_JOINTS = {"gripper", "wrist_roll", "wrist_pitch", "elbow", "shoulder", "base"}
DEFAULT_BASE_MS = 1200


def ensure_joint(name: str) -> str:
    joint = name.strip()
    if joint not in VALID_JOINTS:
        raise ValueError(f"Unknown joint: {joint}")
    return joint


def call_method(arm, method: str, *args, **kwargs):
    handler = getattr(arm, method, None)
    if handler is None:
        raise AttributeError(f"ArmSystem has no method '{method}'")
    return handler(*args, **kwargs)
