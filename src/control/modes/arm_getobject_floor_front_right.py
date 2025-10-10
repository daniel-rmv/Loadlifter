#!/usr/bin/env python3
# arm_getobject_floor_front_right.py
# Pickup in front, place on right side.
# Author: Daniel Würmli

"""Pickup in front, place on right side."""

import time

POSE_MS = 1700
HOME_MS = 1900
ROTATE_MS = 1200
GRIP_MS = 700
SETTLE_S = 0.4

PICK_SHOULDER = 7
PICK_ELBOW = 50
PICK_PITCH = 2


def _wait(duration_ms: int, extra: float = SETTLE_S) -> None:
    time.sleep(duration_ms / 1000.0 + extra)


def _move_pick_pose(arm):
    arm.shoulder_percent(PICK_SHOULDER, duration_ms=POSE_MS)
    arm.elbow_percent(PICK_ELBOW, duration_ms=POSE_MS)
    arm.pitch_percent(PICK_PITCH, duration_ms=POSE_MS)
    _wait(POSE_MS)


def _home(arm):
    arm.home(duration_ms=HOME_MS)
    _wait(HOME_MS)


def run(arm):
    _home(arm)

    arm.open_gripper(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.2)

    _move_pick_pose(arm)

    arm.gripper_object(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.2)

    _home(arm)

    arm.base_right_90(duration_ms=ROTATE_MS)
    _wait(ROTATE_MS)

    _move_pick_pose(arm)

    arm.open_gripper(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.4)

    _home(arm)
