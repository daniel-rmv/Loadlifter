#!/usr/bin/env python3
# arm_getobjecttop_left_right.py
# Pick up from the top on the left and place on the right.
# Author: Daniel Würmli

"""Pick up from the top on the left and place on the right."""

import time

POSE_MS = 1700
HOME_MS = 1900
ROTATE_MS = 1200
GRIP_MS = 700
SETTLE_S = 0.4


def _wait(duration_ms: int, extra: float = SETTLE_S) -> None:
    time.sleep(duration_ms / 1000.0 + extra)


def _home(arm):
    arm.home(duration_ms=HOME_MS)
    _wait(HOME_MS)


def _move_top_pose(arm):
    arm.move_top_pick_pose(duration_ms=POSE_MS)
    _wait(POSE_MS)


def run(arm):
    _home(arm)

    arm.open_gripper(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.2)

    arm.base_left_90(duration_ms=ROTATE_MS)
    _wait(ROTATE_MS)

    _move_top_pose(arm)

    arm.gripper_object(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.2)

    _home(arm)

    arm.base_right_90(duration_ms=ROTATE_MS)
    _wait(ROTATE_MS)

    _move_top_pose(arm)

    arm.open_gripper(duration_ms=GRIP_MS)
    _wait(GRIP_MS, extra=0.4)

    _home(arm)
