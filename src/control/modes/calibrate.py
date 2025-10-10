#!/usr/bin/env python3
# calibrate.py
# Interactive arm calibration helper.
# Author: Daniel Würmli

"""Interactive arm calibration helper."""

import time

from .arm_common import call_method, VALID_JOINTS
from ...high_level.arm_system import pulse_to_deg


PROMPT = "\n[CAL] Position the arm manually. Press ENTER as soon as it is aligned. "
READ_ATTEMPTS = 5
READ_DELAY_S = 0.15


def _read_joint_state(arm, joint: str):
    # ensure joint is actively powered before reading
    try:
        arm.load_joint(joint)
    except Exception:
        pass
    time.sleep(0.2)

    raw = None
    for _ in range(READ_ATTEMPTS):
        raw = arm.read_joint_pulse(joint)
        if raw is not None:
            break
        time.sleep(READ_DELAY_S)

    deg = None
    if raw is not None:
        try:
            deg = pulse_to_deg(joint, raw)
        except Exception:
            deg = None
    else:
        # letzte Chance: direkter Deg-Read (kann ebenfalls None liefern)
        for _ in range(READ_ATTEMPTS):
            deg = arm.read_joint_deg(joint)
            if deg is not None:
                break
            time.sleep(READ_DELAY_S)

    return deg, raw


def run(arm):
    print("[CAL] Entering calibration mode (all servos will be unloaded).")
    call_method(arm, "unload_all")
    time.sleep(0.4)

    try:
        input(PROMPT)
    except KeyboardInterrupt:
        print("\n[CAL] Aborted.")
        return

    print("[CAL] Reloading servos...")
    call_method(arm, "load_all")
    time.sleep(0.6)

    print("[CAL] Current joint positions:")
    for joint in sorted(VALID_JOINTS):
        deg, pulse = _read_joint_state(arm, joint)
        print(f"  - {joint:12s} deg={deg!r}  pulse={pulse!r}")

    print("[CAL] Done. Values are not stored—please note them down.")
