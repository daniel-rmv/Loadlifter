#!/usr/bin/env python3
# defined_route_getobjecttop.py
# Guided route with top pickup and return to base.
# Author: Daniel Würmli

"""Guided route that mirrors follow_route with a top pickup in the first aisle."""

import time

from . import arm_getobjecttop_right
from ...high_level.arm_system import ArmSystem


TARGET_FRONT_TOL_MM = 20.0
ROTATE_MS = 1200
POSE_MS = 1700
HOME_MS = 1900
GRIP_MS = 700
SETTLE_S = 0.35


def _run_arm_pick():
    arm = ArmSystem()
    try:
        arm_getobjecttop_right.run(arm)
    finally:
        try:
            arm.cleanup()
        except Exception:
            pass


def _run_arm_release():
    arm = ArmSystem()
    try:
        arm.base_right_90(duration_ms=ROTATE_MS)
        time.sleep(ROTATE_MS / 1000.0 + SETTLE_S)

        arm.shoulder_percent(25.0, duration_ms=POSE_MS)
        arm.elbow_percent(55.0, duration_ms=POSE_MS)
        arm.pitch_percent(35.0, duration_ms=POSE_MS)
        time.sleep(POSE_MS / 1000.0 + SETTLE_S)

        arm.open_gripper(duration_ms=GRIP_MS)
        time.sleep(GRIP_MS / 1000.0 + SETTLE_S)

        arm.home(duration_ms=HOME_MS)
        time.sleep(HOME_MS / 1000.0 + SETTLE_S)
    finally:
        try:
            arm.cleanup()
        except Exception:
            pass


def run(nav, cfg: dict, buzzer, drv) -> None:
    """Execute the defined route with top pickup."""
    target_front_mm = float(cfg.get("defined_route_front_target_mm", 1000.0))
    target_tol_mm = float(cfg.get("defined_route_front_tolerance_mm", TARGET_FRONT_TOL_MM))
    event = nav.follow_left_until_right_open_or_front(
        right_open_mm=cfg["right_open_mm"],
        require_rearm=cfg["right_open_require_rearm"],
        rearm_below_mm=cfg.get("right_open_rearm_mm"),
    )

    if event == "front_stop":
        time.sleep(0.15)
        nav.rotate_left_deg(180.0)
        time.sleep(ALIGN_WAIT_S)
        nav.follow_right_until_stop()
        time.sleep(0.15)
        nav.rotate_left_deg(180.0)
        
        try:
            drv.stop()
        except Exception:
            pass
        nav.hard_zero()
        time.sleep(0.1)
        buzzer.on()
        time.sleep(cfg.get("buzzer_end_s", 3.0))
        buzzer.off()
        return

    duration = cfg["extra_forward_after_open_s"]
    nav.timed_forward(duration)
    nav.rotate_right_deg(90.0)
    if not nav.wait_for_valid_scan(("front", "right"), timeout_s=0.5):
        print("[WARN] LiDAR still settling after right turn")

    nav.timed_forward(cfg["sidekick_initial_forward_s"])
    if not nav.wait_for_valid_scan(("front", "left", "right"), timeout_s=0.5):
        print("[WARN] LiDAR still settling before channel alignment")
    nav.align_storage_channel(expect_front_wall=True)
    nav.centered_forward_until_front(front_thresh_mm=cfg["side_dead_end_mm"])

    nav.rotate_left_deg(180.0)
    if not nav.wait_for_valid_scan(("front", "left", "right"), timeout_s=0.5):
        print("[WARN] LiDAR still settling after 180° turn")
    nav.align_storage_channel(expect_front_wall=False)
    nav.move_to_front_distance(
        target_front_mm,
        tolerance_mm=target_tol_mm,
        maintain_center=False,
        expect_front_wall=False,
    )

    _run_arm_pick()

    nav.align_storage_channel(expect_front_wall=False)
    nav.centered_forward_until_front(front_thresh_mm=cfg["side_rejoin_front_mm"])

    nav.timed_forward(cfg["side_final_forward_s"])
    if not nav.wait_for_valid_scan(("front", "left", "right"), timeout_s=0.5):
        print("[WARN] LiDAR still settling before base turn")
    nav.rotate_left_deg(90.0)

    nav.follow_right_until_stop()

    nav.rotate_right_deg(180.0)

    _run_arm_release()
    try:
        drv.stop()
    except Exception:
        pass
    nav.hard_zero()
    buzzer.on()
    time.sleep(cfg.get("buzzer_end_s", 3.0))
    buzzer.off()
