#!/usr/bin/env python3
# follow_route.py
# Follow route behaviour.
# Author: Daniel Würmli

"""Follow route behaviour."""

import time


def run(nav, cfg: dict, buzzer, drv) -> None:
    """Execute the follow-route workflow."""
    require_rearm_next = False
    while True:
        event = nav.follow_left_until_right_open_or_front(
            right_open_mm=cfg["right_open_mm"],
            require_rearm=require_rearm_next or cfg["right_open_require_rearm"],
            rearm_below_mm=cfg.get("right_open_rearm_mm"),
        )

        if event == "front_stop":
            time.sleep(0.15)
            nav.rotate_left_deg(180.0)
            time.sleep(0.10)
            nav.follow_right_until_stop()
            time.sleep(0.15)
            nav.rotate_left_deg(180.0)
            time.sleep(0.10)
            try:
                drv.stop()
            except Exception:
                pass
            nav.hard_zero()
            time.sleep(0.1)
            buzzer.on()
            time.sleep(cfg.get("buzzer_end_s", 3.0))
            buzzer.off()
            break

        duration = cfg["extra_forward_after_open_s"] if not require_rearm_next else cfg.get("extra_forward_after_open_repeat_s", cfg["extra_forward_after_open_s"])
        nav.timed_forward(duration)
        nav.rotate_right_deg(90.0)
        time.sleep(0.10)
        if not nav.wait_for_valid_scan(("front", "right"), timeout_s=0.5):
            print("[WARN] LiDAR still settling after right turn")

        nav.timed_forward(cfg["sidekick_initial_forward_s"])
        if not nav.wait_for_valid_scan(("front", "left", "right"), timeout_s=0.5):
            print("[WARN] LiDAR still settling before centered-forward")
        nav.align_storage_channel(expect_front_wall=True)
        nav.centered_forward_until_front(front_thresh_mm=cfg["side_dead_end_mm"])

        nav.rotate_left_deg(180.0)
        time.sleep(0.10)
        if not nav.wait_for_valid_scan(("front", "right"), timeout_s=0.5):
            print("[WARN] LiDAR still settling after 180° turn")
        nav.align_storage_channel(expect_front_wall=False)
        nav.centered_forward_until_front(front_thresh_mm=cfg["side_rejoin_front_mm"])

        nav.timed_forward(cfg["side_final_forward_s"])
        if not nav.wait_for_valid_scan(("front", "left", "right"), timeout_s=0.5):
            print("[WARN] LiDAR still settling before exit turn")
        nav.rotate_right_deg(90.0)
        time.sleep(0.10)

        require_rearm_next = True
