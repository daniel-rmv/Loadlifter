#!/usr/bin/env python3
# follow_wall.py
# Follow wall mode behaviour.
# Author: Daniel Würmli

"""Follow wall mode behaviour."""

import time


def run(nav, cfg: dict, buzzer, drv) -> None:
    """Execute the follow-wall routine."""
    nav.follow_left_until_stop()
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
