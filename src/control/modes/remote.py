#!/usr/bin/env python3
# remote.py
# Remote control mode.
# Author: Daniel Würmli

"""Remote control mode."""

import sys
import termios
import tty
import select
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from ...high_level.motor_system import MotorSystem
    from ...high_level.buzzer_system import BuzzerSystem


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def run(motors: "MotorSystem", buzzer: "BuzzerSystem", cfg: dict) -> None:
    """Handle keyboard-driven remote control."""
    try:
        REMOTE_POLL_S = cfg["REMOTE_POLL_S"]
        BASE_INITIAL_HOLD_S = cfg["BASE_INITIAL_HOLD_S"]
        ADAPTIVE_MIN_S = cfg["ADAPTIVE_MIN_S"]
        ADAPTIVE_MAX_S = cfg["ADAPTIVE_MAX_S"]
        ADAPTIVE_MARGIN_S = cfg["ADAPTIVE_MARGIN_S"]
        MOV_RELEASE_HOLD_S = cfg["MOV_RELEASE_HOLD_S"]
        BUZZER_RELEASE_HOLD_S = cfg["BUZZER_RELEASE_HOLD_S"]
        REMOTE_FWD_MM_S = cfg.get("REMOTE_FWD_MM_S", cfg["forward_mm_s"])
        REMOTE_YAW_PULSES = cfg["REMOTE_YAW_PULSES"]
        REMOTE_STRAFE_P = cfg["REMOTE_STRAFE_P"]
        REMOTE_YAW_SIGN = cfg["REMOTE_YAW_SIGN"]
        REMOTE_STRAFE_SIGN = cfg["REMOTE_STRAFE_SIGN"]
    except KeyError as exc:
        raise KeyError(f"Missing remote configuration key: {exc}") from exc

    print("\n--- REMOTE MODE ---")
    print("W/S = forward/backward   A/D = rotate left/right")
    print("N/M = strafe left/right   B = buzzer (hold)   SPACE = stop   Q = quit")
    print(f"(v≈{REMOTE_FWD_MM_S:.0f} mm/s, yaw={REMOTE_YAW_PULSES}, strafe={REMOTE_STRAFE_P})\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    axes = {
        "fwd": {"val": 0.0, "dead": 0.0, "ih": BASE_INITIAL_HOLD_S, "t0": 0.0, "repeat_seen": False},
        "yaw": {"val": 0, "dead": 0.0, "ih": BASE_INITIAL_HOLD_S, "t0": 0.0, "repeat_seen": False},
        "strf": {"val": 0, "dead": 0.0, "ih": BASE_INITIAL_HOLD_S, "t0": 0.0, "repeat_seen": False},
        "buzz": {"on": False, "dead": 0.0, "ih": BASE_INITIAL_HOLD_S, "t0": 0.0, "repeat_seen": False},
    }

    def _start_axis(ax_name, new_val, kind="mov"):
        ax = axes[ax_name]
        first = ((ax_name != "buzz" and ax["val"] == 0) or (ax_name == "buzz" and not ax["on"]))
        now = time.time()
        if ax_name != "buzz":
            ax["val"] = new_val
        else:
            if not ax["on"]:
                buzzer.on()
            ax["on"] = True
        if first:
            ax["t0"] = now
            ax["repeat_seen"] = False
            ax["dead"] = now + ax["ih"]
        else:
            ax["dead"] = now + (BUZZER_RELEASE_HOLD_S if kind == "buzz" else MOV_RELEASE_HOLD_S)

    def _maybe_adapt(ax_name, dt):
        ax = axes[ax_name]
        new_ih = clamp(dt + ADAPTIVE_MARGIN_S, ADAPTIVE_MIN_S, ADAPTIVE_MAX_S)
        ax["ih"] = 0.5 * ax["ih"] + 0.5 * new_ih

    def _bridge_gap(ax_name):
        ax = axes[ax_name]
        now = time.time()
        if not ax["repeat_seen"] and (now - ax["t0"]) < ADAPTIVE_MAX_S:
            ax["dead"] = now + 0.05
            return True
        return False

    try:
        last_apply = 0.0
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], REMOTE_POLL_S)

            if rlist:
                c = sys.stdin.read(1).lower()
                if c == "q":
                    break
                if c == " ":
                    axes["fwd"]["val"] = 0.0
                    axes["yaw"]["val"] = 0
                    axes["strf"]["val"] = 0
                    axes["fwd"]["dead"] = axes["yaw"]["dead"] = axes["strf"]["dead"] = 0.0
                    if axes["buzz"]["on"]:
                        buzzer.off()
                    axes["buzz"]["on"] = False
                    axes["buzz"]["dead"] = 0.0
                    print("[STOP]")
                    continue

                if c == "w":
                    _start_axis("fwd", +abs(REMOTE_FWD_MM_S), "mov")
                elif c == "s":
                    _start_axis("fwd", -abs(REMOTE_FWD_MM_S), "mov")
                elif c == "a":
                    _start_axis("yaw", +abs(REMOTE_YAW_PULSES) * REMOTE_YAW_SIGN, "mov")
                elif c == "d":
                    _start_axis("yaw", -abs(REMOTE_YAW_PULSES) * REMOTE_YAW_SIGN, "mov")
                elif c == "n":
                    _start_axis("strf", +abs(REMOTE_STRAFE_P) * REMOTE_STRAFE_SIGN, "mov")
                elif c == "m":
                    _start_axis("strf", -abs(REMOTE_STRAFE_P) * REMOTE_STRAFE_SIGN, "mov")
                elif c == "b":
                    _start_axis("buzz", None, "buzz")

                now = time.time()
                for key, keys in (("fwd", ("w", "s")), ("yaw", ("a", "d")), ("strf", ("n", "m"))):
                    if c in keys:
                        ax = axes[key]
                        if ax["t0"] > 0 and not ax["repeat_seen"]:
                            dt = now - ax["t0"]
                            if 0.10 < dt < 1.20:
                                ax["repeat_seen"] = True
                                _maybe_adapt(key, dt)

            now = time.time()
            for name in ("fwd", "yaw", "strf"):
                ax = axes[name]
                if ax["val"] != 0 and ax["dead"] > 0 and now > ax["dead"]:
                    if not _bridge_gap(name):
                        ax["val"] = 0 if name != "fwd" else 0.0
                        ax["dead"] = 0.0
            bz = axes["buzz"]
            if bz["on"] and time.time() > bz["dead"]:
                buzzer.off()
                bz["on"] = False
                bz["dead"] = 0.0

            if (time.time() - last_apply) >= REMOTE_POLL_S:
                motors.drive_full(
                    forward_mm_s=axes["fwd"]["val"],
                    strafe_pulses=axes["strf"]["val"],
                    yaw_pulses=axes["yaw"]["val"],
                )
                last_apply = time.time()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            if axes["buzz"]["on"]:
                buzzer.off()
        except Exception:  # pragma: no cover
            pass
        motors.stop()
