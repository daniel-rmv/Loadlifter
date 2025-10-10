#!/usr/bin/env python3
# motor_system.py
# High-level motor coordination service.
# Author: Daniel Würmli

from ..low_level.motor_controller import MotorController
import time


class MotorSystem:
    def __init__(self, ramp_step=3, g_vy=0.25):
        self._ll = MotorController(ramp_step=ramp_step, g_vy=g_vy)

    # --------- Driving APIs ----------
    def drive(self, forward_mm_s: float = 0.0, yaw_pulses: int = 0):
        """Drive forward while applying yaw (no strafe)."""
        base = self._ll.base_mag_from_speed(forward_mm_s)
        self._ll.command(base_fwd=base, strafe=0, yaw=int(yaw_pulses))

    def drive_full(self, forward_mm_s: float = 0.0, strafe_pulses: int = 0, yaw_pulses: int = 0):
        """Full control over forward, strafe, and yaw simultaneously."""
        base = self._ll.base_mag_from_speed(forward_mm_s)
        self._ll.command(base_fwd=base, strafe=int(strafe_pulses), yaw=int(yaw_pulses))

    def drive_forward(self, speed_mm_s: float):
        self.drive(forward_mm_s=speed_mm_s, yaw_pulses=0)

    def rotate_left(self, yaw_pulses=12):
        self._ll.command(base_fwd=0, strafe=0, yaw=+abs(int(yaw_pulses)))

    def rotate_right(self, yaw_pulses=12):
        self._ll.command(base_fwd=0, strafe=0, yaw=-abs(int(yaw_pulses)))

    def strafe_left(self, mag: int):
        """Strafe to the left. Swap the sign here if the direction is inverted."""
        self._ll.command(base_fwd=0, strafe=+abs(int(mag)), yaw=0)

    def strafe_right(self, mag: int):
        """Strafe to the right."""
        self._ll.command(base_fwd=0, strafe=-abs(int(mag)), yaw=0)

    # --- Calibrated rotation / braking helpers ---
    def yaw_spin(self, yaw_pulses: int):
        """Pure yaw rotation (no forward motion). Positive = left, negative = right."""
        self._ll.command(base_fwd=0, strafe=0, yaw=int(yaw_pulses))

    def brake_yaw(self, opposite_pulses=8, duration=0.06):
        """Apply a short counter impulse to reduce inertia."""
        if opposite_pulses > 0:
            self.rotate_right(opposite_pulses)
        elif opposite_pulses < 0:
            self.rotate_left(abs(opposite_pulses))
        time.sleep(max(0.0, float(duration)))
        self.stop()

    def stop(self):
        self._ll.stop_all()
        time.sleep(0.02)
