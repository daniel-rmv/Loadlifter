#!/usr/bin/env python3
# pwm.py
# PWM abstraction shared by low-level drivers.
# Author: Daniel Würmli

"""PWM abstraction shared by low-level drivers."""

from typing import Optional

try:
    import RPi.GPIO as _GPIO  # type: ignore
except Exception:  # pragma: no cover
    _GPIO = None


class PWMChannel:
    """Simple PWM channel wrapper."""

    def __init__(self, pin: int, frequency_hz: float):
        self.pin = int(pin)
        self.frequency_hz = float(frequency_hz)
        self._backend = _GPIO
        self._pwm = None
        self._duty = 0.0

        if self._backend is not None:
            self._backend.setwarnings(False)
            self._backend.setmode(self._backend.BCM)
            self._backend.setup(self.pin, self._backend.OUT, initial=self._backend.LOW)
            self._pwm = self._backend.PWM(self.pin, self.frequency_hz)
            self._pwm.start(0.0)

    def change_frequency(self, frequency_hz: float) -> None:
        self.frequency_hz = float(frequency_hz)
        if self._pwm is not None:
            self._pwm.ChangeFrequency(self.frequency_hz)

    def set_duty_cycle(self, percent: float) -> None:
        self._duty = max(0.0, min(100.0, float(percent)))
        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(self._duty)

    def stop(self) -> None:
        if self._pwm is not None:
            self._pwm.stop()
            self._pwm = None

    def cleanup(self) -> None:
        self.stop()
        if self._backend is not None:
            try:
                self._backend.cleanup(self.pin)
            except Exception:
                pass


__all__ = ["PWMChannel"]
