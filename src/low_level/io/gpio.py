#!/usr/bin/env python3
# gpio.py
# GPIO abstraction used by low-level drivers.
# Author: Daniel Würmli

"""GPIO abstraction used by low-level drivers.

If no real GPIO backend is available, a no-op implementation is used so modules remain importable on macOS/CI environments.
"""

from typing import Optional

try:
    import RPi.GPIO as _GPIO  # type: ignore
except Exception:  # pragma: no cover – RPi.GPIO is usually not present locally
    _GPIO = None


class GPIOPin:
    """Minimal wrapper around a single GPIO output pin."""

    def __init__(self, pin: int, active_high: bool = True):
        self.pin = int(pin)
        self.active_high = bool(active_high)
        self._backend = _GPIO
        self._state: Optional[bool] = None

        if self._backend is not None:
            self._backend.setwarnings(False)
            self._backend.setmode(self._backend.BCM)
            self._backend.setup(self.pin, self._backend.OUT, initial=self._to_hw(False))

    def _to_hw(self, state: bool) -> int:
        return self._backend.HIGH if self.active_high == state else self._backend.LOW

    def set(self, state: bool) -> None:
        """Set the pin to True/False. Without a backend the state is cached only."""
        self._state = bool(state)
        if self._backend is not None:
            self._backend.output(self.pin, self._to_hw(state))

    def high(self) -> None:
        self.set(True)

    def low(self) -> None:
        self.set(False)

    def cleanup(self) -> None:
        if self._backend is not None:
            try:
                self._backend.cleanup(self.pin)
            except Exception:
                pass


__all__ = ["GPIOPin"]
