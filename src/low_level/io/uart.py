#!/usr/bin/env python3
# uart.py
# UART abstraction built on top of pyserial.
# Author: Daniel Würmli

"""UART abstraction built on top of pyserial."""

from typing import Optional

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None


class UARTPort:
    """Lightweight wrapper around `serial.Serial` to simplify test stubbing."""

    def __init__(self, device: str, baudrate: int, timeout: float = 0.1, write_timeout: float = 0.8):
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._ser: Optional["serial.Serial"] = None

        if serial is not None:
            self._ser = serial.Serial(
                device,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=write_timeout,
            )

    @property
    def handle(self):
        if self._ser is None:
            raise RuntimeError("UART backend not available on this platform.")
        return self._ser

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


__all__ = ["UARTPort"]
