#!/usr/bin/env python3
# servo_controller.py
# Low-level ServoController handling half-duplex UART for LOBOT/Bus servos.
# Author: Daniel Würmli

"""
Low-level ServoController providing half-duplex UART access for LOBOT/Bus servos.
- Minimal, clean layer: packet construction, TX/RX, and primitive commands.
- No high-level pose or motion logic included.
"""

from typing import Optional, Tuple
import time, struct

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

import serial

# LOBOT Commands
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31

DEFAULT_BAUD = 115200
RX_PIN = 4
TX_PIN = 27

class ServoController:
    def __init__(self, port: str = '/dev/serial0', baud: int = DEFAULT_BAUD,
                 timeout: float = 0.1, write_timeout: float = 0.8,
                 use_gpio: bool = True, rx_pin: int = RX_PIN, tx_pin: int = TX_PIN):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout, write_timeout=write_timeout)
        self.use_gpio = bool(use_gpio and GPIO is not None)
        self.rx_pin = rx_pin
        self.tx_pin = tx_pin
        if self.use_gpio:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.rx_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.tx_pin, GPIO.OUT, initial=GPIO.LOW)

    # --- Half-duplex direction switching ---
    def _port_write(self):
        if self.use_gpio:
            GPIO.output(self.rx_pin, GPIO.LOW)
            GPIO.output(self.tx_pin, GPIO.HIGH)

    def _port_read(self):
        if self.use_gpio:
            GPIO.output(self.rx_pin, GPIO.HIGH)
            GPIO.output(self.tx_pin, GPIO.LOW)

    # --- Packet utilities ---
    @staticmethod
    def _checksum(buf: bytes) -> int:
        s = sum(buf) - 0x55 - 0x55
        return (~s) & 0xFF

    def _packet(self, sid: int, cmd: int, dat1=None, dat2=None) -> bytes:
        b = bytearray([0x55, 0x55, sid])
        if dat1 is None and dat2 is None:
            b += bytes([3, cmd])
        elif dat2 is None:
            b += bytes([4, cmd, dat1 & 0xFF])
        else:
            if isinstance(dat1, tuple):
                d1 = dat1[0] & 0xFF
                d2 = (dat1[0] >> 8) & 0xFF
            else:
                d1 = dat1 & 0xFF
                d2 = (dat1 >> 8) & 0xFF
            d3 = dat2 & 0xFF
            d4 = (dat2 >> 8) & 0xFF
            b += bytes([7, cmd, d1, d2, d3, d4])
        b.append(self._checksum(b))
        return bytes(b)

    # --- TX/RX ---
    def write_cmd(self, sid: Optional[int], cmd: int, dat1=None, dat2=None) -> None:
        sid = 0xFE if sid is None else int(sid)
        pkt = self._packet(sid, cmd, dat1, dat2)
        self._port_write()
        self.ser.write(pkt)
        time.sleep(0.00034)

    def read_cmd(self, sid: Optional[int], cmd: int) -> None:
        sid = 0xFE if sid is None else int(sid)
        pkt = self._packet(sid, cmd)
        self._port_write()
        self.ser.write(pkt)
        time.sleep(0.00034)

    def get_reply(self, expect_cmd: int):
        """Parse a simple response frame; returns int, (int, int), or None."""
        self.ser.flushInput()
        self._port_read()
        time.sleep(0.005)
        if self.ser.in_waiting == 0:
            return None
        data = self.ser.read(self.ser.in_waiting)
        if len(data) < 7:
            return None
        try:
            idx = data.index(b"\x55\x55")
        except ValueError:
            return None
        frame = data[idx:]
        if len(frame) < 7:
            return None
        if frame[2] == 0x55 and frame[3] == 0x55:
            frame = frame[1:]
        if frame[4] < 3:
            return None
        length = frame[4]
        cmd = frame[5 - 1 + 2]  # robust, aber effektiv expect_cmd pruefen reicht
        if cmd != expect_cmd:
            return None
        if length == 3:
            return True
        if length == 4:
            return struct.unpack('b', frame[5:6])[0]
        if length == 5:
            val = frame[5] | (frame[6] << 8)
            return struct.unpack('<h', struct.pack('<H', val))[0]
        if length == 7:
            v1 = frame[5] | (frame[6] << 8)
            v2 = frame[7] | (frame[8] << 8)
            return (struct.unpack('<h', struct.pack('<H', v1))[0],
                    struct.unpack('<h', struct.pack('<H', v2))[0])
        return None

    # --- Public Low-Level API ---
    def move_time_write(self, sid: int, pulse: int, t_ms: int) -> None:
        pulse = max(0, min(1000, int(pulse)))
        t_ms  = max(0, min(30000, int(t_ms)))
        self.write_cmd(sid, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, t_ms)

    def stop(self, sid: Optional[int] = None) -> None:
        self.write_cmd(sid, LOBOT_SERVO_MOVE_STOP)

    def read_pos(self, sid: int):
        self.read_cmd(sid, LOBOT_SERVO_POS_READ)
        return self.get_reply(LOBOT_SERVO_POS_READ)

    def unload(self, sid: int) -> None:
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

    def load(self, sid: int) -> None:
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 1)

    def cleanup(self) -> None:
        try:
            if self.use_gpio and GPIO is not None:
                GPIO.cleanup()
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
