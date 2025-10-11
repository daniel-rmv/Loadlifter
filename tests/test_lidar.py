#!/usr/bin/env python3
# lidar_test.py
# Reads frames, parses angle/distance/intensity, checks CRC, prints the first few points.
# Author: Daniel Würmli

import serial
import struct
import time
from collections import deque

PORT = "/dev/ttyUSB0"   # USB serial device 
BAUD = 230400           # baudrate
TIMEOUT = 0.05

# CRC8 table copied from the original C++ driver (same ordering)
CRC8_TABLE = [
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x15,0x58,0x8f,0xc2,0x6c,0x21,0xf6,0xbb,0xe7,0xaa,0x7d,0x30,0x9e,0xd3,0x04,0x49,
    0xbc,0xf1,0x26,0x6b,0xc5,0x88,0x5f,0x12,0x4e,0x03,0xd4,0x99,0x37,0x7a,0xad,0xe0,
    0x2a,0x67,0xb0,0xfd,0x53,0x1e,0xc9,0x84,0xd8,0x95,0x42,0x0f,0xa1,0xec,0x3b,0x76,
    0x83,0xce,0x19,0x54,0xfa,0xb7,0x60,0x2d,0x71,0x3c,0xeb,0xa6,0x08,0x45,0x92,0xdf,
    0x3f,0x72,0xa5,0xe8,0x46,0x0b,0xdc,0x91,0xcd,0x80,0x57,0x1a,0xb4,0xf9,0x2e,0x63,
    0x96,0xdb,0x0c,0x41,0xef,0xa2,0x75,0x38,0x64,0x29,0xfe,0xb3,0x1d,0x50,0x87,0xca,
    0x54,0x19,0xce,0x83,0x2d,0x60,0xb7,0xfa,0xa6,0xeb,0x3c,0x71,0xdf,0x92,0x45,0x08,
    0xfd,0xb0,0x67,0x2a,0x84,0xc9,0x1e,0x53,0x0f,0x42,0x95,0xd8,0x76,0x3b,0xec,0xa1,
    0x41,0x0c,0xdb,0x96,0x38,0x75,0xa2,0xef,0xb3,0xfe,0x29,0x64,0xca,0x87,0x50,0x1d,
    0xe8,0xa5,0x72,0x3f,0x91,0xdc,0x0b,0x46,0x1a,0x57,0x80,0xcd,0x63,0x2e,0xf9,0xb4,
    0x7e,0x33,0xe4,0xa9,0x07,0x4a,0x9d,0xd0,0x8c,0xc1,0x16,0x5b,0xf5,0xb8,0x6f,0x22,
    0xd7,0x9a,0x4d,0x00,0xae,0xe3,0x34,0x79,0x25,0x68,0xbf,0xf2,0x5c,0x11,0xc6,0x8b,
    0x6b,0x26,0xf1,0xbc,0x12,0x5f,0x88,0xc5,0x99,0xd4,0x03,0x4e,0xe0,0xad,0x7a,0x37,
    0xc2,0x8f,0x58,0x15,0xbb,0xf6,0x21,0x6c,0x30,0x7d,0xaa,0xe7,0x49,0x04,0xd3,0x9e
]


def crc8(data: bytes, init_val=0x00):
    crc = init_val
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def sync_to_header(ser):
    while True:
        b = ser.read(1)
        if not b:
            return False
        if b[0] == 0x54:
            return True


def parse_frame(rest: bytes):
    # buffer starts with ver_len
    ver_len = rest[0]
    count = ver_len & 0x1F  # low 5 bits = number of points
    if count == 0 or count > 40:
        return None

    need = 1 + 2 + 2 + count * 3 + 2 + 2 + 1
    if len(rest) < need:
        return None

    speed = struct.unpack_from("<H", rest, 1)[0] / 100.0
    start_raw = struct.unpack_from("<H", rest, 3)[0]
    start_angle = start_raw / 100.0

    off = 5
    dist_mm = []
    inten = []
    for _ in range(count):
        d = struct.unpack_from("<H", rest, off)[0]
        dist_mm.append(d)
        inten.append(rest[off + 2])
        off += 3

    end_raw = struct.unpack_from("<H", rest, off)[0]
    end_angle = end_raw / 100.0
    off += 2

    timestamp = struct.unpack_from("<H", rest, off)[0]
    off += 2

    crc_byte = rest[off]

    # CRC check includes header byte (0x54) plus all payload bytes before CRC
    calc = crc8(bytes([0x54]) + rest[:-1])

    # angle interpolation
    if count > 1:
        diff = end_angle - start_angle
        # handle wrap-around
        if diff < -180:
            diff += 360
        elif diff > 180:
            diff -= 360
        step = diff / (count - 1)
    else:
        step = 0.0

    angles = [(start_angle + i * step) % 360 for i in range(count)]

    return {
        "speed_dps": speed,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "angles_deg": angles,
        "dist_mm": dist_mm,
        "intensity": inten,
        "timestamp_ms": timestamp,
        "crc_ok": (calc == crc_byte),
        "points": count,
    }


def main():
    print(f"[INFO] -> Opening {PORT} @ {BAUD} baud ...")
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    frames = 0
    last_log = time.time()
    recent = deque(maxlen=3)

    try:
        while True:
            if not sync_to_header(ser):
                print("[ERROR] No header found (timeout).")
                continue

            # read minimal header to discover point count
            head_rest = read_exact(ser, 1 + 2 + 2)  # ver_len + speed + start angle
            if head_rest is None:
                continue

            ver_len = head_rest[0]
            count = ver_len & 0x1F
            need = count * 3 + 2 + 2 + 1
            body = read_exact(ser, need)
            if body is None:
                continue

            frame = parse_frame(head_rest + body)
            if not frame:
                continue

            frames += 1
            recent.append(frame)

            if time.time() - last_log >= 0.5:
                f = recent[-1]
                print(
                    f"[INFO] \nFrame #{frames} | pts={f['points']:02d} | speed={f['speed_dps']:.1f}°/s | "
                    f"[INFO] {f['start_angle']:.2f}° -> {f['end_angle']:.2f}° | CRC={'ok' if f['crc_ok'] else 'BAD'}"
                )
                for i in range(min(6, f['points'])):
                    print(
                        f"[INFO] {i:02d}: angle={f['angles_deg'][i]:6.2f}°  dist={f['dist_mm'][i]:5d} mm  I={f['intensity'][i]:3d}"
                    )
                last_log = time.time()

    finally:
        ser.close()


if __name__ == "__main__":
    main()
