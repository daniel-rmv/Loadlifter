#!/usr/bin/env python3
# test_arm.py
# opening, closing and opening the gripper again

import time
import serial

PORT = "/dev/serial0"
BAUD = 115200
SERVO_ID = 1             # gripper
MOVE_TIME_MS = 600
GRIPPER_OPEN_PULSE = 32
GRIPPER_CLOSE_PULSE = 607

def _checksum(buf: bytearray) -> int:
    total = sum(buf) - 0x55 - 0x55
    return (~total) & 0xFF

def build_move_packet(pulse: int, duration_ms: int) -> bytes:
    pulse = max(0, min(1000, pulse))
    duration_ms = max(0, min(30000, duration_ms))
    frame = bytearray([
        0x55,
        0x55,
        SERVO_ID & 0xFF,
        7,
        1,  # LOBOT_SERVO_MOVE_TIME_WRITE
        pulse & 0xFF,
        (pulse >> 8) & 0xFF,
        duration_ms & 0xFF,
        (duration_ms >> 8) & 0xFF,
    ])
    frame.append(_checksum(frame))
    return bytes(frame)

def send_gripper_command(pulse: int) -> None:
    pkt = build_move_packet(pulse, MOVE_TIME_MS)
    with serial.Serial(PORT, baudrate=BAUD, timeout=0.5, write_timeout=0.5) as ser:
        ser.write(pkt)
        time.sleep(MOVE_TIME_MS / 1000.0 + 0.2)

def main() -> None:
    print("[INFO] starting.")
    try:
        print("[INFO] -> opening…")
        send_gripper_command(GRIPPER_OPEN_PULSE)
        print("[INFO] -> closing…")
        send_gripper_command(GRIPPER_CLOSE_PULSE)
        print("[INFO] -> opening again…")
        send_gripper_command(GRIPPER_OPEN_PULSE)
    except serial.SerialException as exc:
    	print(f"[ERROR] Serial connection failed: {exc}")
	except Exception as exc:
    	print(f"[ERROR] Error sending gripper command: {exc}")
	else:
    	print("[INFO] Gripper test completed.")

if __name__ == "__main__":
    main()
