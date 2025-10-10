#!/usr/bin/env python3
# test_chassis.py
# turn the wheel (ID 1) forward at a speed of 40

import time
from smbus2 import SMBus

I2C_PORT = 1			# bus
ADDR = 0x34				# adress for enginecontroller
REG_MOTOR1 = 51         # register of the wheel
TEST_PULSE = 40         # forward momentum
DURATION_S = 5.0        # duration in seconds

def main() -> None:
    print("[INFO] Starting the test (Engine 1).")
    try:
        with SMBus(I2C_PORT) as bus:
            bus.write_i2c_block_data(ADDR, REG_MOTOR1, [TEST_PULSE & 0xFF])
            time.sleep(DURATION_S)
            bus.write_i2c_block_data(ADDR, REG_MOTOR1, [0])
    except FileNotFoundError:
        print("[ERROR] I2C-Device not found (/dev/i2c-1).")
    except PermissionError:
        print("[ERROR] No permission for I2C-Access.")
    except Exception as exc:
        print(f"[ERROR] Error while writing to the enginecontroller: {exc}")
    else:
        print("[INFO] Test finished, engine stopped.")

if __name__ == "__main__":
    main()