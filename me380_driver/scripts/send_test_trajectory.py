#!/usr/bin/env python3
"""Send a test N=10 trajectory to the ME380 controller over serial."""

import time

import serial

PORT = "/dev/cu.usbmodem1101"
BAUD = 115200
READ_TIMEOUT_S = 0.5
STARTUP_DELAY_S = 2.0
FINISH_TIMEOUT_S = 30.0
ROW_PERIOD_S = 0.02


def build_rows():
    rows = []
    # delta_j1..j4 = 3 deg, delta_j5..j6 = 0
    # vel_j1..j4 ramps 0.2 -> 2.0 deg/s, vel_j5..j6 = 0
    # gripper = 0
    for i in range(1, 11):
        v = 0.2 * i
        row = f"3 3 3 3 0 0 {v:.1f} {v:.1f} {v:.1f} {v:.1f} 0 0 0"
        rows.append(row)
    return rows


def main():
    rows = build_rows()

    with serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT_S) as ser:
        time.sleep(STARTUP_DELAY_S)
        ser.reset_input_buffer()

        # Stream one 13-value row per 20 ms sample.
        for i, row in enumerate(rows, start=1):
            msg = row + "\n"
            ser.write(msg.encode("utf-8"))
            print(f"> row {i}/{len(rows)}: {row}")
            time.sleep(ROW_PERIOD_S)

            # Drain any immediate firmware responses (e.g., ERR: buffer full).
            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                print(f"< {line}")
                if line.startswith("ERR"):
                    raise RuntimeError(f"Controller error: {line}")

        deadline = time.time() + FINISH_TIMEOUT_S

        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(f"< {line}")

            if line == "FINISHED":
                print("Trajectory complete.")
                return
            if line.startswith("ERR"):
                raise RuntimeError(f"Controller error: {line}")

        raise TimeoutError("Timed out waiting for FINISHED.")


if __name__ == "__main__":
    main()
