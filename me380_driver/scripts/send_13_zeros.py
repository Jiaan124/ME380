#!/usr/bin/env python3
import time
import serial


DEVICE = "/dev/ttyACM0"
BAUD = "115200"
LINE = "10 10 10 10 10 10 10 10 10 10 10 10 0\n"
REPEAT_COUNT = 1
DELAY_S = 0.05
STARTUP_DELAY_S = 3.0


def main() -> int:
    with serial.Serial(DEVICE, int(BAUD), timeout=0.05) as serial_dev:
        time.sleep(STARTUP_DELAY_S)
        serial_dev.reset_input_buffer()
        serial_dev.reset_output_buffer()
        for _ in range(REPEAT_COUNT):
            serial_dev.write(LINE.encode("ascii"))
            serial_dev.flush()
            print(f"TX: {LINE.strip()}")
            time.sleep(DELAY_S)

        print("Listening for serial messages. Press Ctrl+C to stop.")
        try:
            while True:
                msg = serial_dev.readline()
                if not msg:
                    continue
                decoded = msg.decode("ascii", errors="replace").strip()
                if decoded:
                    print(f"RX: {decoded}")
        except KeyboardInterrupt:
            print("\nStopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
