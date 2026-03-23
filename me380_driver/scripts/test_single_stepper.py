#!/usr/bin/env python3
"""
Test script for a single stepper motor (step + dir).
Edit the variables below, then run. Uses RPi.GPIO on Raspberry Pi; use --no-gpio to print only.
"""

import sys
import time

"""
  X (J1)     Step 17 -> D2,  Dir 27 -> D5
  Y (J2)     Step 22 -> D3,  Dir 23 -> D6
  Z (J3)     Step 24 -> D4,  Dir 25 -> D7
  A (J4)     Step  5 -> D12, Dir  6 -> D13
"""

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    GPIO = None

# ----- Edit 
# these -----
NUM_STEPS = 4000
TIME_BETWEEN_STEPS_MS = 0.2  # seconds between steps (0.01 = 100 steps/sec)
STEP_PIN = 22
DIR_PIN = 23
STEP_PULSE = 0.0005

# GPIO_J1_STEP, GPIO_J1_DIR = 17, 27
# GPIO_J2_STEP, GPIO_J2_DIR = 22, 23
# GPIO_J3_STEP, GPIO_J3_DIR = 24, 25
# GPIO_J4_STEP, GPIO_J4_DIR = 5, 6
# ----------------------



def main():
    use_gpio = "--no-gpio" not in sys.argv and HAS_GPIO
    if not use_gpio and "--no-gpio" not in sys.argv and not HAS_GPIO:
        print("RPi.GPIO not available. Use --no-gpio to print only.", file=sys.stderr)
        print("Install for root: sudo pip3 install RPi.GPIO", file=sys.stderr)
        return 1

    if use_gpio:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

    try:
        # Forward (dir=1)
        GPIO.output(DIR_PIN, GPIO.HIGH)
        for _ in range(NUM_STEPS):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            # print("pulsed high")
            time.sleep(STEP_PULSE/1000)
            GPIO.output(STEP_PIN, GPIO.LOW)
            # print("pulsed low")

            time.sleep(TIME_BETWEEN_STEPS_MS/1000)

        # Back (dir=0)
        if use_gpio:
            GPIO.output(DIR_PIN, GPIO.LOW)
        for _ in range(NUM_STEPS):
            if use_gpio:
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(STEP_PULSE/1000)
                GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(TIME_BETWEEN_STEPS_MS/1000)

        print("Done.")
    finally:
        if use_gpio:
            GPIO.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
