#!/usr/bin/env python3
"""
Test script for a single stepper motor (step + dir).
Edit the variables below, then run. Uses RPi.GPIO on Raspberry Pi; use --no-gpio to print only.
"""

import sys
import time

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    GPIO = None

# ----- Edit these -----
NUM_STEPS = 300
TIME_BETWEEN_STEPS_S = 0.1  # seconds between steps (0.01 = 100 steps/sec)
STEP_PIN = 17   # BCM GPIO for step (J1)
DIR_PIN = 27    # BCM GPIO for dir (J1)
STEP_PULSE = 0.005
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
            print("pulsed high")
            time.sleep(STEP_PULSE)
            GPIO.output(STEP_PIN, GPIO.LOW)
            print("pulsed low")

            time.sleep(STEP_PULSE)

        # Back (dir=0)
        if use_gpio:
            GPIO.output(DIR_PIN, GPIO.LOW)
        for _ in range(NUM_STEPS):
            if use_gpio:
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(STEP_PULSE)
                GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_PULSE)

        print("Done.")
    finally:
        if use_gpio:
            GPIO.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
