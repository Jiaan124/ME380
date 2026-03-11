#!/usr/bin/env python3
"""
Test a single servo. Matches esp32_driver.cpp: PWM 500–2500 us, center 1500 us.
Uses RPi.GPIO. Run: sudo python3 servo_test.py
"""

import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# From esp32_driver.cpp
PWM_CENTER = 1500
PWM_MIN = 500
PWM_MAX = 2500
SERVO_RANGE_DEG = 135.0
US_PER_DEG = 2000.0 / 270.0

# Which servo to test (BCM GPIO): A=12, B=13, EE=18
SERVO_PIN = 12

# Test settings
HOLD_TIME_S = 1.5
SERVO_HZ = 50


def pulse_us_to_duty(pulse_us: float) -> float:
    """50 Hz, 20 ms period -> duty % for RPi.GPIO."""
    return (pulse_us / 20000.0) * 100.0


def main() -> int:
    if GPIO is None:
        print("RPi.GPIO not available. Install: pip3 install RPi.GPIO", file=sys.stderr)
        print("Then run with: sudo python3 servo_test.py", file=sys.stderr)
        return 1

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT, initial=GPIO.LOW)

    pwm = GPIO.PWM(SERVO_PIN, SERVO_HZ)
    pwm.start(pulse_us_to_duty(PWM_CENTER))

    def set_us(us: float) -> None:
        us = max(PWM_MIN, min(PWM_MAX, us))
        pwm.ChangeDutyCycle(pulse_us_to_duty(us))

    max_us = min(PWM_MAX, int(PWM_CENTER + SERVO_RANGE_DEG * US_PER_DEG))
    min_us = max(PWM_MIN, int(PWM_CENTER - SERVO_RANGE_DEG * US_PER_DEG))

    try:
        print("Testing servo on GPIO {}".format(SERVO_PIN))
        print("Center")
        time.sleep(HOLD_TIME_S)
        set_us(max_us)
        print("Max")
        time.sleep(HOLD_TIME_S)
        set_us(min_us)
        print("Min")
        time.sleep(HOLD_TIME_S)
        set_us(PWM_CENTER)
        print("Center")
        time.sleep(HOLD_TIME_S)
        print("Done.")
    finally:
        pwm.stop()
        GPIO.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
