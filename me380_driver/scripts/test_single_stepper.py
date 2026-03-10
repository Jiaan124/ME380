#!/usr/bin/env python3
"""
Test script for a single stepper motor (step + dir).
Drives one axis with configurable step/dir GPIO pins. Optional verbose output
of each step and dir state. Works on Raspberry Pi with RPi.GPIO; without it,
prints step/dir actions only (no hardware).
"""

import argparse
import sys
import time

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    GPIO = None

STEP_PULSE_US = 5
STEP_PULSE_S = STEP_PULSE_US / 1e6


def do_step(step_pin: int, dir_pin: int, direction: int, use_gpio: bool, verbose: bool) -> None:
    """Output one step: set dir, then pulse step. direction: 1 = CW, -1 = CCW."""
    if use_gpio and HAS_GPIO:
        GPIO.output(dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(STEP_PULSE_S)
        GPIO.output(step_pin, GPIO.LOW)
    if verbose:
        dir_val = 1 if direction > 0 else 0
        print(f"  dir={dir_val} step=1 -> step=0")


def run_test(
    step_pin: int,
    dir_pin: int,
    steps_forward: int,
    steps_back: int,
    steps_per_sec: float,
    verbose: bool,
    use_gpio: bool,
) -> None:
    min_interval = 1.0 / steps_per_sec if steps_per_sec > 0 else 0.001

    if use_gpio and HAS_GPIO:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(step_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)

    try:
        total = steps_forward + steps_back
        print(f"Test: {steps_forward} steps forward, then {steps_back} steps back (dir=1 then dir=0)")
        if use_gpio and HAS_GPIO:
            print(f"Step pin GPIO {step_pin}, Dir pin GPIO {dir_pin}")
        else:
            print("(No RPi.GPIO: printing step/dir only)")

        for i in range(steps_forward):
            do_step(step_pin, dir_pin, 1, use_gpio, verbose)
            if verbose and (i + 1) % 10 == 0:
                print(f"  ... {i + 1}/{steps_forward} forward")
            time.sleep(min_interval)

        for i in range(steps_back):
            do_step(step_pin, dir_pin, -1, use_gpio, verbose)
            if verbose and (i + 1) % 10 == 0:
                print(f"  ... {i + 1}/{steps_back} back")
            time.sleep(min_interval)

        print("Done.")
    finally:
        if use_gpio and HAS_GPIO:
            GPIO.cleanup()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Test a single stepper motor (step + dir output)."
    )
    parser.add_argument(
        "--step-pin",
        type=int,
        default=17,
        help="BCM GPIO number for STEP (default: 17, J1)",
    )
    parser.add_argument(
        "--dir-pin",
        type=int,
        default=27,
        help="BCM GPIO number for DIR (default: 27, J1)",
    )
    parser.add_argument(
        "-f", "--forward",
        type=int,
        default=100,
        help="Number of steps forward (dir=1) (default: 100)",
    )
    parser.add_argument(
        "-b", "--back",
        type=int,
        default=100,
        help="Number of steps back (dir=0) (default: 100)",
    )
    parser.add_argument(
        "--steps-per-sec",
        type=float,
        default=500.0,
        help="Step rate in steps per second (default: 500)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Print each step/dir transition",
    )
    parser.add_argument(
        "--no-gpio",
        action="store_true",
        help="Do not use hardware GPIO; only print step/dir (for testing without Pi)",
    )
    args = parser.parse_args()

    use_gpio = not args.no_gpio
    if use_gpio and not HAS_GPIO:
        print("RPi.GPIO not available. Use --no-gpio to run in print-only mode.", file=sys.stderr)
        print("On Raspberry Pi, install with:  pip3 install RPi.GPIO   (or: sudo apt install python3-rpi.gpio)", file=sys.stderr)
        print("Then run with:  sudo python3 ...  if GPIO access is restricted.", file=sys.stderr)
        return 1

    run_test(
        step_pin=args.step_pin,
        dir_pin=args.dir_pin,
        steps_forward=args.forward,
        steps_back=args.back,
        steps_per_sec=args.steps_per_sec,
        verbose=args.verbose,
        use_gpio=use_gpio,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
