#!/usr/bin/env python3
"""
Hold one or more servos at a fixed pulse width via pigpio until Ctrl+C.

Pulse width 500–2500 µs (same as esp32_driver / driver_node_py).

Requires pigpio daemon:  sudo pigpiod
Run:  python3 servo_test.py
       python3 servo_test.py --pins 12,13 --hold-position 1500

Install: pip3 install pigpio   (client library; daemon is separate)
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import pigpio
except ImportError:
    pigpio = None

PWM_MIN = 500
PWM_MAX = 2500


def parse_pins(s: str) -> list[int]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("need at least one pin")
    out: list[int] = []
    for p in parts:
        try:
            n = int(p, 10)
        except ValueError as e:
            raise argparse.ArgumentTypeError("invalid pin: %r" % p) from e
        if n < 0 or n > 27:
            raise argparse.ArgumentTypeError("pin out of range: %d" % n)
        out.append(n)
    return out


def clamp_us(us: int) -> int:
    return int(max(PWM_MIN, min(PWM_MAX, us)))


def set_all(pi: pigpio.pi, pins: list[int], us: int) -> None:
    for g in pins:
        pi.set_servo_pulsewidth(g, us)


def off_all(pi: pigpio.pi, pins: list[int]) -> None:
    for g in pins:
        pi.set_servo_pulsewidth(g, 0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hold servos at one pulse width (pigpio) until Ctrl+C."
    )
    parser.add_argument(
        "--pins",
        type=str,
        default="12,13,18",
        help="BCM GPIO pins, comma-separated (default: 12,13,18 for ME380)",
    )
    parser.add_argument(
        "--hold-position",
        type=int,
        default=1500,
        metavar="US",
        help="Servo pulse width in microseconds, %d–%d (default: 1500 center)"
        % (PWM_MIN, PWM_MAX),
    )
    args = parser.parse_args()
    pins = parse_pins(args.pins)
    us = clamp_us(args.hold_position)

    if pigpio is None:
        print("pigpio not installed. Try: pip3 install pigpio", file=sys.stderr)
        return 1

    pi = pigpio.pi()
    if not pi.connected:
        print(
            "Cannot connect to pigpio daemon. Start it with:\n  sudo pigpiod",
            file=sys.stderr,
        )
        return 1

    try:
        set_all(pi, pins, us)
        print("Pins %s holding at %d µs — Ctrl+C to stop and turn servos off." % (pins, us))
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        off_all(pi, pins)
        pi.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
