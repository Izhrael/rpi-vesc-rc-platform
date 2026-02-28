#!/usr/bin/env python3
"""Drive VESC RPM from a Linux gamepad axis via evdev.

Single-loop, watchdog-safe refresh:
- Reads axis events from /dev/input/eventX
- Converts selected axis to target ERPM with deadzone
- Sends COMM_SET_RPM continuously at a fixed rate
- Sends neutral on Ctrl+C
"""

from __future__ import annotations

import argparse
import select
import time

from vesc_protocol import VescClient


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply symmetric deadzone and rescale to keep full-range output."""
    if abs(value) <= deadzone:
        return 0.0
    if value > 0:
        return (value - deadzone) / (1.0 - deadzone)
    return (value + deadzone) / (1.0 - deadzone)


def normalize_axis(raw: int, minimum: int, maximum: int, invert: bool) -> float:
    """Normalize raw axis value to [-1.0, 1.0]."""
    center = (minimum + maximum) / 2.0
    half = (maximum - minimum) / 2.0
    if half <= 0:
        return 0.0

    normalized = (raw - center) / half
    normalized = max(-1.0, min(1.0, normalized))
    if invert:
        normalized = -normalized
    return normalized


def parse_axis_code(axis: str, ecodes_module) -> int:
    """Accept ABS_* names or integer codes."""
    if axis.isdigit():
        return int(axis)
    return getattr(ecodes_module, axis)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Map controller axis to VESC RPM")
    p.add_argument("--controller", default="/dev/input/event0", help="evdev device path")
    p.add_argument("--port", default="/dev/ttyACM0", help="VESC serial port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--axis", default="ABS_Y", help="Axis name/code (default: ABS_Y)")
    p.add_argument("--deadzone", type=float, default=0.05, help="Deadzone in [0,1]")
    p.add_argument("--max-erpm", type=int, default=3000)
    p.add_argument("--hz", type=float, default=30.0, help="Refresh rate (20-50 typical)")
    p.add_argument(
        "--invert",
        action="store_true",
        default=True,
        help="Invert axis so stick-up is positive RPM (default: on)",
    )
    p.add_argument(
        "--no-invert",
        action="store_false",
        dest="invert",
        help="Disable axis inversion",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()

    from evdev import InputDevice, ecodes

    if not (0.0 <= args.deadzone < 1.0):
        raise SystemExit("--deadzone must be in [0, 1)")
    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")

    axis_code = parse_axis_code(args.axis, ecodes)
    period = 1.0 / args.hz

    gamepad = InputDevice(args.controller)
    gamepad.set_blocking(False)

    absinfo = gamepad.absinfo(axis_code)
    if absinfo is None:
        raise SystemExit(f"Axis {args.axis} not found on {args.controller}")

    axis_value = absinfo.value

    vesc = VescClient(port=args.port, baudrate=args.baud)

    print(
        f"Controller: {gamepad.path} ({gamepad.name}) | Axis: {args.axis} "
        f"[{absinfo.min}, {absinfo.max}] | max_erpm={args.max_erpm} hz={args.hz}"
    )

    try:
        vesc.set_neutral()
        next_send = time.monotonic()

        while True:
            now = time.monotonic()
            timeout = max(0.0, next_send - now)
            readable, _, _ = select.select([gamepad.fd], [], [], timeout)

            if readable:
                try:
                    for event in gamepad.read():
                        if event.type == ecodes.EV_ABS and event.code == axis_code:
                            axis_value = event.value
                except BlockingIOError:
                    pass

            now = time.monotonic()
            if now >= next_send:
                normalized = normalize_axis(axis_value, absinfo.min, absinfo.max, args.invert)
                shaped = apply_deadzone(normalized, args.deadzone)
                target_rpm = int(round(shaped * args.max_erpm))

                vesc.set_rpm(target_rpm)
                print(
                    f"axis_raw={axis_value:>5} norm={normalized:+.3f} "
                    f"dz={shaped:+.3f} rpm={target_rpm:+5d}",
                    end="\r",
                    flush=True,
                )

                while next_send <= now:
                    next_send += period

    except KeyboardInterrupt:
        print("\nCtrl+C: sending neutral...")
        vesc.set_neutral()
        return 0
    finally:
        vesc.close()


if __name__ == "__main__":
    raise SystemExit(main())
