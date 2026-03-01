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
    p.add_argument("--axis", default="ABS_Y", help="Preferred axis name/code (default: ABS_Y)")
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


def list_abs_axes(gamepad, ecodes_module):
    """Return available ABS axes as list[(code, absinfo)] from capabilities."""
    caps = gamepad.capabilities(absinfo=True)
    axes = caps.get(ecodes_module.EV_ABS, [])
    axes = sorted(axes, key=lambda item: item[0])
    return axes


def try_absinfo(gamepad, axis_code: int):
    try:
        return gamepad.absinfo(axis_code)
    except OSError:
        return None


def choose_axis(gamepad, ecodes_module, requested_axis: str):
    axes = list_abs_axes(gamepad, ecodes_module)
    if not axes:
        raise SystemExit(
            f"No EV_ABS axes found on {gamepad.path} ({gamepad.name}). "
            "Use the correct /dev/input/eventX device for your controller."
        )

    print("Detected ABS axes:")
    for code, info in axes:
        name = ecodes_module.bytype[ecodes_module.EV_ABS].get(code, str(code))
        print(f"  - {name} ({code}): min={info.min} max={info.max} value={info.value}")

    requested_code = None
    try:
        requested_code = parse_axis_code(requested_axis, ecodes_module)
    except AttributeError:
        requested_code = None

    available_codes = {code for code, _ in axes}

    preferred = []
    if requested_code is not None:
        preferred.append(requested_code)
    preferred.extend(
        [
            ecodes_module.ABS_Y,
            ecodes_module.ABS_RY,
            ecodes_module.ABS_Z,
            ecodes_module.ABS_RZ,
            ecodes_module.ABS_X,
            ecodes_module.ABS_RX,
        ]
    )

    seen = set()
    for code in preferred:
        if code in seen:
            continue
        seen.add(code)
        if code in available_codes:
            info = try_absinfo(gamepad, code)
            if info is not None:
                return code, info

    # Final fallback: first available axis with valid absinfo
    for code, _ in axes:
        info = try_absinfo(gamepad, code)
        if info is not None:
            return code, info

    raise SystemExit("Unable to read absinfo from available axes on this device")


def main() -> int:
    args = build_parser().parse_args()

    from evdev import InputDevice, ecodes

    if not (0.0 <= args.deadzone < 1.0):
        raise SystemExit("--deadzone must be in [0, 1)")
    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")

    period = 1.0 / args.hz
    gamepad = InputDevice(args.controller)

    axis_code, absinfo = choose_axis(gamepad, ecodes, args.axis)
    axis_name = ecodes.bytype[ecodes.EV_ABS].get(axis_code, str(axis_code))
    axis_value = absinfo.value

    vesc = VescClient(port=args.port, baudrate=args.baud)

    print(
        f"Using controller {gamepad.path} ({gamepad.name}) | axis={axis_name} ({axis_code}) "
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
                for event in gamepad.read():
                    if event.type == ecodes.EV_ABS and event.code == axis_code:
                        axis_value = event.value

            now = time.monotonic()
            if now >= next_send:
                normalized = normalize_axis(axis_value, absinfo.min, absinfo.max, args.invert)
                shaped = apply_deadzone(normalized, args.deadzone)
                target_rpm = int(round(shaped * args.max_erpm))

                vesc.set_rpm(target_rpm)
                print(
                    f"axis_raw={axis_value:>6} norm={normalized:+.3f} "
                    f"dz={shaped:+.3f} rpm={target_rpm:+6d}",
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
