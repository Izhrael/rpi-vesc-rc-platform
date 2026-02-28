#!/usr/bin/env python3
"""Minimal VESC USB protocol tool (telemetry first, safe control second).

This script intentionally keeps dependencies minimal: pyserial only.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_packet(payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too large for short frame")
    c = crc16(payload)
    return bytes([2, len(payload)]) + payload + struct.pack(">H", c) + bytes([3])


def decode_packet(frame: bytes) -> bytes:
    if len(frame) < 6:
        raise ValueError("frame too short")
    if frame[0] != 2 or frame[-1] != 3:
        raise ValueError("invalid frame markers")
    n = frame[1]
    expected = 1 + 1 + n + 2 + 1
    if len(frame) != expected:
        raise ValueError("length mismatch")
    payload = frame[2 : 2 + n]
    recv_crc = struct.unpack(">H", frame[2 + n : 2 + n + 2])[0]
    calc_crc = crc16(payload)
    if recv_crc != calc_crc:
        raise ValueError("crc mismatch")
    return payload


@dataclass
class ValuesSample:
    raw_payload_hex: str
    rpm: Optional[float] = None
    input_voltage: Optional[float] = None
    avg_motor_current: Optional[float] = None
    avg_input_current: Optional[float] = None


def parse_get_values(payload: bytes) -> ValuesSample:
    # Firmware layouts vary. Parse defensively using common field order.
    if not payload or payload[0] != COMM_GET_VALUES:
        raise ValueError("not a COMM_GET_VALUES payload")

    sample = ValuesSample(raw_payload_hex=payload.hex())

    i = 1

    def need(n: int) -> bool:
        return i + n <= len(payload)

    def i16(scale: float) -> Optional[float]:
        nonlocal i
        if not need(2):
            return None
        v = struct.unpack_from(">h", payload, i)[0]
        i += 2
        return v / scale

    def i32(scale: float) -> Optional[float]:
        nonlocal i
        if not need(4):
            return None
        v = struct.unpack_from(">i", payload, i)[0]
        i += 4
        return v / scale

    # Skip some early temperature / current fields to reach key telemetry.
    _ = i16(10.0)  # temp_mos1
    _ = i16(10.0)  # temp_mos2
    sample.avg_motor_current = i32(100.0)
    sample.avg_input_current = i32(100.0)

    # Skip id / iq / duty / erpm derivative fields if present.
    _ = i32(100.0)
    _ = i32(100.0)
    _ = i16(1000.0)

    sample.rpm = i32(1.0)
    sample.input_voltage = i16(10.0)

    return sample


class VescClient:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def close(self) -> None:
        self.ser.close()

    def send_payload(self, payload: bytes) -> None:
        self.ser.write(encode_packet(payload))
        self.ser.flush()

    def read_packet(self, timeout: float = 0.5) -> bytes:
        deadline = time.time() + timeout
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != 2:
                continue
            length_b = self.ser.read(1)
            if len(length_b) != 1:
                continue
            n = length_b[0]
            rest = self.ser.read(n + 2 + 1)
            if len(rest) != n + 3:
                continue
            frame = b + length_b + rest
            return decode_packet(frame)
        raise TimeoutError("no valid packet received before timeout")

    def get_values(self) -> ValuesSample:
        self.send_payload(bytes([COMM_GET_VALUES]))
        payload = self.read_packet(timeout=0.5)
        return parse_get_values(payload)

    def set_neutral(self) -> None:
        self.send_payload(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", 0))
        self.send_payload(bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0))

    def set_rpm(self, rpm: int) -> None:
        self.send_payload(bytes([COMM_SET_RPM]) + struct.pack(">i", rpm))


def cmd_telemetry(client: VescClient, samples: int, interval: float) -> int:
    for idx in range(samples):
        s = client.get_values()
        print(
            f"sample={idx:03d} rpm={s.rpm} vin={s.input_voltage} "
            f"i_motor={s.avg_motor_current} i_in={s.avg_input_current}"
        )
        time.sleep(interval)
    return 0


def cmd_neutral(client: VescClient) -> int:
    client.set_neutral()
    print("Neutral command sent (current=0, brake_current=0)")
    return 0


def cmd_rpm(client: VescClient, value: int, confirm: bool) -> int:
    if not confirm:
        raise SystemExit("Refusing to set RPM without --confirm")
    client.set_rpm(value)
    print(f"RPM command sent: {value}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Minimal VESC USB protocol tool")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    t = sub.add_parser("telemetry", help="Read-only telemetry")
    t.add_argument("--samples", type=int, default=10)
    t.add_argument("--interval", type=float, default=0.2)

    sub.add_parser("neutral", help="Send safe neutral command")

    r = sub.add_parser("rpm", help="Send controlled RPM command")
    r.add_argument("--value", type=int, required=True)
    r.add_argument("--confirm", action="store_true")

    return p


def main() -> int:
    args = build_parser().parse_args()
    client = VescClient(port=args.port, baudrate=args.baud)
    try:
        if args.cmd == "telemetry":
            return cmd_telemetry(client, samples=args.samples, interval=args.interval)
        if args.cmd == "neutral":
            return cmd_neutral(client)
        if args.cmd == "rpm":
            return cmd_rpm(client, value=args.value, confirm=args.confirm)
        raise SystemExit(f"Unknown command: {args.cmd}")
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
