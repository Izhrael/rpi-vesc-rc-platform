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



COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_SERVO_POS = 12

MC_FAULT_CODE_NAMES = {
    0: "FAULT_CODE_NONE",
    1: "FAULT_CODE_OVER_VOLTAGE",
    2: "FAULT_CODE_UNDER_VOLTAGE",
    3: "FAULT_CODE_DRV",
    4: "FAULT_CODE_ABS_OVER_CURRENT",
    5: "FAULT_CODE_OVER_TEMP_FET",
    6: "FAULT_CODE_OVER_TEMP_MOTOR",
    7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
    10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    11: "FAULT_CODE_ENCODER_SPI",
    12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FAULT_CODE_FLASH_CORRUPTION",
    15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "FAULT_CODE_UNBALANCED_CURRENTS",
    19: "FAULT_CODE_BRK",
    20: "FAULT_CODE_RESOLVER_LOT",
    21: "FAULT_CODE_RESOLVER_DOS",
    22: "FAULT_CODE_RESOLVER_LOS",
}


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
    duty_cycle: Optional[float] = None
    fault_code: Optional[int] = None
    temp_mos: tuple[float, ...] = ()
    temp_motor: Optional[float] = None


def parse_get_values(payload: bytes) -> ValuesSample:
    # Firmware layouts vary. Support two common layouts:
    # A) includes id/iq currents, B) omits id/iq.
    if not payload or payload[0] != COMM_GET_VALUES:
        raise ValueError("not a COMM_GET_VALUES payload")

    sample = ValuesSample(raw_payload_hex=payload.hex())

    def read_i16_at(offset: int, scale: float) -> float:
        return struct.unpack_from(">h", payload, offset)[0] / scale

    def read_i32_at(offset: int, scale: float) -> float:
        return struct.unpack_from(">i", payload, offset)[0] / scale

    body_len = len(payload) - 1
    if body_len <= 0:
        raise ValueError("COMM_GET_VALUES payload too short")

    # fixed tails excluding variable temperature section.
    tails = [
        ("with_id_iq", 45),
        ("without_id_iq", 41),
    ]

    best = None
    for layout, tail_len in tails:
        if body_len < tail_len:
            continue

        base = 1 + (body_len - tail_len)  # first byte after temp block
        end = len(payload)

        if layout == "with_id_iq":
            avg_motor_off = base
            avg_input_off = avg_motor_off + 4
            _id_off = avg_input_off + 4
            _iq_off = _id_off + 4
            duty_off = _iq_off + 4
            rpm_off = duty_off + 2
        else:
            avg_motor_off = base
            avg_input_off = avg_motor_off + 4
            duty_off = avg_input_off + 4
            rpm_off = duty_off + 2

        vin_off = rpm_off + 4
        ah_off = vin_off + 2
        ah_ch_off = ah_off + 4
        wh_off = ah_ch_off + 4
        wh_ch_off = wh_off + 4
        tach_off = wh_ch_off + 4
        tach_abs_off = tach_off + 4
        fault_off = tach_abs_off + 4

        if fault_off != end - 1:
            continue

        vin = read_i16_at(vin_off, 10.0)
        duty = read_i16_at(duty_off, 1000.0)
        fault = payload[fault_off]

        plausible = (0.0 <= vin <= 100.0) and (-1.2 <= duty <= 1.2) and (fault <= 50)
        score = int(plausible)

        candidate = {
            "score": score,
            "avg_motor": read_i32_at(avg_motor_off, 100.0),
            "avg_input": read_i32_at(avg_input_off, 100.0),
            "duty": duty,
            "rpm": read_i32_at(rpm_off, 1.0),
            "vin": vin,
            "fault": fault,
            "temp_region_end": avg_motor_off,
        }

        if best is None or candidate["score"] > best["score"]:
            best = candidate

    if best is None:
        raise ValueError("Unsupported COMM_GET_VALUES layout")

    sample.avg_motor_current = best["avg_motor"]
    sample.avg_input_current = best["avg_input"]
    sample.duty_cycle = best["duty"]
    sample.rpm = best["rpm"]
    sample.input_voltage = best["vin"]
    sample.fault_code = best["fault"]

    temp_region = payload[1 : best["temp_region_end"]]
    if len(temp_region) >= 2:
        temps = [
            struct.unpack_from(">h", temp_region, o)[0] / 10.0
            for o in range(0, len(temp_region) - (len(temp_region) % 2), 2)
        ]
        if temps:
            if len(temps) >= 2:
                sample.temp_mos = tuple(temps[:-1])
                sample.temp_motor = temps[-1]
            else:
                sample.temp_motor = temps[0]

    return sample


class VescClient:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        import serial

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

    def set_servo(self, pos: float) -> None:
        pos = max(0.0, min(1.0, pos))
        value = int(round(pos * 1000.0))
        self.send_payload(bytes([COMM_SET_SERVO_POS]) + struct.pack(">i", value))


def cmd_telemetry(client: VescClient, samples: int, interval: float) -> int:
    for idx in range(samples):
        s = client.get_values()
        fault_name = MC_FAULT_CODE_NAMES.get(s.fault_code, "FAULT_CODE_UNKNOWN")
        print(
            f"sample={idx:03d} rpm={s.rpm} vin={s.input_voltage} "
            f"i_motor={s.avg_motor_current} i_in={s.avg_input_current} "
            f"duty={s.duty_cycle} fault={s.fault_code}({fault_name})"
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


def cmd_servo(client: VescClient, value: float, confirm: bool) -> int:
    if not confirm:
        raise SystemExit("Refusing to set servo without --confirm")
    clamped = max(0.0, min(1.0, value))
    client.set_servo(clamped)
    print(f"Servo command sent: {clamped:.3f}")
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

    s = sub.add_parser("servo", help="Send controlled servo position command")
    s.add_argument("--value", type=float, required=True)
    s.add_argument("--confirm", action="store_true")

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
        if args.cmd == "servo":
            return cmd_servo(client, value=args.value, confirm=args.confirm)
        raise SystemExit(f"Unknown command: {args.cmd}")
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
