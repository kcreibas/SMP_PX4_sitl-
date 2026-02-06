#!/usr/bin/env python3
"""
Simple SMP v1 sender for testing CMD frames over UART.

Requires: pyserial (pip install pyserial)

Examples:
  # Fixed layout (u16 + 4*f32 + 2*f64 + 1*f32), len=38
  python3 tools/smp_send.py --port /dev/ttyUSB0 --cmd 400 \
      --p1 1.1 --p2 2.2 --p3 3.3 --p4 4.4 --p5 55.123456789 --p6 -66.987654321 --p7 7.7

  # Send raw payload bytes (hex)
  python3 tools/smp_send.py --port /dev/ttyUSB0 --msgid 0x0001 --payload-hex 4842
"""

import argparse
import math
import struct
import sys
import time


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(sysid: int, compid: int, seq: int, msg_id: int, payload: bytes) -> bytes:
    header = struct.pack("<BBB", sysid, compid, seq) + struct.pack("<H", msg_id) + struct.pack("<H", len(payload))
    crc = crc16_ccitt(header + payload)
    return b"\xA5" + header + payload + struct.pack("<H", crc)


def cmd_payload_f64(cmd: int, p1: float, p2: float, p3: float, p4: float, p5: float, p6: float, p7: float) -> bytes:
    # u16 + 4 * f32 + 2 * f64 + 1 * f32  => 38 bytes
    return struct.pack("<Hffffddf", cmd, p1, p2, p3, p4, p5, p6, p7)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SMP v1 sender for CMD testing")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--sysid", type=int, default=1, help="Sysid (default: 1)")
    parser.add_argument("--compid", type=int, default=1, help="Compid (default: 1)")
    parser.add_argument("--seq", type=int, default=0, help="Start sequence number (default: 0)")
    parser.add_argument("--msgid", type=lambda s: int(s, 0), default=0x0100, help="Message ID (default: 0x0100)")
    parser.add_argument("--cmd", type=int, default=400, help="CMD id (default: 400)")
    parser.add_argument("--p1", type=float, default=math.nan)
    parser.add_argument("--p2", type=float, default=math.nan)
    parser.add_argument("--p3", type=float, default=math.nan)
    parser.add_argument("--p4", type=float, default=math.nan)
    parser.add_argument("--p5", type=float, default=math.nan)
    parser.add_argument("--p6", type=float, default=math.nan)
    parser.add_argument("--p7", type=float, default=math.nan)
    parser.add_argument("--payload-hex", help="Raw payload hex (overrides CMD payload)")
    parser.add_argument("--count", type=int, default=1, help="Number of frames to send (default: 1)")
    parser.add_argument("--interval-ms", type=int, default=200, help="Interval between frames (default: 200ms)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        import serial  # type: ignore
    except Exception:
        print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
        return 1

    if args.payload_hex:
        payload = bytes.fromhex(args.payload_hex)
    else:
        if args.msgid != 0x0100:
            print("CMD payload builder is only for msgid 0x0100. Use --payload-hex for other IDs.", file=sys.stderr)
            return 1
        payload = cmd_payload_f64(args.cmd, args.p1, args.p2, args.p3, args.p4, args.p5, args.p6, args.p7)

    if len(payload) > 128:
        print(f"Payload too large: {len(payload)} bytes (max 128)", file=sys.stderr)
        return 1

    seq = args.seq & 0xFF

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        for i in range(args.count):
            frame = build_frame(args.sysid & 0xFF, args.compid & 0xFF, seq, args.msgid & 0xFFFF, payload)
            ser.write(frame)
            seq = (seq + 1) & 0xFF
            if i + 1 < args.count:
                time.sleep(max(0, args.interval_ms) / 1000.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
