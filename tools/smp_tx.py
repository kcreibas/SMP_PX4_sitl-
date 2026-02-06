#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time
from typing import Iterable


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


def _hexdump(data: bytes, width: int = 16) -> str:
    chunks: Iterable[bytes] = (data[i : i + width] for i in range(0, len(data), width))
    return " ".join(chunk.hex() for chunk in chunks)


def cmd_payload_f64(cmd: int, p1: float, p2: float, p3: float, p4: float, p5: float, p6: float, p7: float) -> bytes:
    # u16 + 4 * f32 + 2 * f64 + 1 * f32  => 38 bytes
    return struct.pack("<Hffffddf", cmd, p1, p2, p3, p4, p5, p6, p7)


def main() -> int:
    parser = argparse.ArgumentParser(description="Simple SMP sender")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /tmp/smp_gcs)")
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
    parser.add_argument("--count", type=int, default=1, help="Number of frames to send")
    parser.add_argument("--interval-ms", type=int, default=200, help="Interval between frames (ms)")
    parser.add_argument("--repl", action="store_true", help="Interactive mode: type commands line by line")
    parser.add_argument("--menu", action="store_true", help="Interactive menu: press 1/2/3...")
    parser.add_argument("--verbose", action="store_true", help="Print sent frames")
    args = parser.parse_args()

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
        if args.menu:
            print("SMP TX menu:")
            print("  1) ARM (cmd 400, p1=1)")
            print("  2) DISARM (cmd 400, p1=0)")
            print("  3) TAKEOFF (cmd 22)")
            print("  4) LAND (cmd 21)")
            print("  q) quit")
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                if line in ("q", "quit", "exit"):
                    break
                if line == "1":
                    payload = cmd_payload_f64(400, 1.0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
                    msgid = 0x0100
                elif line == "2":
                    payload = cmd_payload_f64(400, 0.0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
                    msgid = 0x0100
                elif line == "3":
                    payload = cmd_payload_f64(22, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
                    msgid = 0x0100
                elif line == "4":
                    payload = cmd_payload_f64(21, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
                    msgid = 0x0100
                else:
                    print("unknown selection")
                    continue
                frame = build_frame(args.sysid & 0xFF, args.compid & 0xFF, seq, msgid & 0xFFFF, payload)
                written = ser.write(frame)
                if args.verbose:
                    print(f"sent {written} bytes: {_hexdump(frame)}")
                seq = (seq + 1) & 0xFF
        elif args.repl:
            print("SMP TX repl: type 'cmd <id> [p1..p7]' or 'hex <msgid> <payloadhex>' or 'quit'")
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                if line in ("q", "quit", "exit"):
                    break
                parts = line.split()
                if parts[0] == "cmd":
                    if len(parts) < 2:
                        print("usage: cmd <id> [p1..p7]")
                        continue
                    cmd_id = int(parts[1], 0)
                    vals = [math.nan] * 7
                    for i, tok in enumerate(parts[2:9]):
                        vals[i] = float(tok)
                    payload = cmd_payload_f64(cmd_id, *vals)
                    msgid = 0x0100
                elif parts[0] == "hex":
                    if len(parts) < 3:
                        print("usage: hex <msgid> <payloadhex>")
                        continue
                    msgid = int(parts[1], 0)
                    payload = bytes.fromhex(parts[2])
                else:
                    print("unknown command")
                    continue
                frame = build_frame(args.sysid & 0xFF, args.compid & 0xFF, seq, msgid & 0xFFFF, payload)
                written = ser.write(frame)
                if args.verbose:
                    print(f"sent {written} bytes: {_hexdump(frame)}")
                seq = (seq + 1) & 0xFF
        else:
            for i in range(args.count):
                frame = build_frame(args.sysid & 0xFF, args.compid & 0xFF, seq, args.msgid & 0xFFFF, payload)
                written = ser.write(frame)
                if args.verbose:
                    print(f"sent {written} bytes: {_hexdump(frame)}")
                seq = (seq + 1) & 0xFF
                if i + 1 < args.count:
                    time.sleep(max(0, args.interval_ms) / 1000.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
