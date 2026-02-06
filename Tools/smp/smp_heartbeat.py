#!/usr/bin/env python3
import argparse
import struct
import time

import serial


SYNC = 0xA5
MAX_PAYLOAD = 31


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


def build_frame(sysid: int, compid: int, seq: int, msgid: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")

    header = bytes([SYNC, sysid, compid, seq, msgid, len(payload)])
    crc_input = bytes([sysid, compid, seq, msgid, len(payload)]) + payload
    crc = crc16_ccitt(crc_input)
    return header + payload + struct.pack("<H", crc)


def parse_frames(buf: bytearray):
    frames = []
    i = 0
    while i + 8 <= len(buf):
        if buf[i] != SYNC:
            i += 1
            continue
        if i + 8 > len(buf):
            break
        sysid = buf[i + 1]
        compid = buf[i + 2]
        seq = buf[i + 3]
        msgid = buf[i + 4]
        payload_len = buf[i + 5]
        frame_len = 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2
        if i + frame_len > len(buf):
            break
        payload = bytes(buf[i + 6 : i + 6 + payload_len])
        crc_rx = buf[i + 6 + payload_len] | (buf[i + 6 + payload_len + 1] << 8)
        crc_input = bytes([sysid, compid, seq, msgid, payload_len]) + payload
        crc_calc = crc16_ccitt(crc_input)
        if crc_calc == crc_rx:
            frames.append((sysid, compid, seq, msgid, payload))
        i += frame_len
    if i > 0:
        del buf[:i]
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description="Send SMP v1 heartbeat frames.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0 or COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--sysid", type=int, default=1, help="Sysid (0-255)")
    parser.add_argument("--compid", type=int, default=1, help="Compid (0-255)")
    parser.add_argument("--msgid", type=int, default=1, help="Msgid (0-255)")
    parser.add_argument("--payload", default="HB", help="Payload string (ASCII)")
    parser.add_argument("--ack-msgid", type=int, default=0xFF, help="ACK msgid to expect (0-255)")
    parser.add_argument("--ack-payload", default="ACK", help="ACK payload string (ASCII)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between sends")
    parser.add_argument("--once", action="store_true", help="Send only one frame")
    args = parser.parse_args()

    payload = args.payload.encode("ascii", errors="strict")
    ack_payload = args.ack_payload.encode("ascii", errors="strict")

    seq = 0
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        frame = build_frame(args.sysid, args.compid, seq & 0xFF, args.msgid, payload)
        ser.write(frame)
        if args.once:
            end_time = time.time() + 2.0
            buf = bytearray()
            while time.time() < end_time:
                data = ser.read(256)
                if data:
                    buf.extend(data)
                    for f in parse_frames(buf):
                        if f[3] == args.ack_msgid and f[4] == ack_payload:
                            print("ACK received")
                            return
            print("No ACK received")
            return

        while True:
            time.sleep(args.interval)
            seq = (seq + 1) & 0xFF
            frame = build_frame(args.sysid, args.compid, seq, args.msgid, payload)
            ser.write(frame)


if __name__ == "__main__":
    main()

