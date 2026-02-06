#!/usr/bin/env python3
import argparse
import struct
import time

import serial


SYNC = 0xA5
MAX_PAYLOAD = 31
MSG_COMMAND_ACK = 0x90
MSG_COMMAND_DO = 0x91
MSG_COMMAND_SHORT = 0x92
MSG_COMMAND_LONG = 0x93


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
        sysid = buf[i + 1]
        compid = buf[i + 2]
        seq = buf[i + 3]
        msgid = buf[i + 4]
        payload_len = buf[i + 5]
        frame_len = 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2
        if payload_len > MAX_PAYLOAD or i + frame_len > len(buf):
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
    parser = argparse.ArgumentParser(description="Send SMP COMMAND_* and wait for COMMAND_ACK.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM4)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--sysid", type=int, default=1, help="Sysid (0-255)")
    parser.add_argument("--compid", type=int, default=1, help="Compid (0-255)")
    parser.add_argument("--type", choices=["do", "short", "long"], default="short", help="Command type")
    parser.add_argument("--cmd", type=int, required=True, help="Command id (uint8)")
    parser.add_argument("--p1", type=float, default=0.0, help="Param1")
    parser.add_argument("--p2", type=float, default=0.0, help="Param2")
    parser.add_argument("--p3", type=float, default=0.0, help="Param3")
    parser.add_argument("--p4", type=float, default=0.0, help="Param4")
    parser.add_argument("--p5", type=float, default=0.0, help="Param5")
    parser.add_argument("--p6", type=float, default=0.0, help="Param6")
    parser.add_argument("--p7", type=float, default=0.0, help="Param7")
    parser.add_argument("--target-sys", type=int, default=0, help="Target system (0-255)")
    parser.add_argument("--target-comp", type=int, default=0, help="Target component (0-255)")
    parser.add_argument("--timeout", type=float, default=2.0, help="ACK wait seconds")
    args = parser.parse_args()

    seq = 0
    if args.type == "do":
        payload = struct.pack("<BfBB", args.cmd & 0xFF, args.p1, args.target_sys & 0xFF, args.target_comp & 0xFF)
        msgid = MSG_COMMAND_DO
    elif args.type == "long":
        payload = struct.pack(
            "<BfffffffBB",
            args.cmd & 0xFF,
            args.p1,
            args.p2,
            args.p3,
            args.p4,
            args.p5,
            args.p6,
            args.p7,
            args.target_sys & 0xFF,
            args.target_comp & 0xFF,
        )
        msgid = MSG_COMMAND_LONG
    else:
        payload = struct.pack(
            "<BffffBB", args.cmd & 0xFF, args.p1, args.p2, args.p3, args.p4, args.target_sys & 0xFF, args.target_comp & 0xFF
        )
        msgid = MSG_COMMAND_SHORT

    frame = build_frame(args.sysid, args.compid, seq, msgid, payload)

    buf = bytearray()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(
            "TX {} cmd={} p1={} p2={} p3={} p4={} p5={} p6={} p7={} tgt={}.{} sysid={} compid={}".format(
                args.type.upper(),
                args.cmd,
                args.p1,
                args.p2,
                args.p3,
                args.p4,
                args.p5,
                args.p6,
                args.p7,
                args.target_sys,
                args.target_comp,
                args.sysid,
                args.compid,
            )
        )
        ser.write(frame)
        end_time = time.time() + args.timeout
        while time.time() < end_time:
            data = ser.read(256)
            if data:
                buf.extend(data)
                for _, _, _, msgid, payload in parse_frames(buf):
                    if msgid == MSG_COMMAND_ACK and len(payload) >= 3:
                        cmd_id, result, seq = struct.unpack_from("<BBB", payload, 0)
                        print(f"CMD_ACK cmd={cmd_id} result={result} seq={seq}")
                        return
        print("No CMD_ACK received")


if __name__ == "__main__":
    main()

