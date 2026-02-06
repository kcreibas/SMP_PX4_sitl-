#!/usr/bin/env python3
import argparse
import struct
import time

import serial


SYNC = 0xA5
MAX_PAYLOAD = 128

MSG_NAMES = {
    0x0001: "HB",
    0x0002: "SYS",
    0x0003: "ATT",
    0x0004: "POS_L",
    0x0005: "POS_G",
    0x0006: "IMU",
    0x0007: "HLTH",
    0x0008: "GPS",
    0x0009: "VFR",
    0x000A: "ARM",
    0x0100: "CMD",
    0x0101: "ACK",
    0x0200: "ECHO",
}

MSG_CMD_ACK = 0x0101


def fmt_float_list(values):
    return ", ".join(f"{v:.3f}" for v in values)


def decode_payload(msgid: int, payload: bytes) -> str:
    if msgid == 0x0001 and len(payload) >= 7:
        uptime_ms, arming, nav, failsafe = struct.unpack_from("<IBBB", payload, 0)
        return f"uptime={uptime_ms}ms arm={arming} nav={nav} fs={failsafe}"
    if msgid == 0x0002 and len(payload) >= 13:
        batt_v, batt_a, batt_rem, errors = struct.unpack_from("<ffBI", payload, 0)
        return f"batt={batt_v:.2f}V {batt_a:.2f}A rem={batt_rem}% err={errors}"
    if msgid == 0x0003 and len(payload) >= 28:
        q = struct.unpack_from("<ffff", payload, 0)
        rates = struct.unpack_from("<fff", payload, 16)
        return f"q=({fmt_float_list(q)}) rates=({fmt_float_list(rates)})"
    if msgid == 0x0004 and len(payload) >= 24:
        x, y, z, vx, vy, vz = struct.unpack_from("<ffffff", payload, 0)
        return f"xyz=({x:.2f},{y:.2f},{z:.2f}) v=({vx:.2f},{vy:.2f},{vz:.2f})"
    if msgid == 0x0005 and len(payload) >= 12:
        lat, lon, alt = struct.unpack_from("<iif", payload, 0)
        return f"lat={lat} lon={lon} alt={alt:.2f}"
    if msgid == 0x0006 and len(payload) >= 36:
        ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack_from("<fffffffff", payload, 0)
        return (
            f"acc=({ax:.2f},{ay:.2f},{az:.2f}) "
            f"gyro=({gx:.2f},{gy:.2f},{gz:.2f}) "
            f"mag=({mx:.2f},{my:.2f},{mz:.2f})"
        )
    if msgid == 0x0009 and len(payload) >= 20:
        groundspeed, airspeed, alt, climb, heading_deg = struct.unpack_from("<fffff", payload, 0)
        return (
            f"gs={groundspeed:.2f} as={airspeed:.2f} alt={alt:.2f} "
            f"climb={climb:.2f} hdg={heading_deg:.1f}"
        )
    if msgid == MSG_CMD_ACK and len(payload) >= 4:
        cmd_id, result, progress = struct.unpack_from("<HBB", payload, 0)
        return f"cmd={cmd_id} res={result} prog={progress}"
    return f"len={len(payload)} data={payload.hex()}"


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


def parse_frames(buf: bytearray):
    frames = []
    i = 0
    while i + 9 <= len(buf):
        if buf[i] != SYNC:
            i += 1
            continue
        if i + 9 > len(buf):
            break
        sysid = buf[i + 1]
        compid = buf[i + 2]
        seq = buf[i + 3]
        msgid = buf[i + 4] | (buf[i + 5] << 8)
        payload_len = buf[i + 6] | (buf[i + 7] << 8)
        frame_len = 1 + 1 + 1 + 1 + 2 + 2 + payload_len + 2
        if payload_len > MAX_PAYLOAD or i + frame_len > len(buf):
            break
        payload = bytes(buf[i + 8 : i + 8 + payload_len])
        crc_rx = buf[i + 8 + payload_len] | (buf[i + 8 + payload_len + 1] << 8)
        crc_input = struct.pack("<BBBHH", sysid, compid, seq, msgid, payload_len) + payload
        crc_calc = crc16_ccitt(crc_input)
        if crc_calc == crc_rx:
            frames.append((sysid, compid, seq, msgid, payload))
        i += frame_len
    if i > 0:
        del buf[:i]
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="Simple SMP receiver")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /tmp/smp_mon)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    args = parser.parse_args()

    buf = bytearray()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"listening on {args.port} @ {args.baud}")
        while True:
            data = ser.read(256)
            if not data:
                continue
            buf.extend(data)
            for sysid, compid, seq, msgid, payload in parse_frames(buf):
                ts = time.time()
                name = MSG_NAMES.get(msgid, f"0x{msgid:04X}")
                decoded = decode_payload(msgid, payload)
                print(
                    f"{ts:.3f} | {name:<5} | sys={sysid:03d} comp={compid:03d} "
                    f"seq={seq:03d} | {decoded}"
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
