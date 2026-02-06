#!/usr/bin/env python3
import argparse
import collections
import curses
import struct
import time

import serial

SYNC = 0xA5
MAX_PAYLOAD = 128
MSG_CMD_ACK = 0x0101

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


def fmt_float_list(values):
    return ", ".join(f"{v:.2f}" for v in values)


def decode_payload(msgid: int, payload: bytes) -> str:
    if msgid == 0x0001 and len(payload) >= 7:
        uptime_ms, arming, nav, failsafe = struct.unpack_from("<IBBB", payload, 0)
        return f"uptime={uptime_ms}ms arm={arming} nav={nav} fs={failsafe}"
    if msgid == 0x0003 and len(payload) >= 28:
        q = struct.unpack_from("<ffff", payload, 0)
        rates = struct.unpack_from("<fff", payload, 16)
        return f"q=({fmt_float_list(q)}) rates=({fmt_float_list(rates)})"
    if msgid == 0x0004 and len(payload) >= 24:
        x, y, z, vx, vy, vz = struct.unpack_from("<ffffff", payload, 0)
        return f"xyz=({x:.2f},{y:.2f},{z:.2f}) v=({vx:.2f},{vy:.2f},{vz:.2f})"
    if msgid == 0x0006 and len(payload) >= 36:
        ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack_from("<fffffffff", payload, 0)
        return (
            f"acc=({ax:.2f},{ay:.2f},{az:.2f}) "
            f"gyro=({gx:.2f},{gy:.2f},{gz:.2f}) "
            f"mag=({mx:.2f},{mz:.2f})"
        )
    if msgid == 0x0100 and len(payload) >= 2:
        cmd_id = struct.unpack_from("<H", payload, 0)[0]
        p1 = struct.unpack_from("<f", payload, 2)[0] if len(payload) >= 6 else float("nan")
        return f"cmd={cmd_id} p1={p1:.2f} len={len(payload)}"
    if msgid == MSG_CMD_ACK and len(payload) >= 4:
        cmd_id, result, progress = struct.unpack_from("<HBB", payload, 0)
        return f"cmd={cmd_id} res={result} prog={progress}"
    return f"len={len(payload)}"


def _parse_filter(text: str):
    if not text:
        return None
    ids = set()
    for tok in text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        ids.add(int(tok, 0))
    return ids if ids else None


def _read_frames(ser, buf: bytearray, filter_ids):
    frames = []
    data = ser.read(256)
    if data:
        buf.extend(data)
        for sysid, compid, seq, msgid, payload in parse_frames(buf):
            if filter_ids is not None and msgid not in filter_ids:
                continue
            frames.append((sysid, compid, seq, msgid, payload))
    return frames


def ui_loop(stdscr, port: str, baud: int, filter_ids, port_fc: str | None, port_pc: str | None):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    top = collections.deque(maxlen=200)
    mid = collections.deque(maxlen=200)
    bottom = collections.deque(maxlen=200)

    buf = bytearray()
    ser = serial.Serial(port, baud, timeout=0.05)
    buf_fc = bytearray()
    buf_pc = bytearray()
    ser_fc = serial.Serial(port_fc, baud, timeout=0.05) if port_fc else None
    ser_pc = serial.Serial(port_pc, baud, timeout=0.05) if port_pc else None

    paused = False
    try:
        while True:
            try:
                key = stdscr.getkey()
                if key in ("q", "Q"):
                    break
                if key in ("p", "P", " "):
                    paused = not paused
                if key in ("c", "C"):
                    top.clear()
                    bottom.clear()
            except curses.error:
                pass

            if not paused:
                if ser_fc or ser_pc:
                    if ser_fc:
                        for sysid, compid, seq, msgid, payload in _read_frames(ser_fc, buf_fc, filter_ids):
                            ts = time.time()
                            name = MSG_NAMES.get(msgid, f"0x{msgid:04X}")
                            decoded = decode_payload(msgid, payload)
                            line = f"{ts:.3f} | {name:<5} | s={sysid:03d} c={compid:03d} q={seq:03d} | {decoded}"
                            top.append(line)
                    if ser_pc:
                        for sysid, compid, seq, msgid, payload in _read_frames(ser_pc, buf_pc, filter_ids):
                            ts = time.time()
                            name = MSG_NAMES.get(msgid, f"0x{msgid:04X}")
                            decoded = decode_payload(msgid, payload)
                            line = f"{ts:.3f} | {name:<5} | s={sysid:03d} c={compid:03d} q={seq:03d} | {decoded}"
                            bottom.append(line)
                else:
                    for sysid, compid, seq, msgid, payload in _read_frames(ser, buf, filter_ids):
                        ts = time.time()
                        name = MSG_NAMES.get(msgid, f"0x{msgid:04X}")
                        decoded = decode_payload(msgid, payload)
                        line = f"{ts:.3f} | {name:<5} | s={sysid:03d} c={compid:03d} q={seq:03d} | {decoded}"
                        if msgid == MSG_CMD_ACK:
                            top.append(line)
                        elif msgid == 0x0100:
                            mid.append(line)
                        else:
                            bottom.append(line)

            stdscr.erase()
            h, w = stdscr.getmaxyx()
            mid_line = h // 2
            status = "PAUSED" if paused else "LIVE"
            filt = ",".join(f"0x{x:04X}" for x in sorted(filter_ids)) if filter_ids else "ALL"
            stdscr.addstr(0, 0, f"SMP MSG ({status}) filter={filt}  [q quit | p pause | c clear]")
            if ser_fc or ser_pc:
                stdscr.addstr(1, 0, "FC -> PC")
                for i, line in enumerate(list(top)[- (mid_line - 3) :]):
                    stdscr.addstr(2 + i, 0, line[: w - 1])
                stdscr.hline(mid_line, 0, "-", w - 1)
                stdscr.addstr(mid_line + 1, 0, "PC -> FC")
                for i, line in enumerate(list(bottom)[- (h - mid_line - 3) :]):
                    stdscr.addstr(mid_line + 2 + i, 0, line[: w - 1])
            else:
                h1 = max(3, h // 4)
                h2 = max(3, h // 4)
                top_end = min(h1, h - 3)
                mid_start = top_end + 1
                mid_end = min(mid_start + h2, h - 2)
                bot_start = mid_end + 1

                stdscr.addstr(1, 0, "CMD_ACK / LOG")
                for i, line in enumerate(list(top)[- (top_end - 2) :]):
                    stdscr.addstr(2 + i, 0, line[: w - 1])
                stdscr.hline(top_end, 0, "-", w - 1)

                stdscr.addstr(mid_start, 0, "CMD (PC -> FC)")
                for i, line in enumerate(list(mid)[- (mid_end - mid_start - 1) :]):
                    stdscr.addstr(mid_start + 1 + i, 0, line[: w - 1])
                stdscr.hline(mid_end, 0, "-", w - 1)

                stdscr.addstr(bot_start, 0, "OTHER MSG")
                for i, line in enumerate(list(bottom)[- (h - bot_start - 1) :]):
                    stdscr.addstr(bot_start + 1 + i, 0, line[: w - 1])

            stdscr.refresh()
    finally:
        ser.close()
        if ser_fc:
            ser_fc.close()
        if ser_pc:
            ser_pc.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="SMP UI (split view)")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /tmp/smp_mon)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--port-fc", help="FC->PC monitor port (e.g. /tmp/smp_mon_px4)")
    parser.add_argument("--port-pc", help="PC->FC monitor port (e.g. /tmp/smp_mon_gcs)")
    parser.add_argument(
        "--filter",
        help="Comma-separated msgids to show (e.g. 0x0100,0x0101,0x0200)",
    )
    args = parser.parse_args()

    filter_ids = _parse_filter(args.filter)
    curses.wrapper(ui_loop, args.port, args.baud, filter_ids, args.port_fc, args.port_pc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
