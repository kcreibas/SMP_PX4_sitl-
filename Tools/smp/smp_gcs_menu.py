#!/usr/bin/env python3
"""
Simple SMP GCS (menu-driven) for testing RX telemetry and sending CMD frames.
Requires: pyserial (pip install pyserial)
"""

import argparse
import math
import socket
import struct
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None


SYNC = 0xA5
MAX_PAYLOAD = 31

MSG_HEARTBEAT = 0x01
MSG_VEH_DATA = 0x02
MSG_SYS_STATUS = 0x10
MSG_BATTERY_STATUS = 0x11
MSG_GPS_FIX = 0x20
MSG_GLOBAL_POSITION = 0x21
MSG_ATTITUDE = 0x30
MSG_HEADING = 0x31
MSG_AIRDATA = 0x40
MSG_COMMAND_ACK = 0x90
MSG_COMMAND_DO = 0x91
MSG_COMMAND_SHORT = 0x92
MSG_COMMAND_LONG = 0x93
MSG_PARAM_SET = 0xB0
MSG_PARAM_GET = 0xB1

MSG_NAMES = {
    MSG_HEARTBEAT: "HB",
    MSG_VEH_DATA: "VEH",
    MSG_SYS_STATUS: "SYS",
    MSG_BATTERY_STATUS: "BATT",
    MSG_GPS_FIX: "GPS",
    MSG_GLOBAL_POSITION: "GLOB",
    MSG_ATTITUDE: "ATT",
    MSG_HEADING: "HDG",
    MSG_AIRDATA: "AIR",
    MSG_COMMAND_ACK: "ACK",
    MSG_COMMAND_DO: "CMD_DO",
    MSG_COMMAND_SHORT: "CMD_S",
    MSG_COMMAND_LONG: "CMD_L",
    MSG_PARAM_SET: "PSET",
    MSG_PARAM_GET: "PGET",
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


def build_frame(sysid: int, compid: int, seq: int, msg_id: int, payload: bytes) -> bytes:
    header = bytes([sysid, compid, seq, msg_id, len(payload)])
    crc = crc16_ccitt(header + payload)
    return bytes([SYNC]) + header + payload + struct.pack("<H", crc)


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


def fmt_float_list(values):
    return ", ".join(f"{v:.3f}" for v in values)


def decode_payload(msgid: int, payload: bytes) -> str:
    if msgid == MSG_HEARTBEAT and len(payload) >= 5:
        mtype, autopilot, base_mode, custom_mode, system_status = struct.unpack_from("<BBBBB", payload, 0)
        return (
            f"type={mtype} autopilot={autopilot} base={base_mode} "
            f"custom={custom_mode} status={system_status}"
        )
    if msgid == MSG_VEH_DATA and len(payload) >= 11:
        vehicle_type, max_speed, max_alt, endurance, payload_cap, fw_ver = struct.unpack_from("<BHHHHH", payload, 0)
        return (
            f"type={vehicle_type} max_spd={max_speed} max_alt={max_alt} "
            f"end={endurance} payload={payload_cap} fw={fw_ver}"
        )
    if msgid == MSG_SYS_STATUS and len(payload) >= 9:
        sens, cpu, vbatt, ibatt, batt_rem, err_comm = struct.unpack_from("<BBHhBH", payload, 0)
        return f"sens={sens} cpu={cpu} vbatt={vbatt} ibatt={ibatt} rem={batt_rem} err={err_comm}"
    if msgid == MSG_BATTERY_STATUS and len(payload) >= 9:
        volt, curr, rem, time_rem, cell_cnt, temp = struct.unpack_from("<HhBHBb", payload, 0)
        return f"v={volt} i={curr} rem={rem} time={time_rem} cell={cell_cnt} temp={temp}"
    if msgid == MSG_GPS_FIX and len(payload) >= 10:
        fix, sats, hdop, vdop, eph, epv = struct.unpack_from("<BBHHHH", payload, 0)
        return f"fix={fix} sats={sats} hdop={hdop} vdop={vdop} eph={eph} epv={epv}"
    if msgid == MSG_GLOBAL_POSITION and len(payload) >= 22:
        lat, lon, alt, rel_alt, vx, vy, vz = struct.unpack_from("<iiiihhh", payload, 0)
        return f"lat={lat} lon={lon} alt={alt} rel={rel_alt} v=({vx},{vy},{vz})"
    if msgid == MSG_ATTITUDE and len(payload) >= 12:
        roll, pitch, yaw, rs, ps, ys = struct.unpack_from("<hhHhhh", payload, 0)
        return f"r={roll} p={pitch} y={yaw} rs={rs} ps={ps} ys={ys}"
    if msgid == MSG_HEADING and len(payload) >= 4:
        heading, heading_target = struct.unpack_from("<HH", payload, 0)
        return f"hdg={heading} tgt={heading_target}"
    if msgid == MSG_AIRDATA and len(payload) >= 11:
        airspeed, groundspeed, climb, throttle, alt_amsl = struct.unpack_from("<HHhBi", payload, 0)
        return f"as={airspeed} gs={groundspeed} climb={climb} thr={throttle} alt={alt_amsl}"
    if msgid == MSG_COMMAND_ACK and len(payload) >= 3:
        cmd_id, result, seq = struct.unpack_from("<BBB", payload, 0)
        return f"cmd={cmd_id} res={result} seq={seq}"
    if msgid == MSG_COMMAND_DO and len(payload) >= 7:
        cmd_id, p1, tgt_sys, tgt_comp = struct.unpack_from("<BfBB", payload, 0)
        return f"cmd={cmd_id} p1={p1:.3f} tgt={tgt_sys}.{tgt_comp}"
    if msgid == MSG_COMMAND_SHORT and len(payload) >= 19:
        cmd_id, p1, p2, p3, p4, tgt_sys, tgt_comp = struct.unpack_from("<BffffBB", payload, 0)
        return (
            f"cmd={cmd_id} p1={p1:.3f} p2={p2:.3f} p3={p3:.3f} p4={p4:.3f} "
            f"tgt={tgt_sys}.{tgt_comp}"
        )
    if msgid == MSG_COMMAND_LONG and len(payload) >= 31:
        cmd_id, p1, p2, p3, p4, p5, p6, p7, tgt_sys, tgt_comp = struct.unpack_from(
            "<BfffffffBB", payload, 0
        )
        return (
            f"cmd={cmd_id} p1={p1:.3f} p2={p2:.3f} p3={p3:.3f} p4={p4:.3f} "
            f"p5={p5:.3f} p6={p6:.3f} p7={p7:.3f} tgt={tgt_sys}.{tgt_comp}"
        )
    if msgid == MSG_PARAM_SET and len(payload) >= 23:
        tgt_sys, tgt_comp = struct.unpack_from("<BB", payload, 0)
        param_id = struct.unpack_from("<16s", payload, 2)[0].split(b"\x00", 1)[0].decode("ascii", "replace")
        param_val, param_type = struct.unpack_from("<fB", payload, 18)
        return f"tgt={tgt_sys}.{tgt_comp} id={param_id} val={param_val:.3f} type={param_type}"
    if msgid == MSG_PARAM_GET and len(payload) >= 18:
        tgt_sys, tgt_comp = struct.unpack_from("<BB", payload, 0)
        param_id = struct.unpack_from("<16s", payload, 2)[0].split(b"\x00", 1)[0].decode("ascii", "replace")
        return f"tgt={tgt_sys}.{tgt_comp} id={param_id}"
    return f"len={len(payload)} data={payload.hex()}"


def cmd_do_payload(cmd_id: int, p1: float, target_system: int, target_component: int) -> bytes:
    return struct.pack("<BfBB", cmd_id, p1, target_system, target_component)


def cmd_short_payload(
    cmd_id: int, p1: float, p2: float, p3: float, p4: float, target_system: int, target_component: int
) -> bytes:
    return struct.pack("<BffffBB", cmd_id, p1, p2, p3, p4, target_system, target_component)


def cmd_long_payload(
    cmd_id: int,
    p1: float,
    p2: float,
    p3: float,
    p4: float,
    p5: float,
    p6: float,
    p7: float,
    target_system: int,
    target_component: int,
) -> bytes:
    return struct.pack("<BfffffffBB", cmd_id, p1, p2, p3, p4, p5, p6, p7, target_system, target_component)


class _TcpLink:
    def __init__(self, host: str, port: int):
        self._sock = socket.create_connection((host, port), timeout=5.0)
        self._sock.settimeout(0.2)

    def read(self, size: int) -> bytes:
        try:
            return self._sock.recv(size)
        except socket.timeout:
            return b""

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def close(self) -> None:
        self._sock.close()


def _parse_tcp_addr(addr: str):
    if ":" not in addr:
        raise ValueError("tcp address must be host:port or :port")
    host, port_str = addr.rsplit(":", 1)
    if host in ("", "*", "0.0.0.0"):
        host = "127.0.0.1"
    port = int(port_str)
    if port <= 0 or port > 65535:
        raise ValueError("invalid port")
    return host, port


def read_loop(link, stop_evt: threading.Event, live_flag, buf_log, buf_lock):
    buf = bytearray()
    while not stop_evt.is_set():
        data = link.read(256)
        if data:
            buf.extend(data)
            for sysid, compid, seq, msgid, payload in parse_frames(buf):
                name = MSG_NAMES.get(msgid, f"0x{msgid:02X}")
                msg = decode_payload(msgid, payload)
                ts = time.time()
                line = f"{ts:.3f} | {name:<5} | sys={sysid:03d} comp={compid:03d} seq={seq:03d} | {msg}"
                with buf_lock:
                    buf_log.append(line)
                if live_flag["enabled"]:
                    print(line)


def prompt_float(label: str):
    val = input(label).strip()
    if val == "":
        return None
    return float(val)


def send_cmd_do(link, seq, sysid, compid, cmd_id, p1, target_sys, target_comp):
    payload = cmd_do_payload(cmd_id, p1, target_sys, target_comp)
    frame = build_frame(sysid, compid, seq, MSG_COMMAND_DO, payload)
    link.write(frame)


def send_cmd_short(link, seq, sysid, compid, cmd_id, p1, p2, p3, p4, target_sys, target_comp):
    payload = cmd_short_payload(cmd_id, p1, p2, p3, p4, target_sys, target_comp)
    frame = build_frame(sysid, compid, seq, MSG_COMMAND_SHORT, payload)
    link.write(frame)


def send_cmd_long(link, seq, sysid, compid, cmd_id, p1, p2, p3, p4, p5, p6, p7, target_sys, target_comp):
    payload = cmd_long_payload(cmd_id, p1, p2, p3, p4, p5, p6, p7, target_sys, target_comp)
    frame = build_frame(sysid, compid, seq, MSG_COMMAND_LONG, payload)
    link.write(frame)


def main() -> int:
    parser = argparse.ArgumentParser(description="SMP menu GCS (monitor + CMD sender)")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", help="Serial port (e.g. /tmp/smp_gcs, /dev/ttyUSB0, COM4)")
    group.add_argument("--tcp", help="TCP address (host:port or :port)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--sysid", type=int, default=1, help="Sysid")
    parser.add_argument("--compid", type=int, default=1, help="Compid")
    parser.add_argument("--log-size", type=int, default=200, help="Telemetry ring buffer size")
    parser.add_argument("--live", action="store_true", help="Print telemetry live")
    args = parser.parse_args()

    if args.tcp:
        host, port = _parse_tcp_addr(args.tcp)
        link = _TcpLink(host, port)
        print(f"connected to tcp://{host}:{port}")
    else:
        if serial is None:
            raise SystemExit("pyserial is required for serial mode (pip install pyserial)")
        link = serial.Serial(args.port, args.baud, timeout=0.2)
        print(f"connected to {args.port} @ {args.baud}")

    try:
        stop_evt = threading.Event()
        live_flag = {"enabled": args.live}
        buf_log = deque(maxlen=max(10, args.log_size))
        buf_lock = threading.Lock()
        t = threading.Thread(target=read_loop, args=(link, stop_evt, live_flag, buf_log, buf_lock), daemon=True)
        t.start()

        seq = 0
        while True:
            print("\n=== SMP GCS MENU ===")
            print("1) COMMAND_DO (cmd_id, p1)")
            print("2) COMMAND_SHORT (cmd_id, p1..p4)")
            print("3) COMMAND_LONG (cmd_id, p1..p7)")
            print("4) RAW PAYLOAD HEX")
            print("5) TOGGLE LIVE TELEMETRY")
            print("6) SHOW LAST 20 LOGS")
            print("7) QUIT")
            choice = input("> ").strip()

            if choice == "1":
                cmd_id = int(input("cmd_id (0-255): ").strip())
                p1 = prompt_float("p1 (blank=NaN): ")
                tgt_sys = int(input("target_system (0-255): ").strip())
                tgt_comp = int(input("target_component (0-255): ").strip())
                p1_val = p1 if p1 is not None else math.nan
                send_cmd_do(link, seq, args.sysid, args.compid, cmd_id, p1_val, tgt_sys, tgt_comp)
            elif choice == "2":
                cmd_id = int(input("cmd_id (0-255): ").strip())
                p1 = prompt_float("p1 (blank=NaN): ")
                p2 = prompt_float("p2 (blank=NaN): ")
                p3 = prompt_float("p3 (blank=NaN): ")
                p4 = prompt_float("p4 (blank=NaN): ")
                tgt_sys = int(input("target_system (0-255): ").strip())
                tgt_comp = int(input("target_component (0-255): ").strip())
                p1_val = p1 if p1 is not None else math.nan
                p2_val = p2 if p2 is not None else math.nan
                p3_val = p3 if p3 is not None else math.nan
                p4_val = p4 if p4 is not None else math.nan
                send_cmd_short(
                    link,
                    seq,
                    args.sysid,
                    args.compid,
                    cmd_id,
                    p1_val,
                    p2_val,
                    p3_val,
                    p4_val,
                    tgt_sys,
                    tgt_comp,
                )
            elif choice == "3":
                cmd_id = int(input("cmd_id (0-255): ").strip())
                p1 = prompt_float("p1 (blank=NaN): ")
                p2 = prompt_float("p2 (blank=NaN): ")
                p3 = prompt_float("p3 (blank=NaN): ")
                p4 = prompt_float("p4 (blank=NaN): ")
                p5 = prompt_float("p5 (blank=NaN): ")
                p6 = prompt_float("p6 (blank=NaN): ")
                p7 = prompt_float("p7 (blank=NaN): ")
                tgt_sys = int(input("target_system (0-255): ").strip())
                tgt_comp = int(input("target_component (0-255): ").strip())
                send_cmd_long(
                    link,
                    seq,
                    args.sysid,
                    args.compid,
                    cmd_id,
                    p1 if p1 is not None else math.nan,
                    p2 if p2 is not None else math.nan,
                    p3 if p3 is not None else math.nan,
                    p4 if p4 is not None else math.nan,
                    p5 if p5 is not None else math.nan,
                    p6 if p6 is not None else math.nan,
                    p7 if p7 is not None else math.nan,
                    tgt_sys,
                    tgt_comp,
                )
            elif choice == "4":
                msgid = int(input("msgid (e.g. 0x01): ").strip(), 0)
                payload_hex = input("payload hex (no spaces): ").strip()
                payload = bytes.fromhex(payload_hex)
                frame = build_frame(args.sysid, args.compid, seq, msgid, payload)
                link.write(frame)
            elif choice == "5":
                live_flag["enabled"] = not live_flag["enabled"]
                print(f"live telemetry = {live_flag['enabled']}")
            elif choice == "6":
                with buf_lock:
                    tail = list(buf_log)[-20:]
                if not tail:
                    print("(no telemetry yet)")
                else:
                    print("\n".join(tail))
            elif choice == "7":
                break
            else:
                print("unknown choice")
                continue

            seq = (seq + 1) & 0xFF

        stop_evt.set()
        t.join(timeout=1.0)
    finally:
        try:
            link.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
