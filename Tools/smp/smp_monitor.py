#!/usr/bin/env python3
import argparse
import socket
import struct
import time

import serial


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

HEALTH_COMPONENTS = {
    0x0000000000000002: "absolute_pressure",
    0x0000000000000004: "differential_pressure",
    0x0000000000000008: "g"
    "ps",
    0x0000000000000010: "optical_flow",
    0x0000000000000020: "vision_position",
    0x0000000000000040: "distance_sensor",
    0x0000000000000080: "remote_control",
    0x0000000000000100: "motors_escs",
    0x0000000000000200: "utm",
    0x0000000000000400: "logging",
    0x0000000000000800: "battery",
    0x0000000000001000: "communication_links",
    0x0000000000002000: "rate_controller",
    0x0000000000004000: "attitude_controller",
    0x0000000000008000: "position_controller",
    0x0000000000010000: "attitude_estimate",
    0x0000000000020000: "local_position_estimate",
    0x0000000000040000: "mission",
    0x0000000000080000: "avoidance",
    0x0000000000100000: "system",
    0x0000000000200000: "camera",
    0x0000000000400000: "gimbal",
    0x0000000000800000: "payload",
    0x0000000001000000: "global_position_estimate",
    0x0000000002000000: "storage",
    0x0000000004000000: "parachute",
    0x0000000008000000: "magnetometer",
    0x0000000010000000: "accel",
    0x0000000020000000: "gyro",
    0x0000000040000000: "open_drone_id",
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


def decode_flags(mask: int) -> str:
    names = [name for bit, name in HEALTH_COMPONENTS.items() if mask & bit]
    return ",".join(names) if names else "-"


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


def _parse_udp_addr(addr: str):
    if ":" not in addr:
        raise ValueError("udp address must be host:port or :port")
    host, port_str = addr.rsplit(":", 1)
    if host in ("", "*"):
        host = "0.0.0.0"
    port = int(port_str)
    if port <= 0 or port > 65535:
        raise ValueError("invalid port")
    return host, port


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor SMP v1 frames.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", help="Serial port (e.g. COM4 or /dev/ttyACM0)")
    group.add_argument("--tcp", help="TCP address (host:port or :port)")
    group.add_argument("--udp", help="UDP address (host:port or :port)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    args = parser.parse_args()

    buf = bytearray()
    if args.tcp:
        host, port = _parse_tcp_addr(args.tcp)
        with socket.create_connection((host, port), timeout=5.0) as sock:
            sock.settimeout(0.2)
            print(f"listening on tcp://{host}:{port}")
            while True:
                try:
                    data = sock.recv(256)
                except socket.timeout:
                    data = b""
                if data:
                    buf.extend(data)
                    for sysid, compid, seq, msgid, payload in parse_frames(buf):
                        msg = decode_payload(msgid, payload)
                        ts = time.time()
                        name = MSG_NAMES.get(msgid, f"0x{msgid:02X}")
                        print(f"{ts:.3f} | {name:<5} | sys={sysid:03d} comp={compid:03d} seq={seq:03d} | {msg}")
    elif args.udp:
        host, port = _parse_udp_addr(args.udp)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((host, port))
            sock.settimeout(0.2)
            print(f"listening on udp://{host}:{port}")
            while True:
                try:
                    data, _ = sock.recvfrom(256)
                except socket.timeout:
                    data = b""
                if data:
                    buf.extend(data)
                    for sysid, compid, seq, msgid, payload in parse_frames(buf):
                        msg = decode_payload(msgid, payload)
                        ts = time.time()
                        name = MSG_NAMES.get(msgid, f"0x{msgid:02X}")
                        print(f"{ts:.3f} | {name:<5} | sys={sysid:03d} comp={compid:03d} seq={seq:03d} | {msg}")
    else:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            print(f"listening on {args.port} @ {args.baud}")
            while True:
                data = ser.read(256)
                if data:
                    buf.extend(data)
                    for sysid, compid, seq, msgid, payload in parse_frames(buf):
                        msg = decode_payload(msgid, payload)
                        ts = time.time()
                        name = MSG_NAMES.get(msgid, f"0x{msgid:02X}")
                        print(f"{ts:.3f} | {name:<5} | sys={sysid:03d} comp={compid:03d} seq={seq:03d} | {msg}")


if __name__ == "__main__":
    main()
