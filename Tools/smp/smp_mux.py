#!/usr/bin/env python3
"""
PTY mux for SMP: one PX4 endpoint, two clients (GCS + monitor).
Creates three PTYs and bridges:
  PX4 -> (GCS, MON)
  GCS/MON -> PX4
"""

import argparse
import os
import pty
import termios
import tty
import selectors
import signal
import sys


def _make_pty(link_path: str):
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # Put the slave side in raw mode so the PTY doesn't mangle bytes.
    tty.setraw(slave_fd)

    if os.path.islink(link_path) or os.path.exists(link_path):
        os.unlink(link_path)
    os.symlink(slave_name, link_path)

    return master_fd, slave_fd, slave_name


def _set_nonblock(fd: int):
    flags = os.O_NONBLOCK
    os.set_blocking(fd, False)
    return flags


def _write_all(fd: int, data: bytes):
    if not data:
        return
    try:
        os.write(fd, data)
    except BlockingIOError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="SMP PTY mux (PX4 <-> GCS/MON)")
    parser.add_argument("--px4", default="/tmp/smp_px4", help="PX4 PTY link path")
    parser.add_argument("--gcs", default="/tmp/smp_gcs", help="GCS PTY link path")
    parser.add_argument("--mon", default="/tmp/smp_mon", help="Monitor PTY link path")
    parser.add_argument("--mon-px4", help="PX4->MON PTY link path (directional)")
    parser.add_argument("--mon-gcs", help="GCS->MON PTY link path (directional)")
    parser.add_argument("--verbose", action="store_true", help="Print PTY paths")
    parser.add_argument("--tap-gcs", action="store_true", help="Mirror GCS->MON for debugging")
    args = parser.parse_args()

    px4_master, px4_slave, px4_name = _make_pty(args.px4)
    gcs_master, gcs_slave, gcs_name = _make_pty(args.gcs)
    mon_master, mon_slave, mon_name = _make_pty(args.mon)
    mon_px4_master = mon_px4_slave = mon_px4_name = None
    mon_gcs_master = mon_gcs_slave = mon_gcs_name = None
    if args.mon_px4:
        mon_px4_master, mon_px4_slave, mon_px4_name = _make_pty(args.mon_px4)
    if args.mon_gcs:
        mon_gcs_master, mon_gcs_slave, mon_gcs_name = _make_pty(args.mon_gcs)

    _set_nonblock(px4_master)
    _set_nonblock(gcs_master)
    _set_nonblock(mon_master)
    if mon_px4_master is not None:
        _set_nonblock(mon_px4_master)
    if mon_gcs_master is not None:
        _set_nonblock(mon_gcs_master)

    if args.verbose:
        print(f"PX4 PTY : {args.px4} -> {px4_name}")
        print(f"GCS PTY : {args.gcs} -> {gcs_name}")
        print(f"MON PTY : {args.mon} -> {mon_name}")
        if mon_px4_name:
            print(f"MON PX4: {args.mon_px4} -> {mon_px4_name}")
        if mon_gcs_name:
            print(f"MON GCS: {args.mon_gcs} -> {mon_gcs_name}")
        sys.stdout.flush()

    sel = selectors.DefaultSelector()
    sel.register(px4_master, selectors.EVENT_READ, data="px4")
    sel.register(gcs_master, selectors.EVENT_READ, data="gcs")
    sel.register(mon_master, selectors.EVENT_READ, data="mon")

    def _sigterm(_signo, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sigterm)

    try:
        while True:
            for key, _ in sel.select(timeout=0.5):
                src = key.data
                fd = key.fileobj
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    continue

                if src == "px4":
                    _write_all(gcs_master, data)
                    _write_all(mon_master, data)
                    if mon_px4_master is not None:
                        _write_all(mon_px4_master, data)
                else:
                    _write_all(px4_master, data)
                    if src == "gcs" and args.tap_gcs:
                        _write_all(mon_master, data)
                    if src == "gcs" and mon_gcs_master is not None:
                        _write_all(mon_gcs_master, data)
    except KeyboardInterrupt:
        pass
    finally:
        for fd in (px4_master, gcs_master, mon_master, mon_px4_master, mon_gcs_master):
            try:
                if fd is not None:
                    os.close(fd)
            except OSError:
                pass
        for fd in (px4_slave, gcs_slave, mon_slave, mon_px4_slave, mon_gcs_slave):
            try:
                if fd is not None:
                    os.close(fd)
            except OSError:
                pass
        for link in (args.px4, args.gcs, args.mon, args.mon_px4, args.mon_gcs):
            try:
                if link and os.path.islink(link):
                    os.unlink(link)
            except OSError:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
