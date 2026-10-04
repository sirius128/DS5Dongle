#!/usr/bin/env python3
"""Log DS5Dongle haptics diagnostics (feature report 0xF9 tail) to CSV.

Needs a firmware built from the debug/haptic-diag branch. Polls once per
interval and also samples the controller's input report to track the
controller clock (SensorTimestamp, nominal 3 MHz) against the host clock.

    pip install hidapi
    python3 tools/haptic_diag_log.py [-i 1.0] [-o haptic_diag.csv]
"""
import argparse
import struct
import sys
import time

import hid

VID, PID = 0x054C, 0x0CE6


def open_shared():
    """Open the dongle without seizing it.

    hidapi on macOS opens devices with kIOHIDOptionsTypeSeizeDevice by default,
    which steals the controller's input from games while logging. hid_init()
    resets the flag, so enumerate first (forces hid_init) and then clear it.
    """
    hid.enumerate(VID, PID)
    if sys.platform == "darwin":
        import ctypes
        lib = ctypes.CDLL(hid.__file__)
        lib.hid_darwin_set_open_exclusive.argtypes = [ctypes.c_int]
        lib.hid_darwin_set_open_exclusive(0)
    dev = hid.device()
    dev.open(VID, PID)
    return dev


def read_diag(dev):
    r = bytes(dev.get_feature_report(0xF9, 64))
    if r and r[0] == 0xF9:
        r = r[1:]
    if len(r) < 52 or r[2] != 1:
        raise RuntimeError("firmware does not export haptic diagnostics (0xF9 len=%d)" % len(r))
    u32 = struct.unpack_from("<9I", r, 4)
    u16 = struct.unpack_from("<4H", r, 40)
    return {
        "rssi": struct.unpack_from("<b", r, 0)[0],
        "connected": r[3],
        "uptime_ms": u32[0],
        "usb_frames_in": u32[1],
        "rs_frames_out": u32[2],
        "blocks_made": u32[3],
        "blocks_overwritten": u32[4],
        "bt39_sent": u32[5],
        "bt_write_fail": u32[6],
        "l2cap_send_err": u32[7],
        "out_clipped": u32[8],
        "in_peak": u16[0],
        "out_peak": u16[1],
        "in_rms": u16[2],
        "out_rms": u16[3],
        "send_fifo": r[48],
        "send_fifo_max": r[49],
        "haptics_fifo": r[50],
    }


def read_input(dev):
    """Return (SensorTimestamp, battery byte) from the freshest 0x01 input report.

    hidapi buffers input reports between polls, so drain the backlog first;
    using a stale report skews the clock estimate by the buffering delay.
    """
    latest = None
    for _ in range(64):  # bounded: reports keep arriving at ~660 Hz
        r = dev.read(64, 0)
        if not r:
            break
        if r[0] == 0x01 and len(r) >= 64:
            latest = r
    if latest is None:
        r = dev.read(64, 100)
        if r and r[0] == 0x01 and len(r) >= 64:
            latest = r
    if latest is None:
        return None, None
    b = bytes(latest[1:])
    return struct.unpack_from("<I", b, 27)[0], b[52]


COLS = ["host_s", "uptime_s", "connected", "rssi", "battery",
        "usb_fps", "rs_fps", "bt39_ps", "ctrl_ppm",
        "in_peak", "in_rms", "out_peak", "out_rms",
        "d_overwritten", "d_bt_write_fail", "d_l2cap_err", "d_clipped",
        "send_fifo", "send_fifo_max", "haptics_fifo"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interval", type=float, default=1.0)
    ap.add_argument("-o", "--output", default="haptic_diag.csv")
    args = ap.parse_args()

    dev = open_shared()
    out = open(args.output, "a", buffering=1)
    if out.tell() == 0:
        out.write(",".join(COLS) + "\n")
    print("\t".join(COLS))

    t0 = time.monotonic()
    prev = None
    pts = []  # (host_t, unwrapped controller ts) since the controller (re)connected
    ts_last = None
    ts_wraps = 0
    while True:
        now = time.monotonic()
        d = read_diag(dev)
        ts, batt = read_input(dev)
        t_ts = time.monotonic()
        ppm = ""
        if ts is not None and d["connected"]:
            if ts_last is not None and ts < ts_last:
                if ts_last - ts > 2**31:
                    ts_wraps += 1      # 32-bit counter wrapped
                else:
                    pts, ts_wraps = [], 0  # controller restarted: clock reset
            ts_last = ts
            pts.append((t_ts, ts + ts_wraps * 2**32))
            if pts[-1][0] - pts[0][0] > 10:
                # least-squares slope: immune to the constant report-buffering offset
                n = len(pts)
                hx = [p[0] - pts[0][0] for p in pts]
                cy = [p[1] - pts[0][1] for p in pts]
                mx, my = sum(hx) / n, sum(cy) / n
                sxx = sum((x - mx) ** 2 for x in hx)
                sxy = sum((x - mx) * (y - my) for x, y in zip(hx, cy))
                ppm = "%+.1f" % ((sxy / sxx / 3e6 - 1) * 1e6)
        elif not d["connected"]:
            pts, ts_last, ts_wraps = [], None, 0
        if prev is not None:
            dt = (d["uptime_ms"] - prev["uptime_ms"]) / 1000 or 1
            delta = lambda k: d[k] - prev[k]
            row = [
                "%.1f" % (now - t0), "%.1f" % (d["uptime_ms"] / 1000), d["connected"], d["rssi"],
                "" if batt is None else "%d%%" % ((batt & 0xF) * 10),
                "%.1f" % (delta("usb_frames_in") / dt), "%.2f" % (delta("rs_frames_out") / dt),
                "%.2f" % (delta("bt39_sent") / dt), ppm,
                d["in_peak"], d["in_rms"], d["out_peak"], d["out_rms"],
                delta("blocks_overwritten"), delta("bt_write_fail"), delta("l2cap_send_err"),
                delta("out_clipped"), d["send_fifo"], d["send_fifo_max"], d["haptics_fifo"],
            ]
            line = [str(x) for x in row]
            out.write(",".join(line) + "\n")
            print("\t".join(line))
        prev = d
        time.sleep(max(0.0, args.interval - (time.monotonic() - now)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
