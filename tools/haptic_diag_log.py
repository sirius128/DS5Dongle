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
    """Return (SensorTimestamp, battery byte) from the next 0x01 input report."""
    for _ in range(50):
        r = dev.read(64, 50)
        if r and r[0] == 0x01 and len(r) >= 64:
            b = bytes(r[1:])
            return struct.unpack_from("<I", b, 27)[0], b[52]
    return None, None


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

    dev = hid.device()
    dev.open(VID, PID)
    out = open(args.output, "a", buffering=1)
    if out.tell() == 0:
        out.write(",".join(COLS) + "\n")
    print("\t".join(COLS))

    t0 = time.monotonic()
    prev = None
    ts_base = None  # (host_t, unwrapped controller ts)
    ts_last = None
    ts_wraps = 0
    while True:
        now = time.monotonic()
        d = read_diag(dev)
        ts, batt = read_input(dev)
        ppm = ""
        if ts is not None:
            if ts_last is not None and ts < ts_last:
                ts_wraps += 1
            ts_last = ts
            ts_u = ts + ts_wraps * 2**32
            if ts_base is None or not d["connected"]:
                ts_base = (now, ts_u)
            elif now - ts_base[0] > 5:
                rate = (ts_u - ts_base[1]) / (now - ts_base[0])
                ppm = "%+.1f" % ((rate / 3e6 - 1) * 1e6)
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
