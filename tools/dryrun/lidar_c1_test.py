#!/usr/bin/env python3
"""Dry-run check for the RPLIDAR C1: start a scan, read a few rotations, print a summary.

Usage: python3 lidar_c1_test.py [/dev/ttyUSB0] [--baud 460800] [--turns 5]
Needs pyserial (sudo apt install python3-serial). Moves nothing but the LiDAR's own motor.
"""
import argparse, statistics, sys, time

import serial  # type: ignore

DESCRIPTOR = bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])


def parse_node(b):
    """One 5-byte scan node -> (new_turn, quality, angle_deg, dist_mm), or None if the bytes are out of step."""
    s, not_s = b[0] & 1, (b[0] >> 1) & 1
    if s == not_s or not (b[1] & 1):
        return None
    angle = ((b[1] >> 1) | (b[2] << 7)) / 64.0
    dist = (b[3] | (b[4] << 8)) / 4.0
    return bool(s), b[0] >> 2, angle, dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--turns", type=int, default=5)
    a = ap.parse_args()
    ser = serial.Serial(a.port, a.baud, timeout=1)
    ser.write(bytes([0xA5, 0x25])); time.sleep(0.05); ser.reset_input_buffer()
    ser.write(bytes([0xA5, 0x20]))
    desc = ser.read(7)
    if desc != DESCRIPTOR:
        print(f"FAIL: no scan reply from {a.port} at {a.baud} baud (got {desc.hex() or 'nothing'}). "
              "Check the port, the cable and the baud rate.")
        return 1
    turns, cur, bad, t0, t_first = [], [], 0, time.time(), None
    buf = b""
    while len(turns) < a.turns + 1 and time.time() - t0 < 15:
        buf += ser.read(500)
        while len(buf) >= 5:
            n = parse_node(buf[:5])
            if n is None:
                buf = buf[1:]; bad += 1; continue
            buf = buf[5:]
            new, q, ang, d = n
            if new:
                if cur:
                    turns.append(cur)
                    if t_first is None:
                        t_first = time.time()
                cur = []
            cur.append((q, ang, d))
    ser.write(bytes([0xA5, 0x25])); ser.close()
    full = turns[1:]  # the first turn starts mid-rotation
    if not full:
        print("FAIL: scan started but no complete rotation in 15 s. Is the LiDAR spinning?")
        return 1
    hz = (len(full)) / max(1e-3, time.time() - t_first)
    pts = [len(t) for t in full]
    good = [d for t in full for (q, _, d) in t if q > 0 and d > 0]
    print(f"OK: {len(full)} rotations at about {hz:.1f} Hz, {min(pts)}-{max(pts)} points per rotation, "
          f"{len(good)} valid points, {bad} skipped bytes")
    if good:
        print(f"    distance mm: min {min(good):.0f}, median {statistics.median(good):.0f}, max {max(good):.0f}")
    print("    Expected: about 10 Hz, 400-500 points, median a few metres indoors.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
