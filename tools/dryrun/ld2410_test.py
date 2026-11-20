#!/usr/bin/env python3
"""Dry-run check for the LD2410 radar over a USB-serial adapter: print presence as you walk in and out.

Usage: python3 ld2410_test.py [/dev/ttyUSB1] [--baud 256000] [--seconds 30]
Wiring: LD2410 VCC to 5 V, GND to GND, its TX to the adapter's RX (3.3 V TTL adapter).
Needs pyserial (sudo apt install python3-serial).
"""
import argparse, sys, time

import serial  # type: ignore

HEAD, TAIL = bytes([0xF4, 0xF3, 0xF2, 0xF1]), bytes([0xF8, 0xF7, 0xF6, 0xF5])
STATES = {0: "nobody", 1: "moving person", 2: "still person", 3: "moving and still"}


def parse_frames(buf):
    """Yield (state, move_cm, still_cm, detect_cm) for each complete basic report frame; return the leftover bytes."""
    out = []
    while True:
        i = buf.find(HEAD)
        if i < 0:
            return out, buf[-3:]
        if len(buf) < i + 6:
            return out, buf[i:]
        n = buf[i + 4] | (buf[i + 5] << 8)
        end = i + 6 + n + 4
        if len(buf) < end:
            return out, buf[i:]
        data, tail = buf[i + 6:i + 6 + n], buf[i + 6 + n:end]
        if tail == TAIL and len(data) >= 13 and data[1] == 0xAA:
            st = data[2]
            mv = data[3] | (data[4] << 8)
            sd = data[6] | (data[7] << 8)
            dd = data[9] | (data[10] << 8)
            out.append((st, mv, sd, dd))
        buf = buf[end:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--seconds", type=float, default=30)
    a = ap.parse_args()
    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    buf, frames, last, t0, t_print = b"", 0, None, time.time(), 0.0
    print(f"Listening on {a.port} for {a.seconds:.0f} s: walk away, stand still, walk back.")
    while time.time() - t0 < a.seconds:
        buf += ser.read(256)
        got, buf = parse_frames(buf)
        for st, mv, sd, dd in got:
            frames += 1
            now = time.time()
            if st != last or now - t_print > 2:
                print(f"{now - t0:5.1f} s  {STATES.get(st, st)}  moving {mv} cm, still {sd} cm, nearest {dd} cm")
                last, t_print = st, now
    ser.close()
    if frames == 0:
        print("FAIL: no LD2410 frames. Check TX->RX wiring, 5 V power and the baud rate (default 256000).")
        return 1
    print(f"OK: {frames} frames in {a.seconds:.0f} s (expect about 10 per second).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
