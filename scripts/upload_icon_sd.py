#!/usr/bin/env python3
"""upload_icon_sd.py — push icon PNGs onto the SD card mounted in the board,
via USB-Serial/JTAG reader mode.

Same protocol as upload_tiles_serial.py (magic OSMUP1, FILE <size> <rel>,
raw bytes, OK, DONE), but for a set of icon files at the SD root. Reboots
the board into normal nav mode when done.

Usage:
    python upload_icon_sd.py --port /dev/cu.usbmodem3101 \
        --files ../assets/icon/*.png
"""
import argparse
import os
import sys
import time

import serial

MAGIC = b"OSMUP1\n"
BAUD = 115200

LOG_PREFIXES = ("I (", "E (", "W (", "V (", "D (", "Guru Meditation")


def reset_boot_app(s):
    """esptool-style hard reset that boots into the app (IO0 stays HIGH)."""
    s.setDTR(False)
    s.setRTS(True)          # EN=LOW (reset), IO0=HIGH
    time.sleep(0.12)
    s.setRTS(False)         # EN=HIGH -> app boots
    time.sleep(0.05)
    s.setDTR(False)


class LineReader:
    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def read_until_line(self, timeout_s):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                line = line.strip(b"\r").decode("utf-8", errors="replace")
                if line:
                    return line
            time.sleep(0.01)
        return None


def wait_for(s, reader, want, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        line = reader.read_until_line(min(2.0, end - time.time() + 0.2))
        if line is None:
            continue
        if line.startswith(LOG_PREFIXES):
            continue
        if line == want:
            return True
        if line.startswith("ERR"):
            print(f"  ! firmware error: {line}")
            return False
        print(f"  ? unexpected: {line}")
    return False


def send_exact(s, data):
    s.write(data)
    s.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/cu.usbmodem3101")
    ap.add_argument("--files", nargs="+", default=[os.path.join(
        os.path.dirname(__file__), "..", "assets", "icon", "nav_arrow.png")])
    ap.add_argument("--magic-delay", type=float, default=1.2)
    args = ap.parse_args()

    files = [f for f in args.files if os.path.isfile(f)]
    if not files:
        sys.exit("no files found")
    total = sum(os.path.getsize(f) for f in files)
    print(f"uploading {len(files)} icons ({total} B)")

    s = serial.Serial(args.port, BAUD, timeout=1)
    reader = LineReader(s)

    print(f"resetting board on {args.port}...")
    reset_boot_app(s)
    time.sleep(args.magic_delay)

    print("sending upload magic...")
    send_exact(s, MAGIC)
    if not wait_for(s, reader, "READY", 8.0):
        sys.exit("no READY from firmware (board not booted / wrong firmware?)")

    ok = fail = 0
    for path in files:
        size = os.path.getsize(path)
        # icons live in /sdcard/icon/ (firmware's mkdir_p creates the folder)
        dest = "icon/" + os.path.basename(path)
        send_exact(s, f"FILE {size} {dest}\n".encode())
        with open(path, "rb") as f:
            while True:
                chunk = f.read(4096)
                if not chunk:
                    break
                send_exact(s, chunk)
        if wait_for(s, reader, "OK", 30.0):
            ok += 1
        else:
            fail += 1
            print(f"  FAIL {dest}")

    print(f"sending DONE ({ok} ok, {fail} fail)...")
    send_exact(s, b"DONE\n")
    wait_for(s, reader, "OK", 10.0)

    print(f"\nuploaded {ok}/{len(files)} icons, ~{total/1024:.1f} KB")
    print("board is rebooting into nav mode")
    s.close()


if __name__ == "__main__":
    main()
