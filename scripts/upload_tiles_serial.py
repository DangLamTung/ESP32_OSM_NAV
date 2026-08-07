#!/usr/bin/env python3
"""
upload_tiles_serial.py — USB reader-mode client for osm_idf.

Streams the downloaded OSM tiles (z/x/y.png) from the PC into the SD card that
is mounted in the board, over the existing USB-Serial/JTAG port (COM9). No card
removal needed.

Workflow:
  1. Make sure the firmware with USB reader mode is flashed (sd_upload.c).
  2. Put a (mounted) SD card in the board and plug in USB -> COMx.
  3. Run:  python upload_tiles_serial.py --dir tiles_hcmc
  4. The script resets the board into the app, sends the upload magic, streams
     every tile into /sdcard/<z>/<x>/<y>.png, then reboots the board into
     normal nav mode.

Usage:
    python upload_tiles_serial.py --port COM9 --dir <folder-with-z-x-y-layout>
"""
import argparse
import glob
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
    """Reads serial bytes and yields full lines, ignoring stale/partial data."""

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
    """Keep reading lines, skipping OSM/ESP log lines, until a line == want."""
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
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(__file__), "..", "tiles_hcmc"))
    ap.add_argument("--magic-delay", type=float, default=1.2,
                    help="seconds to wait after reset before sending the magic")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "**", "*.png"), recursive=True))
    if not files:
        sys.exit(f"no *.png found under {args.dir}")
    print(f"{len(files)} tiles to upload from {args.dir}")

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
    total_bytes = 0
    for i, path in enumerate(files, 1):
        rel = os.path.relpath(path, args.dir).replace("\\", "/")
        size = os.path.getsize(path)
        send_exact(s, f"FILE {size} {rel}\n".encode())
        with open(path, "rb") as f:
            while True:
                chunk = f.read(4096)
                if not chunk:
                    break
                send_exact(s, chunk)
        if wait_for(s, reader, "OK", 30.0):
            ok += 1
            total_bytes += size
        else:
            fail += 1
            print(f"  FAIL {rel}")
        if i % 200 == 0:
            print(f"  ... {i}/{len(files)}")

    print(f"sending DONE ({ok} ok, {fail} fail)...")
    send_exact(s, b"DONE\n")
    wait_for(s, reader, "OK", 10.0)

    print(f"\nuploaded {ok}/{len(files)} files, ~{total_bytes/1048576:.1f} MB")
    print(f"board is rebooting into nav mode")
    s.close()


if __name__ == "__main__":
    main()
