#!/usr/bin/env python3
"""upload_routing_graph.py — push a .rng routing graph to the board's SD card.

Streams a single binary file into /sdcard over the USB-Serial/JTAG reader mode
(the same protocol sd_upload.c uses for tiles), then reboots into nav mode.

Usage:
    python upload_routing_graph.py --port /dev/cu.usbmodem101 /tmp/saigon_core.rng
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
    s.setDTR(False)
    s.setRTS(True)
    time.sleep(0.12)
    s.setRTS(False)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("file", help="the .rng graph file to upload")
    ap.add_argument("--port", default="/dev/cu.usbmodem101")
    ap.add_argument("--dest", default="routing.rng", help="SD path (under /sdcard)")
    args = ap.parse_args()

    size = os.path.getsize(args.file)
    print(f"uploading {args.file} ({size/1048576:.2f} MB) -> /sdcard/{args.dest}")

    s = serial.Serial(args.port, BAUD, timeout=1)
    reader = LineReader(s)

    print(f"resetting board on {args.port}...")
    reset_boot_app(s)
    time.sleep(1.4)

    print("sending upload magic...")
    send = lambda d: (s.write(d), s.flush())
    send(MAGIC)
    if not wait_for(s, reader, "READY", 8.0):
        sys.exit("no READY from firmware (board not booted / wrong firmware?)")

    send(f"FILE {size} {args.dest}\n".encode())
    # Native USB-Serial/JTAG has no flow control and drops bytes under burst
    # writes, so pace in small chunks (board reads 512 B at a time).
    with open(args.file, "rb") as f:
        while True:
            chunk = f.read(512)
            if not chunk:
                break
            send(chunk)
            time.sleep(0.008)
    if not wait_for(s, reader, "OK", 120.0):
        sys.exit("file write failed on board")

    send(b"DONE\n")
    wait_for(s, reader, "OK", 10.0)
    print("done — board rebooting into nav mode")
    s.close()


if __name__ == "__main__":
    main()
