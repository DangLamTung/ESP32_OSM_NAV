"""sd_capture.py — hard-reset the ESP32-S3 (USB-Serial/JTAG) and capture the
boot serial log, so the one-time sd_card_init() result in setup() is visible.

Usage:
    python sd_capture.py [PORT] [DURATION_SECONDS] [OUTFILE]

The DTR/RTS sequence mirrors esptool's --after hard_reset (works on the
ES3C28P/ES3N28P native USB-Serial/JTAG, verified during flashing).
"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else 'COM9'
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 45
out = sys.argv[3] if len(sys.argv) > 3 else None

s = serial.Serial(port, 115200, timeout=1)

# --- hard reset (esptool-style) so the app boots fresh ---
def _set(dtr, rts):
    s.setDTR(dtr)  # DTR=False  -> IO0=HIGH (boot app, not download)
    s.setRTS(rts)  # RTS=True   -> EN=LOW (chip held in reset)
    time.sleep(0.1)

# IO0=HIGH the whole time, pulse EN low->high => reboot into the app.
_set(False, True)    # EN=LOW, chip in reset, IO0=HIGH
time.sleep(0.15)
_set(False, False)   # EN=HIGH, chip out of reset, IO0=HIGH -> app boots
time.sleep(0.05)

# --- capture ---
end = time.time() + dur
f = open(out, 'w', encoding='utf-8', errors='replace') if out else None
try:
    while time.time() < end:
        n = s.in_waiting
        if n:
            txt = s.read(n).decode('utf-8', errors='replace')
            sys.stdout.write(txt)
            sys.stdout.flush()
            if f:
                f.write(txt)
                f.flush()
        else:
            time.sleep(0.02)
finally:
    if f:
        f.close()
    s.close()
