#!/usr/bin/env python3
"""verify_icons_sd.py — keep re-uploading the 10 icon PNGs until every one
reports OK from the board (SD write is flaky on this hardware: a failed write
unlinks the file, so we round-trip until all confirm in the same pass)."""
import os
import subprocess
import sys

ICONS = ["turn_left", "turn_right", "turn_slight_left", "turn_slight_right",
         "straight", "u_turn", "speed", "gear", "speed_limit", "nav_arrow"]
HERE = os.path.dirname(os.path.abspath(__file__))
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem3101"

files = [os.path.join(HERE, "..", "assets", "icon", n + ".png") for n in ICONS]

confirmed = set()
for round_no in range(1, 10):
    remaining = [f for f in files if os.path.basename(f) not in confirmed]
    if not remaining:
        break
    print(f"--- round {round_no}: {len(remaining)} to confirm ---")
    r = subprocess.run([sys.executable, os.path.join(HERE, "upload_icon_sd.py"),
                        "--port", PORT, "--files", *remaining],
                       capture_output=True, text=True)
    out = r.stdout + r.stderr
    failed = {os.path.basename(l.split()[-1])
              for l in out.splitlines() if l.startswith("  FAIL")}
    for n in ICONS:
        if n not in failed:
            confirmed.add(n)
    print(f"  confirmed so far: {sorted(confirmed)}")
    if failed:
        print(f"  still failing: {sorted(failed)}")

print("\n=== FINAL ===")
print("confirmed on SD:", sorted(confirmed))
missing = [n for n in ICONS if n not in confirmed]
print("missing:", missing if missing else "NONE - all 10 icons on card")
