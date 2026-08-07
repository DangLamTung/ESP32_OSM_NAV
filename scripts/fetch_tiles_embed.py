#!/usr/bin/env python3
"""Fetch the OSM tiles covering the Bến Thành demo area (zoom 16, 2x2 block)
and report their sizes so we can decide how many to embed in flash."""
import urllib.request, os, sys

Z = 16
TILES = [(52191, 30794), (52191, 30795), (52192, 30794), (52192, 30795)]
OUT = os.path.join(os.path.dirname(__file__), "..", "preview", "tiles")

def fetch(x, y):
    url = f"https://tile.openstreetmap.org/{Z}/{x}/{y}.png"
    req = urllib.request.Request(url, headers={"User-Agent": "car_nav/1.0 (tile embed)"})
    with urllib.request.urlopen(req, timeout=45) as r:
        return r.read()

os.makedirs(OUT, exist_ok=True)
total = 0
for (x, y) in TILES:
    png = fetch(x, y)
    total += len(png)
    fn = os.path.join(OUT, f"tile_{x}_{y}.png")
    with open(fn, "wb") as f:
        f.write(png)
    print(f"{x},{y}: {len(png)} bytes -> {fn}")

print(f"TOTAL 4 tiles: {total} bytes")
print(f"Flash free is ~114 KB (0x1bf00) -> 2x2 likely too big; 1-2 tiles fit.")
