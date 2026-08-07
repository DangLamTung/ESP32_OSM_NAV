#!/usr/bin/env python3
"""Sanity-check the embedded JPEG tiles in src/map_tiles.h: extract each
array, save as .jpg, decode with PIL, print dims + avg color."""
import io, os, re, sys
from PIL import Image

HDR = os.path.join(os.path.dirname(__file__), "..", "src", "map_tiles.h")
OUT = os.path.join(os.path.dirname(__file__), "..", "preview", "checked")
os.makedirs(OUT, exist_ok=True)

src = open(HDR, encoding="utf-8").read()

# Find each TILE_xxx_yyy array
for m in re.finditer(r"static const uint8_t (TILE_\d+_\d+)\[\] = \{(.*?)\};", src, re.S):
    name = m.group(1)
    nums = [int(x) for x in re.findall(r"\d+", m.group(2))]
    data = bytes(nums)
    jpg = os.path.join(OUT, name + ".jpg")
    open(jpg, "wb").write(data)
    try:
        im = Image.open(io.BytesIO(data))
        im.load()
        im2 = im.convert("RGB")
        px = im2.resize((1, 1)).getpixel((0, 0))
        print(f"{name}: {len(data)} bytes, {im2.size}, avg ~{px}, SOI={data[:2]}, EOI={data[-2:]=}")
    except Exception as e:
        print(f"{name}: FAILED {e!r}  (first bytes {data[:16]})")

# Also verify lookup table entries match arrays
for m in re.finditer(r"\{\s*(\d+),\s*(\d+),\s*(\d+),\s*(TILE_\d+_\d+),\s*sizeof\(\1\) \}", src):
    pass
for m in re.finditer(r"\{\s*(\d+),\s*(\d+),\s*(\d+),\s*(TILE_\d+_\d+),\s*sizeof\(\4\)\s*\}", src):
    z, x, y, name = m.groups()
    if not re.search(rf"{name}\[\]", src):
        print("MISMATCH:", z, x, y, name)
print("done")
