#!/usr/bin/env python3
"""Generate a DENSE, street-following 'path ahead' from the real OSRM route:
take only the first CUTOFF_KM of the trip and decimate to N points by distance.
At ~30-80 m spacing the polyline follows z15 streets instead of cutting across
blocks. The board only draws the first 64 route points anyway (path-ahead)."""
import json
import math

d = json.load(open("osrm_bt_vt.json"))
coords = d["routes"][0]["geometry"]["coordinates"]  # [[lon, lat], ...]


def dist(a, b):
    R = 6371000.0
    la1, la2 = math.radians(a[1]), math.radians(b[1])
    dla = la2 - la1
    dlo = math.radians(b[0] - a[0])
    h = math.sin(dla / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin(dlo / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


CUTOFF_KM = 1.5   # keep the whole path inside the tiled 5km radius -> street-correct map
N = 64            # board's NAV_MAX_ROUTE_POINTS

cums = [0.0]
for i in range(1, len(coords)):
    cums.append(cums[-1] + dist(coords[i - 1], coords[i]))

cutoff = CUTOFF_KM * 1000.0
idx = 0
while idx < len(coords) - 1 and cums[idx] < cutoff:
    idx += 1
sub = coords[: idx + 1]
subcums = cums[: idx + 1]
print("kept pts:", len(sub), "covered km:", round(subcums[-1] / 1000, 2),
      "-> spacing m:", round(subcums[-1] / N, 1))

targets = [subcums[-1] * i / (N - 1) for i in range(N)]
out = []
j = 0
for t in targets:
    while j < len(sub) - 2 and subcums[j + 1] < t:
        j += 1
    a, b = sub[j], sub[j + 1]
    d0, d1 = subcums[j], subcums[j + 1]
    f = (t - d0) / (d1 - d0) if d1 > d0 else 0
    out.append((a[1] + (b[1] - a[1]) * f, a[0] + (b[0] - a[0]) * f))  # (lat, lon)

js = "const PATH_AHEAD = [\n"
for i in range(0, len(out), 4):
    row = ", ".join("[%.6f, %.6f]" % p for p in out[i:i + 4])
    js += "  " + row + ",\n"
js += "];"
print(js)
