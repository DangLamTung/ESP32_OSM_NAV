#!/usr/bin/env python3
"""Fetch REAL OSM road data (Overpass) around a location and emit the compact
`<road>` XML the ESP32 renderer consumes. Used to generate the demo packet
embedded in car_nav/src/main.c so the boot/driving demo shows real streets
(not hand-typed coordinates)."""
import json, sys, urllib.request, urllib.parse

LAT, LON, RADIUS = 10.7718, 106.6982, 250   # Ben Thanh, HCMC
OVER = "https://overpass-api.de/api/interpreter"

CLASS = {
    "motorway": "motorway", "motorway_link": "motorway",
    "trunk": "trunk", "trunk_link": "trunk",
    "primary": "primary", "primary_link": "primary",
    "secondary": "secondary", "secondary_link": "secondary",
    "tertiary": "tertiary", "tertiary_link": "tertiary",
    "unclassified": "minor", "residential": "minor",
    "living_street": "minor", "service": "minor", "road": "minor",
}
ORDER = {"motorway": 0, "trunk": 1, "primary": 2, "secondary": 3,
         "tertiary": 4, "minor": 5}
MAX_ROADS = 10
MAX_PTS = 18          # per-road point cap (renderer parses 256, keep small)
MAX_TOTAL_PTS = 60    # keep the packet < ~1700 bytes for the C3
ROUTE_MAX_PTS = 12

def fetch():
    q = f"""[out:json][timeout:30];
way(around:{RADIUS},{LAT},{LON})["highway"];
out geom;
"""
    req = urllib.request.Request(OVER, data=urllib.parse.urlencode(
        {"data": q}).encode(), headers={"User-Agent": "navbridge-demo/1.0"})
    with urllib.request.urlopen(req, timeout=45) as r:
        return json.load(r)

def main():
    data = fetch()
    roads = []
    for el in data.get("elements", []):
        if el.get("type") != "way":
            continue
        hw = el.get("tags", {}).get("highway")
        cls = CLASS.get(hw)
        if not cls or hw in ("footway", "pedestrian", "path", "steps",
                             "cycleway", "bridleway", "bus_stop", "platform"):
            continue
        name = el.get("tags", {}).get("name", "") or ""
        ref = el.get("tags", {}).get("ref", "")
        label = name or ref
        geom = el.get("geometry", [])
        if len(geom) < 2:
            continue
        pts = [(g["lat"], g["lon"]) for g in geom]
        # decimate long ways
        if len(pts) > MAX_PTS:
            step = (len(pts) + MAX_PTS - 1) // MAX_PTS
            pts = pts[::step]
        # keep majors always; minors only if named (keeps packet small)
        if cls == "minor" and not label:
            continue
        roads.append((ORDER[cls], cls, label, pts))

    roads.sort(key=lambda r: (r[0], not r[2]))
    total = 0
    out = []
    for cls, label, pts in [(r[1], r[2], r[3]) for r in roads][:MAX_ROADS]:
        pts = pts[:MAX_PTS]
        if total + len(pts) > MAX_TOTAL_PTS:
            break
        s = ",".join(f"{p[0]:.6f},{p[1]:.6f}" for p in pts)
        if label:
            out.append(f'<road cls="{cls}" name="{label}" pts="{s}"/>')
        else:
            out.append(f'<road cls="{cls}" pts="{s}"/>')
        total += len(pts)

    # route: the longest kept road (a real street to drive along)
    best = max((r for r in roads if r[1] != "minor"),
               key=lambda r: len(r[3]), default=None)
    route = ""
    if best:
        rp = best[3]
        if len(rp) > ROUTE_MAX_PTS:
            step = (len(rp) + ROUTE_MAX_PTS - 1) // ROUTE_MAX_PTS
            rp = rp[::step]
        route = "<route pts=\"" + ",".join(
            f"{p[0]:.6f},{p[1]:.6f}" for p in rp) + "\"/>"

    xml = "".join(out)
    print(f"# roads={len(out)} points={total} bytes={len(xml)} "
          f"route_bytes={len(route)}")
    print("ROADS=" + xml)
    print("ROUTE=" + route)
    print("ALL=" + xml + route)

if __name__ == "__main__":
    main()
