#!/usr/bin/env python3
"""
download_offline_tiles.py — download OSM raster tiles for a small area around a
center point and store them in the standard `z/x/y.png` layout, ready to be
copied to the SD card root (the osm_idf app then reads /sdcard/<z>/<x>/<y>.png).

Default center = Bến Thành, HCMC (matches the app's view center).
Compatible with the osm_idf SD-tile loader (see components/OpenStreetMap-esp32).

Usage:
    python download_offline_tiles.py [--center-lat 10.7718 --center-lon 106.6982]
        [--min-zoom 11 --max-zoom 16 --radius-km 5 --out ./tiles --threads 6]

NOTE on OSM tile policy: tile.openstreetmap.org is for light/individual use.
This downloads a SMALL personal area only. Keep --radius-km small, --threads
low, and always send a descriptive User-Agent (we do). Do not bulk-scrape.
"""
import argparse
import hashlib
import math
import os
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed

# The docstring/progress contain non-ASCII (Vietnamese) — force UTF-8 output so
# --help and prints don't crash on a cp1252 Windows console.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

UA = "osm-offline-map/1.0 (ESP32-S3 offline navigation display; contact: osm-offline-map@users.noreply.github.com)"
TILE_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
# Alternative source (works when OSM blocks you):
#   Carto Voyager: https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png
#   Carto Light  : https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png
EARTH_KM = 40075.0  # equatorial circumference

# OSM returns a fixed "Access blocked" placeholder image (HTTP 200) when a
# request is throttled/banned. We detect it by sha256 prefix so it is never
# saved as a real tile.
BLOCKED_SHA256_PREFIX = "b02c44252dac5a5e"

DELAY = 0.5  # seconds to wait before each request (set from --delay)

# app default view center (Bến Thành, HCMC) from src/main.cpp
DEFAULT_LAT, DEFAULT_LON = 10.7718, 106.6982


def lon2tile(lon, z):
    n = 1 << z
    return (lon + 180.0) / 360.0 * n


def lat2tile(lat, z):
    n = 1 << z
    r = math.radians(lat)
    return (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * n


def tile_span_km(lat, z):
    """Approximate width of one tile in km at this latitude (longitude dir)."""
    return EARTH_KM * math.cos(math.radians(lat)) / (1 << z)


def fetch_tile(z, x, y):
    url = TILE_TEMPLATE.format(z=z, x=x, y=y)
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=45) as r:
        return r.read()


def download_one(job):
    z, x, y, out_dir = job
    path = os.path.join(out_dir, str(z), str(x), f"{y}.png")
    if os.path.isfile(path) and os.path.getsize(path) > 0:
        # Skip only REAL tiles — re-fetch anything that is the OSM blocked image.
        try:
            with open(path, "rb") as f:
                if not hashlib.sha256(f.read()).hexdigest().startswith(BLOCKED_SHA256_PREFIX):
                    return (z, x, y, "skip", os.path.getsize(path))
        except OSError:
            pass
    time.sleep(DELAY)  # polite throttle — do not hammer the tile server
    data = None
    for attempt in (1, 2):  # one retry
        try:
            data = fetch_tile(z, x, y)
            break
        except Exception as e:
            if attempt == 2:
                return (z, x, y, f"FAIL {type(e).__name__}", 0)
            time.sleep(1.0)
    if hashlib.sha256(data).hexdigest().startswith(BLOCKED_SHA256_PREFIX):
        return (z, x, y, "BLOCKED", 0)  # OSM served the Access-blocked image
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)
    return (z, x, y, "ok", len(data))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--center-lat", type=float, default=DEFAULT_LAT)
    ap.add_argument("--center-lon", type=float, default=DEFAULT_LON)
    ap.add_argument("--min-zoom", type=int, default=11)
    ap.add_argument("--max-zoom", type=int, default=16)
    ap.add_argument("--radius-km", type=float, default=5.0,
                    help="radius around center to cover at every zoom")
    ap.add_argument("--bbox", type=str, default=None,
                    help="rectangular area 'min_lon,min_lat,max_lon,max_lat' "
                         "(takes precedence over center/radius)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "tiles"))
    ap.add_argument("--threads", type=int, default=3,
                    help="parallel workers (keep low to respect the tile server)")
    ap.add_argument("--url-template", type=str, default=None,
                    help="tile URL template with {z}/{x}/{y} placeholders "
                         "(default: OSM; e.g. a Carto basemap URL)")
    ap.add_argument("--delay", type=float, default=0.5,
                    help="seconds to wait before each tile request (polite throttle)")
    args = ap.parse_args()

    global DELAY
    DELAY = args.delay
    if args.url_template:
        globals()["TILE_TEMPLATE"] = args.url_template

    if args.min_zoom < 0 or args.max_zoom > 19 or args.min_zoom > args.max_zoom:
        sys.exit("bad zoom range")
    os.makedirs(args.out, exist_ok=True)

    bbox = None
    if args.bbox:
        parts = [float(p) for p in args.bbox.split(",")]
        if len(parts) != 4:
            sys.exit("--bbox must be 'min_lon,min_lat,max_lon,max_lat'")
        bbox = (parts[0], parts[1], parts[2], parts[3])

    jobs = []
    for z in range(args.min_zoom, args.max_zoom + 1):
        if bbox:
            min_lon, min_lat, max_lon, max_lat = bbox
            x0, x1 = int(lon2tile(min_lon, z)), int(lon2tile(max_lon, z))
            y0, y1 = int(lat2tile(max_lat, z)), int(lat2tile(min_lat, z))
        else:
            half = max(1, int(math.ceil(args.radius_km / tile_span_km(args.center_lat, z))))
            cx, cy = lon2tile(args.center_lon, z), lat2tile(args.center_lat, z)
            x0, x1 = int(cx) - half, int(cx) + half
            y0, y1 = int(cy) - half, int(cy) + half
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                jobs.append((z, x, y, args.out))

    area_desc = (f"bbox={args.bbox}") if bbox else (f"center {args.center_lat},{args.center_lon} r={args.radius_km}km")
    print(f"downloading {len(jobs)} tiles (z{args.min_zoom}..z{args.max_zoom}, "
          f"{area_desc}) -> {os.path.abspath(args.out)}")
    t0 = time.time()
    stats = {"ok": 0, "skip": 0, "fail": 0}
    total_bytes = 0
    with ThreadPoolExecutor(max_workers=args.threads) as ex:
        futs = [ex.submit(download_one, j) for j in jobs]
        done = 0
        for fut in as_completed(futs):
            z, x, y, status, nbytes = fut.result()
            done += 1
            if status == "ok":
                stats["ok"] += 1
                total_bytes += nbytes
            elif status == "skip":
                stats["skip"] += 1
                total_bytes += nbytes
            else:
                stats["fail"] += 1
                print(f"  {status}  z{z}/{x}/{y}.png")
            if done % 200 == 0:
                print(f"  ... {done}/{len(jobs)} tiles")
            time.sleep(0.02)  # gentle throttle for the tile server

    mb = total_bytes / (1024 * 1024)
    print(f"\ndone in {time.time()-t0:.0f}s: {stats['ok']} fetched, "
          f"{stats['skip']} skipped, {stats['fail']} failed, ~{mb:.1f} MB")
    print(f"copy the contents of '{os.path.abspath(args.out)}' to the SD card "
          f"root (so /sdcard/<z>/<x>/<y>.png).")


if __name__ == "__main__":
    main()
