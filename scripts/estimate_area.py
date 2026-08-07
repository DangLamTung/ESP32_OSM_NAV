#!/usr/bin/env python3
"""estimate_area.py — print how many tiles a bbox/center area needs per zoom
and project the download size (grounded on the average tile size of an
existing tile set if present).

Usage:
    python estimate_area.py --bbox 106.36,10.35,106.92,11.12 --min-zoom 11 --max-zoom 16
"""
import argparse
import glob
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def lon2tile(lon, z):
    return (lon + 180.0) / 360.0 * (1 << z)


def lat2tile(lat, z):
    r = math.radians(lat)
    return (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * (1 << z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bbox", default=None,
                    help="'min_lon,min_lat,max_lon,max_lat'")
    ap.add_argument("--center", default=None, help="'lat,lon' (with --radius-km)")
    ap.add_argument("--radius-km", type=float, default=5.0)
    ap.add_argument("--min-zoom", type=int, default=11)
    ap.add_argument("--max-zoom", type=int, default=16)
    ap.add_argument("--tiles-dir", default=os.path.join(HERE, "..", "tiles"))
    ap.add_argument("--kbs", type=float, nargs="*", default=[7, 10, 15, 25],
                    help="avg tile KB to project sizes for")
    args = ap.parse_args()

    bbox = None
    center = None
    if args.bbox:
        bbox = [float(p) for p in args.bbox.split(",")]
    elif args.center:
        lat, lon = [float(p) for p in args.center.split(",")]
        center = (lat, lon)

    rows = []
    total = 0
    for z in range(args.min_zoom, args.max_zoom + 1):
        if bbox:
            min_lon, min_lat, max_lon, max_lat = bbox
            x0, x1 = int(lon2tile(min_lon, z)), int(lon2tile(max_lon, z))
            y0, y1 = int(lat2tile(max_lat, z)), int(lat2tile(min_lat, z))
        else:
            lat, lon = center
            span = 40075.0 * math.cos(math.radians(lat)) / (1 << z)
            half = max(1, int(math.ceil(args.radius_km / span)))
            cx, cy = lon2tile(lon, z), lat2tile(lat, z)
            x0, x1, y0, y1 = int(cx) - half, int(cx) + half, int(cy) - half, int(cy) + half
        n = (x1 - x0 + 1) * (y1 - y0 + 1)
        total += n
        rows.append(f"z{z:2d}: x {x0:6d}-{x1:<6d} y {y0:6d}-{y1:<6d}  {n:8,d} tiles")

    rows.append(f"\nTOTAL: {total:,d} tiles")
    for kb in args.kbs:
        rows.append(f"  at avg {kb:>3.0f} KB/tile -> {total * kb / 1024.0:7.1f} MB")

    # measure an existing tile set if present
    fs = glob.glob(os.path.join(args.tiles_dir, "**", "*.png"), recursive=True)
    if fs:
        avg = sum(os.path.getsize(f) for f in fs) / len(fs)
        rows.append(f"\nmeasured avg tile size in {args.tiles_dir}: {avg/1024:.1f} KB "
                    f"({len(fs):,} files)")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
