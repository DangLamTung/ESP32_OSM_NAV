#!/usr/bin/env python3
"""
convert_png_to_jpg.py — convert a downloaded PNG tile set (z/x/y.png) to JPEG
(z/x/y.jpg) so it fits more on the SD card. The osm_idf app decodes JPEG tiles
with the ESP32-S3 hardware JPEG decoder (it tries /sdcard/z/x/y.png first, then
.jpg).

Usage:
    python convert_png_to_jpg.py --dir tiles_hcmc --quality 80 --threads 8
"""
import argparse
import glob
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

from PIL import Image

Image.MAX_IMAGE_PIXELS = None


def convert_one(job):
    png, jpg, quality = job
    try:
        with Image.open(png) as im:
            im = im.convert("RGB")
            im.save(jpg, "JPEG", quality=quality, optimize=True)
        png_size = os.path.getsize(png)
        jpg_size = os.path.getsize(jpg)
        return (png, jpg, True, png_size, jpg_size)
    except Exception as e:
        return (png, jpg, False, 0, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(__file__), "..", "tiles_hcmc"))
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--keep-png", action="store_true",
                    help="keep the original PNG files (default: delete after a "
                         "successful JPG is written)")
    ap.add_argument("--threads", type=int, default=8)
    args = ap.parse_args()

    pngs = sorted(glob.glob(os.path.join(args.dir, "**", "*.png"), recursive=True))
    if not pngs:
        sys.exit(f"no *.png found under {args.dir}")
    print(f"{len(pngs)} PNG tiles to convert -> {os.path.abspath(args.dir)}")

    jobs = []
    for png in pngs:
        jpg = os.path.splitext(png)[0] + ".jpg"
        jobs.append((png, jpg, args.quality))

    ok = 0
    total_png = 0
    total_jpg = 0
    with ThreadPoolExecutor(max_workers=args.threads) as ex:
        futs = [ex.submit(convert_one, j) for j in jobs]
        for i, fut in enumerate(as_completed(futs), 1):
            png, jpg, success, png_size, jpg_size = fut.result()
            if success:
                ok += 1
                total_png += png_size
                total_jpg += jpg_size
                if not args.keep_png:
                    os.remove(png)
            else:
                print(f"  FAIL {png}")
            if i % 1000 == 0:
                print(f"  ... {i}/{len(pngs)}")

    print(f"\nconverted {ok}/{len(pngs)}")
    if ok:
        print(f"  PNG : {total_png/1048576:.1f} MB")
        print(f"  JPG : {total_jpg/1048576:.1f} MB  (q{args.quality})")
        print(f"  saved {100*(1 - total_jpg/total_png):.0f}%  ({'kept PNGs' if args.keep_png else 'removed PNGs'})")


if __name__ == "__main__":
    main()
