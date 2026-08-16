# ESP32_OSM_NAV / scripts — README

Scripts for the ESP32-S3 offline navigation display: building **offline map
tile sets** for the SD card, uploading them, and generating embedded/helper
assets. The big picture (running your own OSM tile server + downloading large
areas) is in [OSM SERVER SETUP](#osm-server-setup--large-area-download) below.

## Contents

| Script | Purpose |
|---|---|
| `download_offline_tiles.py` | Download raster tiles (bbox or center+radius, any zooms, any source URL) into `z/x/y.png` |
| `estimate_area.py` | Print how many tiles a bbox/center area needs per zoom + projected size |
| `convert_png_to_jpg.py` | PNG → JPEG (measured: **JPEG is 2× bigger** for map tiles — don't use it) |
| `check_tiles.py` | Sanity-check embedded JPEG tiles in `src/map_tiles.h` |
| `fetch_osm_demo.py` | Generate a real-OSM demo packet via the Overpass API |
| `fetch_tiles_embed.py` / `gen_embedded_tiles.py` | Embed a few tiles into flash as C arrays |
| `sd_capture.py` | Capture boot log over serial (reset via DTR/RTS) |
| `upload_tiles_serial.py` | Stream `z/x/y.png` tiles PC → `/sdcard` over USB-Serial/JTAG (no card removal) |
| `upload_icon_sd.py` / `verify_icons_sd.py` | Write/verify the icon set to SD |
| `upload_routing_graph.py` | Write `routing.rng` (offline A* graph) to SD |
| `ble_probe` / `ble_probe.swift` | BLE probe tool for the nav service |

Output layout everywhere is the standard slippy-map tree:

```
<out>/<z>/<x>/<y>.png   ->  copy CONTENTS to SD card root
                          ->  app reads /sdcard/<z>/<x>/<y>.png
```

---

## OSM server setup & large-area download

Running your **own** tile server lets you bulk-download huge map areas (e.g.
all of Vietnam z15 ≈ 962k tiles in the bbox) without tripping
`tile.openstreetmap.org`'s bulk-download protection — this network is already
IP-blocked there (it returns a fixed "Access blocked" image). A local server =
no rate limits, `--delay 0`, many threads, same `{z}/{x}/{y}.png` output the
device already reads.

```
 WSL2 Ubuntu (Windows)                              Windows
┌──────────────────────────────────────┐     ┌──────────────────────────┐
│ PostGIS DB (osm-data)                │     │ download_offline_tiles.py│
│    ▲  osm2pgsql import               │     │ --url-template localhost │
│    ▲  vietnam-latest.osm.pbf         │     │   -> z/x/y.png           │
│ overv/openstreetmap-tile-server      │     │        │                 │
│  (renderd + mod_tile + carto style)  │     │        ▼                 │
│  localhost:28080/tile/{z}/{x}/{y}.png│     │  SD card -> ESP32        │
└──────────────────────────────────────┘     └──────────────────────────┘
```

### Server setup (Docker engine in WSL2, no Docker Desktop)

```bash
wsl -d Ubuntu-22.04
sudo apt update && sudo apt -y install docker.io docker-compose-v2
sudo usermod -aG docker $USER && sudo service docker start
```

Get the data + patch the import script (the stock `run.sh` external-data step
stalls forever — skip it, then create empty carto external-data tables so the
renderer doesn't error):

```bash
mkdir -p ~/osm && cd ~/osm
wget -c https://download.geofabrik.de/asia/vietnam-latest.osm.pbf

docker run --rm --entrypoint cat overv/openstreetmap-tile-server /run.sh > run.sh.orig
python3 - <<'PY'
src = open('run.sh.orig').read()
old = "if [ -f /data/style/scripts/get-external-data.py ] && [ -f /data/style/external-data.yml ]; then"
new = "if false; then  # SKIPPED external-data (stalled downloads)"
assert old in src
open('run.sh', 'w').write(src.replace(old, new, 1))
PY
chmod +x run.sh
```

Import with the **Postgres 15** volume path and PBF mounted at
`/data/region.osm.pbf` (the image's README `/12/main` example is outdated):

```bash
docker volume create osm-data && docker volume create osm-tiles
docker run -d --name osm-import \
  -v ~/osm/run.sh:/run.sh \
  -v ~/osm/vietnam-latest.osm.pbf:/data/region.osm.pbf \
  -v osm-data:/var/lib/postgresql/15/main \
  overv/openstreetmap-tile-server import
docker logs -f osm-import          # ~1-3 h CPU for Vietnam
```

Once the import finishes, create the empty external-data tables (see
`osm_idf/scripts/create_external_tables.sql`), then run + smoke-test the server:

```bash
docker exec -it osm-import psql -U renderer -d gis -c \
  "CREATE TABLE IF NOT EXISTS water_polygons (way geometry(Geometry,3857));"

docker rm -f osm-tile-server 2>/dev/null || true
docker run -d --name osm-tile-server \
  -p 28080:80 \
  -v osm-data:/var/lib/postgresql/15/main \
  -v osm-tiles:/var/lib/mod_tile \
  overv/openstreetmap-tile-server run

curl -s -o /tmp/t.png "http://localhost:28080/tile/15/26096/15400.png" && file /tmp/t.png
# Bến Thành HCMC z15 -> PNG 256x256 (HTTP 200)
```

### Download a large area

**1. Plan first** (how many tiles / how big):

```powershell
python estimate_area.py --bbox 102.14,8.19,109.46,23.39 --min-zoom 15 --max-zoom 15
```

Vietnam z15 bbox = **962,481 tiles** to iterate; only ~35% is land, so expect
~340k real PNGs ≈ **3–4 GB** (ocean positions 404/blank and are skipped).

**2. Validate a small area first** (HCMC z11–15 ≈ 5k tiles):

```powershell
python download_offline_tiles.py `
  --bbox 106.36,10.35,106.92,11.12 --min-zoom 11 --max-zoom 15 `
  --threads 8 --delay 0 `
  --url-template "http://localhost:28080/tile/{z}/{x}/{y}.png" `
  --out ..\tiles_hcmc
```

**3. Then the whole country**:

```powershell
python download_offline_tiles.py `
  --bbox 102.14,8.19,109.46,23.39 --min-zoom 15 --max-zoom 15 `
  --threads 16 --delay 0 `
  --url-template "http://localhost:28080/tile/{z}/{x}/{y}.png" `
  --out ..\tiles_vn
```

Both `download_offline_tiles.py` and `download_vn_z15.py` (see
`osm_idf/scripts/`) are **resumable**: valid PNGs are skipped on re-run, and the
OSM "Access blocked" placeholder (sha256 prefix `b02c44252dac5a5e`) is detected
and re-fetched.

### Copy to SD

Copy the **contents** of the output folder to the exFAT card root so the app
reads `/sdcard/<z>/<x>/<y>.png` — or stream without removing the card:

```powershell
python upload_tiles_serial.py --port COM9 --dir ..\tiles_vn
```

### Politeness & attribution

- Always send a descriptive User-Agent (the scripts do).
- Against public servers keep `--threads` low and `--delay ≥ 0.5 s`; Carto's
  bulk path backs off on HTTP 429.
- **Do not bulk-scrape `tile.openstreetmap.org`** — use your local server or
  Carto (`https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png`).
- Carto requires attribution: **© OpenStreetMap contributors © CARTO**.
- OSM data © OpenStreetMap contributors (ODbL).

### Troubleshooting (verified gotchas)

| Symptom | Fix |
|---|---|
| Import never finishes | `run.sh` external-data step stalls → patch it (above), wipe `osm-data`, re-import |
| DB data lost after container restart | Mount volume at `/var/lib/postgresql/15/main` (PG15), not `/12/main` |
| Renderer errors on `water_polygons` / `icesheet_*` | Create the empty tables (see `osm_idf/scripts/create_external_tables.sql`) |
| Tile blank/404 at a position | Ocean / outside the extract — normal, downloader skips |
| Export slow | Pre-render the bbox first (visit a few tiles), then raise `--threads` |
| Docker not running in WSL | `sudo service docker start` |
| Live device tiles "Access blocked" | Network IP is blocked at OSM → use the local server or Carto as the live provider |

### More detail

The full how-to (with native non-Docker option B, disk/RAM planning, and ESP32
wiring) lives in the osm_idf project: `osm_idf/OSM_SERVER_SETUP.md` and
`osm_idf/PLAN_LOCAL_TILE_SERVER.md`.
