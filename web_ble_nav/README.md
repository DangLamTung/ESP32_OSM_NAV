# web_ble_nav — Web Bluetooth simulator for the ESP32-S3 nav board

Phone-side **drive simulator** for the OSM Nav firmware. It connects to the
board over Web Bluetooth, sends it a real OSM route (Bến Thành → Vũng Tàu via
the public OSRM API, or a built-in sample), then drives the car along the route
and streams the same packets the real phone app would send: route windows,
position, next-turn nav (next + next-next), clock, ETA, weather and speed-camera
alerts.

```
browser (this app)  --BLE-->  ESP32-S3 board (car_nav.bin, offline SD tiles)
  "simulates the phone driving"
```

---

## What you need

- **Board**: ESP32-S3 flashed with `build/car_nav.bin`, SD card with offline
  map tiles (and optionally `routing.rng` for offline routing).
- **Browser**: Chrome or Edge (Web Bluetooth requirement). Firefox/Safari won't
  work.
- **Secure context**: Web Bluetooth only works over **https** or **localhost**,
  so serve the page with a local HTTP server (below) — do not open `index.html`
  directly from disk.

---

## 1. Run the web simulator

```bash
cd /Users/tungdl/Documents/Eink/ESP32_OSM_NAV/web_ble_nav
python3 -m http.server 8000
```

Open **http://localhost:8000/** in Chrome/Edge, then:

1. **🔗 Connect to NAV-OSM** — pick your board in the Bluetooth chooser.
   The header dot turns green and the board replies with its hello string.
2. **🚀 Start navigation — send all + drive** (top green button). This:
   - auto-connects if needed,
   - loads the route (falls back to the sample if OSRM is unreachable),
   - sends the first 64-point route window, position at the route start,
     first next-turn nav, clock, ETA and weather,
   - starts driving the full route at the set speed — the map scrolls on the
     board as the car moves.

### Controls

| Button / field | What it does |
|---|---|
| **1 · Route** — *Load sample route* | Fetches a real OSRM route Bến Thành → Vũng Tàu into the XML box (falls back to a 1.5 km sample path). |
| **1 · Route** — *Send route* | Sends just the first 64-point route window to the board. |
| **2 · Drive simulation** — Speed / ×time-lapse / *Start simulate* / *Stop* | Drive along the route polyline at ~10 pos/s; long roads streamed in 64-point batches as you advance. Lower the ×multiplier if it looks too fast. |
| **3 · Maneuver + position** | Manually send a single `nav`, `nav2`, or `pos` packet. |
| **4 · Time + ETA** | Manual `clock` / `eta` packets (top HUD). |
| **5 · Weather** | Manual `weather` packet (top-right widget). |
| **6 · Speed camera** | Manual `camera` alert; the sim also auto-announces one ~every 3 km during a drive. |
| **Log** | Shows every packet sent (binary frames shown as `[bin name]`). |

> Note: the sim **never** subscribes to notifications — the board only receives
> writes, and a failed CCCD subscribe can destabilise the link in some clients.

---

## 2. Headless checks (no board, no browser)

Quick sanity checks that don't need hardware:

```bash
# (1) Sim math + page-JS validity — needs Node
node test_sim.js
# expects: "PAGE JS: OK ..." then the rectangle-route assertion lines

# (2) XML packet parser check — needs Python
python3 verify_packets.py
# expects: all lines "OK" and "ALL OK"
```

`test_sim.js` stubs the DOM/BLE so it can eval the page script and test the
path-following/interpolation/bearing math against a synthetic rectangle route
(verifies 0/90/180/270 headings).

---

## 3. Regenerating the sample route data

The built-in `BT_VT_REAL` array (embedded in `index.html`) is decimated from a
cached OSRM response:

```bash
# uses osrm_bt_vt.json (a saved OSRM Bến Thành → Vũng Tàu response)
python3 make_real_route.py
# prints "const BT_VT_REAL = [...]" — paste it into index.html if you want to
# refresh the embedded fallback route.
```

Note: the app normally fetches a **live** route from
`router.project-osrm.org`, so the embedded array is only used when the fetch
fails or the API is unreachable.

---

## Troubleshooting

- **"Connect failed: GATT operation failed"** — Web Bluetooth in VS Code /
  Electron is limited; use a normal Chrome/Edge window.
- **No device appears in the chooser** — confirm the board is powered, running
  `car_nav.bin`, and advertising service `5a7e1000` (badge shows `NAV-OSM`).
- **Route box empty / "still no route"** — no network to OSRM; click
  *Load sample route* anyway (it falls back to the embedded sample) or paste
  your own `<route z="15"><p lat=".." lon=".."/>…</route>` XML.
- **Map not scrolling** — the board needs SD tiles for the area; the sample
  path stays inside the ~5 km Bến Thành tile radius on purpose.
