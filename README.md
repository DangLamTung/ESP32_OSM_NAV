# ESP32-S3 Car Navigation Display

ESP32 fetches Google Maps tiles + directions via WiFi, renders on ILI9341 via DMA.

## Hardware

2.8" IPS ESP32-S3 board (ES3C28P / ES3N28P, ILI9341V controller):
- ESP32-S3, 16 MB flash
- ILI9341 320×240 landscape over SPI2 (MOSI=11, MISO=13, CLK=12, CS=10, DC=46, RST=18, BL=45)
- FT6X06 capacitive touch on I2C (SDA=16, SCL=15) — not used yet
- Native USB-Serial/JTAG (Type-C)

## Architecture

```
ESP32-S3 (in car)              Google APIs                   Phone (optional)
───────────────                ───────────                   ────────────────
Sleep 30s                      │                            GPS → BLE send
Wake → WiFi ON                 │
┌─HTTPS GET────────────────→  Static Maps API
│  /staticmap?center=...       │
│  &zoom=15&size=320x240       │
│  &markers=...&path=...       │
│←── JPEG 8-15KB               │
│                              │
│  GET──────────────────────→  Directions API
│  /directions/json?origin=... │
│  &destination=...&mode=drive │
│←── JSON route polyline       │
│                              │
├─Decode JPEG (ROM tjpgd)      │
├─Draw route overlay            │
├─DMA→ILI9341 320×240          │
└─WiFi OFF, Sleep 30s          │
5µA                            │
```

## Power Budget — 2×AA 2500mAh

| Mode | Current | Per day | Daily |
|------|---------|---------|-------|
| Deep sleep | 5 µA | 23.6h | 0.12 mAh |
| WiFi + HTTPS | 120 mA | 24s | 0.8 mAh |
| JPEG decode + DMA | 50 mA | 0.3s | 0.4 mAh |
| ILI9341 display | 30 mA | 30 min | 15 mAh |
| **Total** | | | **~16 mAh/day → 150 days** |

## APIs (HTTPS)

**Static Map image:**
```
GET https://maps.googleapis.com/maps/api/staticmap
  ?center=10.762,106.660
  &zoom=15
  &size=320x240
  &markers=color:red|10.762,106.660
  &path=color:blue|weight:3|10.762,106.660|10.763,106.661|...
  &key=API_KEY
→ 320×240 JPEG (~8-15KB)
```

**Directions route:**
```
GET https://maps.googleapis.com/maps/api/directions/json
  ?origin=10.762,106.660
  &destination=10.772,106.670
  &mode=driving
  &key=API_KEY
→ JSON → "overview_polyline": "w`jwF~kpbVu@b@..."
```

## What We Already Have (Working)

From `src/`:
- ✅ ILI9341 320×240 @ 40MHz SPI DMA
- ✅ WiFi HTTPS with TLS cert bundle
- ✅ JPEG decoder (ROM tjpgd, per-line DMA)
- ✅ Map fetcher (download + render)
- ✅ Per-line DMA GIF player pattern (no canvas)

## What To Add

1. Directions API → decode polyline → draw on map
2. GPS input (BLE from phone, or UART GPS module)
3. Power cycle: sleep → WiFi → fetch → display → sleep
4. BLE command interface for phone control
