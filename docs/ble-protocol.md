# NAV-OSM BLE Protocol

Binary, little-endian, frame-based navigation protocol between a phone/nav app
and the ESP32-S3 car display (`ESP32_OSM_NAV` firmware). A Web Bluetooth
simulator lives in [`web_ble_nav/index.html`](../web_ble_nav/index.html) and is
the reference implementation for senders.

- **Transport:** BLE GATT characteristic write (with response).
- **Two encodings:** compact binary frames (preferred) and an XML fallback.
  A frame starting with the magic `0xAA 0x55` is binary; otherwise the payload
  is parsed as XML (`<route…>`, `<nav…>`, …).
- **Encoding:** all multi-byte integers are **little-endian**; strings are
  **UTF-8**.
- **Fixed-point:** coordinates use `×1e7` (deg → i32) for absolute lat/lon and
  `×1e5` (deg → i16) for route point deltas.

---

## 1. GATT

| UUID | Role | Props | Purpose |
|---|---|---|---|
| Service `5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c` | — | — | Nav service |
| Char `5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c` | in / out | READ, WRITE, WRITE_NR, NOTIFY | Nav packets in; raw GPS NMEA out |
| Service `5a7e2000-2b2f-4f66-9f9a-5c0f8e1a2b3c` | — | — | Weather service |
| Char `5a7e2001-2b2f-4f66-9f9a-5c0f8e1a2b3c` | in | WRITE, WRITE_NR | Weather packet in |

- Advertised name: **`NAV-OSM`**
- GPS broadcast (out, on the nav char NOTIFY): raw NMEA lines at **1 Hz** when
  enabled in the settings panel (off by default).
- The web app chunks binary frames into **≤ 500-byte** writes so each stays
  under the ~512-byte BLE MTU, and serializes writes (write-with-response needs
  an ACK, so concurrent writes must be queued).

---

## 2. Binary frame format

```
byte 0     0xAA        magic
byte 1     0x55        magic
byte 2     type        0x01 .. 0x09
byte 3     len_lo      payload length, low byte
byte 4     len_hi      payload length, high byte
byte 5..   payload     type-specific (below)
```

`len = len_lo | (len_hi << 8)` (little-endian, max 65535).

### Packet type overview

| Type | Name | Payload |
|---|---|---|
| `0x01` | ROUTE | zoom, count, first point, then deltas |
| `0x02` | POS | position, speed, heading, speed limit |
| `0x03` | NAV | next maneuver |
| `0x04` | ETA | arrival time + place |
| `0x05` | CLOCK | current time |
| `0x06` | ROUTE_CONT | route continuation (same layout as `0x01`) |
| `0x07` | WEATHER | temp, humidity, code, label (weather service) |
| `0x08` | NAV2 | maneuver *after* the next one (2-step HUD) |
| `0x09` | CAMERA | speed camera ahead |

---

## 3. Payload layouts

### `0x01` ROUTE (and `0x06` ROUTE_CONT) — route polyline

```
zoom  u8            map zoom (usually 15)
count u16           number of points, 2..256
lat0  i32 ×1e7      first point latitude
lon0  i32 ×1e7      first point longitude
(count-1) × [
  dlat i16 ×1e5     delta latitude  (points[i].lat − points[i−1].lat) × 1e7 / 100
  dlon i16 ×1e5     delta longitude
]
```

- Deltas are relative to the *previous absolute* point, so a 256-point route
  is `3 + 8 + 255×4 = 1031` payload bytes.
- `ROUTE_CONT` (`0x06`) draws a faint grey continuation beyond the near
  path-ahead; the bright route (`0x01`) draws over it.
- The firmware holds at most **256 points** (`NAV_MAX_ROUTE_POINTS`). Long
  roads are streamed as a sliding window: the sender keeps ~350 m of road
  behind the car and advances the window as the car moves.

### `0x02` POS — position

```
lat  i32 ×1e7   latitude
lon  i32 ×1e7   longitude
spd  u8         speed, km/h
hdg  u16        heading, degrees 0..359
sl   u8         speed limit, km/h (0 = unknown)
```

### `0x03` NAV — next maneuver (payload shared by `0x08` NAV2)

```
dist   u16       meters to the maneuver
modId  u8        maneuver id (see table below)
slen   u8        street name byte length
street slen×u8   street name, UTF-8
```

**Maneuver IDs**

| id | maneuver |
|---|---|
| 0 | straight |
| 1 | left |
| 2 | right |
| 3 | slight-left |
| 4 | slight-right |
| 5 | u-turn |
| 6 | roundabout |
| 7 | arrive |

### `0x04` ETA — arrival

```
h     u8       arrival hour 0..23
m     u8       arrival minute 0..59
alen  u8       "arrive" label length
arrive alen×u8  destination label, UTF-8
```

### `0x05` CLOCK — current time

```
h  u8   hour 0..23
m  u8   minute 0..59
```

### `0x07` WEATHER — weather widget (sent on the *weather* service char)

```
tempC    s8       temperature °C
humidity u8       relative humidity %
code     u8       WMO weather code (0 = clear, 3 = overcast, 45 = fog, 61 = rain, 95 = storm …)
slen     u8       label length
text     slen×u8  short label, UTF-8 (e.g. "Nắng nóng")
```

### `0x09` CAMERA — speed camera alert

```
dist  u16   meters to the camera ahead
type  u8    0 = fixed, 1 = mobile
```

- `dist = 0` **clears** the alert (camera passed / no camera).
- The board shows an amber `CAMERA nnn m` (or `MOBILE CAM`) badge in the HUD.

---

## 4. XML fallback

Sent on the nav char without the `0xAA 0x55` magic; the firmware finalizes the
packet on the closing tag (or a `0x00` terminator).

```
<route z="15"><p lat="10.771859" lon="106.698163"/><p lat="…" lon="…"/>…</route>
<nav  d="85"  m="left" s="Nguyen Hue"/>
<pos  lat="10.77" lon="106.69" spd="40" hdg="120" sl="60"/>
<eta  h="12"  m="30" a="Ben Thanh"/>
<clock h="12" m="5"/>
```

- `ROUTE_CONT`, `WEATHER`, and `CAMERA` are **binary-only** (no XML form).
- The firmware detects the kind by searching for `<route`, `<nav`, `<pos`,
  `<eta`, `<clock`.

---

## 5. Notes / limits

- **Street/label length:** the firmware caps strings at `NAV_MAX_STREET = 64`
  bytes; keep UTF-8 names (or shortened versions) under that.
- **Route points:** max 256. Batch larger routes into sliding windows.
- **Binary preferred:** it is 5×–20× smaller than XML and avoids any
  attribute-parsing ambiguity.
- **Reference sender:** `web_ble_nav/index.html` — `encRoute`, `encPos`,
  `encNav`/`encNav2`, `encEta`, `encClock`, `encWeather`, `encCamera` build the
  frames; `binFrame` wraps them with the magic + length.
- **Stability guidance:** keep the sustained write rate modest (the sim runs
  ~10 writes/s for position + occasional nav/clock/weather/camera). The board
  interpolates between fixes, so position can be sent at ~10 Hz.
