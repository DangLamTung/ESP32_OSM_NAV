/**
 * app_config.h — build-time configuration for the OSM tile viewer.
 * Board: 2.8" IPS ESP32-S3 + ILI9341 (ES3C28P / ES3N28P)
 */
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

// ---- Network ----
// WiFi credentials live in secrets.h (GIT-IGNORED - never commit real creds).
// If secrets.h is missing, these placeholders keep the build working offline.
#include "secrets.h"
#ifndef WIFI_SSID
#define WIFI_SSID "your-wifi-ssid"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "your-wifi-password"
#endif

// ---- Display (ILI9341, 320x240 landscape) ----
#define SCREEN_W 320
#define SCREEN_H 240

// ---- Map view (Bến Thành, HCMC) ----
#define MAP_CENTER_LAT 10.7718
#define MAP_CENTER_LON 106.6982
#define ZOOM_DEFAULT   15
#define ZOOM_MIN       5    // zoom out down to z5 (overview; SD tiles are z11+)
#define ZOOM_MAX       16   // z16 = network fallback

// ---- Map rotation (heading-up / manual) ----
// The tile fetcher composes a SQUARE "world" sprite (MAP_WORLD_SIZE) centered
// on the view; the visible 320x240 view is a rotated crop of it. 448 is the
// smallest power-of-64 square whose half-diagonal (317px) clears the 320x240
// window corner (200px) at ANY rotation, so no blank corners appear. Costs
// 448x448x2 = ~400KB PSRAM (board has 8MB octal PSRAM).
#define MAP_WORLD_SIZE 448
#define ROTATE_STEP_DEG 15      // manual rotate button step (degrees)

// ---- Smooth (eased) rotation ----
// The map current rotation glides toward a target each redraw instead of
// snapping (Google-Maps-style). ROTATION_EASE_FACTOR is the fraction of the
// remaining angle closed per redraw (~0.25 -> a 90° turn settles in ~0.6s).
#define ROTATION_EASE_FACTOR 0.28f   // snappy-but-smooth turn swing (not laggy)
#define ROTATION_SETTLE_DEG  0.5f   // stop easing within this many degrees

// ---- Smooth follow (position interpolation) ----
// map_follow_interp() tracks the last two GPS fixes and advances the view
// center along the path between them (interpolating, then extrapolating past
// the current fix) so the map scrolls CONTINUOUSLY with no "run then stop"
// settle. The loop keeps redrawing while the car is moving.

// ---- Low-pass jitter filter ----
// Small BLE/render jitter is smoothed with an exponential moving average on the
// view position and the GPS heading. Higher alpha = smoother but slightly
// laggier. 0..1.
#define POS_LP_ALPHA    0.45f  /* view-centre position EMA per frame (tight track) */
#define HEADING_LP_ALPHA 0.45f  /* GPS heading EMA (before rotation target) */
#define HEADING_DEADBAND_DEG 3.0f /* ignore heading wobble < 3 deg (stops map micro-rotation jitter) */
#define CAMERA_DEADBAND_PX 1.0f  /* smallest meaningful step (1 map px) — smoothest demo scroll */

// ---- Map rotation quality ----
// Boot benchmark measured (z15, PSRAM world, 80MHz SPI):
//   north-up blit ~1ms, rotated NN ~11ms, rotated AA ~125ms (!!), push ~19ms,
//   tile compose (fetch) ~28ms. AA rotation is 11x slower than NN -> 6fps, so
//   it is NOT usable for continuous driving. Default = 0 (fast NN); the ROT
//   button in settings can switch to CRISP (AA) for a static/crisp view.
#define MAP_AA_ROTATION 0

// ---- UI mode ----
// 0 = full HUD: maneuver banner + weather widget + bottom bar (speed-limit &
//     speedometer icons, clock + ETA).
// 1 = simple mode: only the needed text — maneuver banner + one consistent row
//     of speed / time / ETA. Lighter on CPU, less clutter.
#define UI_SIMPLE_MODE 0

// ---- Tile cache (24 slots x 128KB (256px) = 3MB PSRAM) ----
// 24 keeps the display's tiles + the preload-ahead tiles cached at once, so
// the background preload never evicts a tile the display needs -> no stutter.
#define TILE_CACHE_SLOTS 24

// ---- Rotation control buttons (left edge, clear of the banner/weather box) ----
#define ROTATE_BTN_X 8
#define ROTATE_BTN_Y (SCREEN_H - 188)      /* 52 — below the 44px banner */
#define ROTATE_BTN_W 44
#define ROTATE_BTN_H 22
#define HDG_BTN_X   8
#define HDG_BTN_Y   (ROTATE_BTN_Y + ROTATE_BTN_H + 4)   /* 78 */
#define HDG_BTN_W   44
#define HDG_BTN_H   22
#define CENTER_BTN_X 8
#define CENTER_BTN_Y (HDG_BTN_Y + HDG_BTN_H + 4)   /* 104 — recenter on the car */
#define CENTER_BTN_W 44
#define CENTER_BTN_H 22

// ---- On-screen controls geometry (landscape 320x240) ----
#define GEAR_BTN_X (SCREEN_W - 50)   /* align with the zoom column */
#define GEAR_BTN_Y (SCREEN_H - 122)  /* just above the "+" zoom button */
#define GEAR_BTN_W 42
#define GEAR_BTN_H 22

#define ZOOM_BTN_W 42
#define ZOOM_BTN_H 40
#define ZOOM_IN_X  (SCREEN_W - 50)
#define ZOOM_IN_Y  (SCREEN_H - 96)   // "+"
#define ZOOM_OUT_X (SCREEN_W - 50)
#define ZOOM_OUT_Y (SCREEN_H - 52)   // "-"

// ---- Settings panel (bottom overlay; gear opens it) ----
#define SETTINGS_PANEL_Y  (SCREEN_H - 96)   /* 144 — two-button rows */
#define SETTINGS_PANEL_H  96
#define WIFI_BTN_X 10
#define WIFI_BTN_Y (SETTINGS_PANEL_Y + 8)
#define WIFI_BTN_W 92
#define WIFI_BTN_H 22
#define GPS_BTN_X  (WIFI_BTN_X + WIFI_BTN_W + 8)   /* 110 — right of WiFi */
#define GPS_BTN_Y  (SETTINGS_PANEL_Y + 8)
#define GPS_BTN_W  92
#define GPS_BTN_H  22
#define MODE_BTN_X (GPS_BTN_X + GPS_BTN_W + 8)   /* 210 — right of GPS */
#define MODE_BTN_Y (SETTINGS_PANEL_Y + 8)
#define MODE_BTN_W 100
#define MODE_BTN_H 22
#define AA_BTN_X   10                          /* row 2: rotation quality */
#define AA_BTN_Y   (SETTINGS_PANEL_Y + 40)     /* 184 */
#define AA_BTN_W   150
#define AA_BTN_H   22
/* offline-routing toggle (settings panel, row 2 right of ROT) */
#define OR_BTN_X  (AA_BTN_X + AA_BTN_W + 8)    /* 168 */
#define OR_BTN_Y  (SETTINGS_PANEL_Y + 40)      /* 184 */
#define OR_BTN_W  (SCREEN_W - OR_BTN_X - 10)   /* 142 */
#define OR_BTN_H  22
#define SLIDER_X   10
#define SLIDER_Y   (SETTINGS_PANEL_Y + 72)     /* 216 */
#define SLIDER_W   (SCREEN_W - 20)
#define SLIDER_H   14
#define BRIGHTNESS_DEFAULT 128   /* default backlight = half (user request) */

// ---- U-Blox GPS (NMEA over UART, broadcast over BLE when a phone connects) ----
#define GPS_UART_NUM 2        // UART2 (pins free on this board: 1-9, 14, 19-21...)
#define GPS_TX_PIN   4        // UART TX -> GPS RX
#define GPS_RX_PIN   5        // UART RX <- GPS TX
#define GPS_BAUD     9600     // U-Blox default
#define GPS_BROADCAST_DEFAULT 0   // off until toggled in the settings panel

// ---- Touch / sleep ----
#define TOUCH_SDA 16            // FT6336 I2C data
#define TOUCH_SCL 15            // FT6336 I2C clock
#define TOUCH_INT_GPIO 17       // FT6336 interrupt pin (wake source)
#define LONG_PRESS_MS  5000     // hold finger still to enter sleep

#endif // APP_CONFIG_H_
