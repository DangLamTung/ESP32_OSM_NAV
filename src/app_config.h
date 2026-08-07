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
#define ZOOM_MIN       11   // offline card tiles cover z11..z15
#define ZOOM_MAX       16   // z16 = network fallback

// ---- Tile cache (16 slots x 128KB (256px) = 2MB PSRAM) ----
#define TILE_CACHE_SLOTS 16

// ---- On-screen controls geometry (landscape 320x240) ----
#define SCAN_BTN_X (SCREEN_W - 44)
#define SCAN_BTN_Y 0
#define SCAN_BTN_W 44
#define SCAN_BTN_H 22
#define SCAN_TAP_Y_MAX 24          // tap zone for SCAN (tight, below is mode)

#define MODE_BTN_X (SCREEN_W - 44)
#define MODE_BTN_Y 26
#define MODE_BTN_W 44
#define MODE_BTN_H 22

#define ZOOM_BTN_W 42
#define ZOOM_BTN_H 40
#define ZOOM_IN_X  (SCREEN_W - 50)
#define ZOOM_IN_Y  (SCREEN_H - 96)   // "+"
#define ZOOM_OUT_X (SCREEN_W - 50)
#define ZOOM_OUT_Y (SCREEN_H - 52)   // "-"

// ---- Touch / sleep ----
#define TOUCH_SDA 16            // FT6336 I2C data
#define TOUCH_SCL 15            // FT6336 I2C clock
#define TOUCH_INT_GPIO 17       // FT6336 interrupt pin (wake source)
#define LONG_PRESS_MS  2000     // hold finger still to enter sleep

#endif // APP_CONFIG_H_
