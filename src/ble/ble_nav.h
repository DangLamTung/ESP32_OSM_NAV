/**
 * ble_nav.h — BLE navigation receiver for the OSM tile viewer.
 *
 * Implements the BLE GATT server exactly like the manufacturer's demo
 * (Example_26_BLE_server: BLEDevice / BLEServer / BLECharacteristic /
 * BLE2902 / notify), but keeps the navbridge-compatible service/char UUIDs
 * from car_nav/src/ble_server.c so the existing phone app connects unchanged.
 *
 * Protocol (binary frames preferred, XML accepted as fallback):
 *   frame: [0xAA 0x55] type len_lo len_hi payload    (little-endian)
 *   0x01 route: zoom(u8) count(u16) lat0/lon0(i32 x1e7) + (count-1) [dlat/dlon i16 x1e5]
 *   0x02 pos  : lat(i32 x1e7) lon(i32 x1e7) spd(u8) hdg(u16) sl(u8)
 *   0x03 nav  : dist(u16) modId(u8) slen(u8) street (UTF-8)
 *   0x04 eta  : h(u8) m(u8) alen(u8) arrive (UTF-8)
 *   0x05 clock: h(u8) m(u8)
 *   0x08 nav2 : same payload as 0x03 — the maneuver AFTER the next one (for the 2-step HUD)
 *   XML fallback: <route..> <nav..> <pos..> <eta..> <clock..> finalized on closing tag / 0x00
 */
#ifndef BLE_NAV_H
#define BLE_NAV_H

#include <Arduino.h>

/* 256 points so a longer road is visible ahead. NavRoute is now parsed into a
 * STATIC buffer in ble_nav.cpp (not a local on the BLE host task stack), so
 * the old 64-pt cap — needed because a 512-pt local NavRoute (~8KB) overflowed
 * CONFIG_BT_BTC_TASK_STACK_SIZE and corrupted the BLE controller — no longer
 * applies. The web side streams 256-pt windows (web MAX_ROUTE_PTS = 256). */
#define NAV_MAX_ROUTE_POINTS 256
#define NAV_MAX_STREET       64

typedef struct {
  double lat, lon;
} NavPoint;

typedef struct {
  int count;
  NavPoint pts[NAV_MAX_ROUTE_POINTS];
  int zoom;                     // route zoom (default 15)
} NavRoute;

typedef struct {
  bool valid;
  char maneuver[16];            // left | right | slight-left | slight-right | straight | u-turn | roundabout | arrive
  int  dist;                    // meters to next turn
  char street[NAV_MAX_STREET];
} NavManeuver;

typedef struct {
  bool valid;
  double lat, lon;
  int spd;                      // km/h
  int hdg;                      // heading, degrees 0..359
  int limit;                    // speed limit km/h, 0 = unknown
} NavPos;

typedef struct {
  bool valid;
  int hour, minute;             // ETA, 24h
  char arrive[NAV_MAX_STREET];  // arrive street/address (ASCII)
} NavEta;

typedef struct {
  bool valid;
  int hour, minute;             // current time, 24h
} NavClock;

typedef struct {
  bool valid;
  int  tempC;                   // temperature, °C (rounded)
  int  humidity;                // relative humidity %
  int  code;                    // WMO weather code (0 = clear, 3 = overcast, etc.)
  char text[NAV_MAX_STREET];    // short label, e.g. "Sunny" / "Nắng"
} NavWeather;

void bleNavBegin(void);

bool navConnected(void);

bool navRouteDirty(void);
void navRouteClearDirty(void);
bool navContDirty(void);
void navContClearDirty(void);
bool navManeuverDirty(void);
void navManeuverClearDirty(void);
bool navMan2Dirty(void);        /* next-next maneuver (0x08) arrived */
void navMan2ClearDirty(void);
bool navPosDirty(void);
void navPosClearDirty(void);
bool navEtaDirty(void);
void navEtaClearDirty(void);
bool navClockDirty(void);
void navClockClearDirty(void);
bool navWeatherDirty(void);
void navWeatherClearDirty(void);

const NavRoute*    navGetRoute(void);
const NavRoute*    navGetRouteCont(void);   // route continuation (beyond path-ahead)
const NavManeuver* navGetManeuver(void);
const NavManeuver* navGetManeuver2(void);  /* the maneuver after the next one */
const NavPos*      navGetPos(void);
const NavEta*      navGetEta(void);
const NavClock*    navGetClock(void);
const NavWeather*  navGetWeather(void);

/* --- weather service (phone -> ESP) --- */
#define WEATHER_SERVICE_UUID "5a7e2000-2b2f-4f66-9f9a-5c0f8e1a2b3c"
#define WEATHER_CHAR_UUID    "5a7e2001-2b2f-4f66-9f9a-5c0f8e1a2b3c"

/* --- GPS broadcast (ESP -> phone, raw NMEA on the nav char's NOTIFY) --- */
void navSetGpsBroadcast(bool on);   // from the settings button
bool navGpsBroadcast(void);

#endif // BLE_NAV_H
