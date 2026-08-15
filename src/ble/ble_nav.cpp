/**
 * ble_nav.cpp — BLE GATT server + compact XML nav parser.
 *
 * GATT layout (matches car_nav/src/ble_server.c so the navbridge app connects):
 *   Service 5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c
 *   Char    5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c  (READ | WRITE | WRITE_NR | NOTIFY)
 *
 * Code structure follows the manufacturer's Arduino demo
 * (Example_26_BLE_server/Blue_Server_test.ino): BLEDevice::init -> createServer
 * -> setCallbacks -> createService -> createCharacteristic -> BLE2902 -> start
 * -> advertise.
 */
#include "ble_nav.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gps_ublox.h"

static const char *TAG = "nav";

#define SERVICE_UUID "5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c"
#define CHAR_UUID    "5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c"
#define ADV_NAME     "NAV-OSM"

/* ---- decoded state ---- */
static NavRoute    g_route;
static NavRoute    g_routeCont;      // continuation polyline (type 0x06)
static NavManeuver g_man;
static NavManeuver g_man2;      // next-next maneuver (type 0x08)
static NavPos      g_pos;
static NavEta      g_eta;
static NavClock    g_clock;
static NavWeather  g_weather;
static volatile bool g_routeDirty = false;
static volatile bool g_contDirty  = false;
static volatile bool g_manDirty   = false;
static volatile bool g_man2Dirty  = false;
static volatile bool g_posDirty   = false;
static volatile bool g_etaDirty   = false;
static volatile bool g_clockDirty = false;
static volatile bool g_weatherDirty = false;
static volatile bool g_connected  = false;
static BLECharacteristic *g_navChar = nullptr;   // NOTIFY out (GPS broadcast)
static BLECharacteristic *g_weatherChar = nullptr;

/* ---- RX accumulation (mirrors car_nav ble_server.c) ---- */
#define RX_BUF_MAX 16384
static char rx_buf[RX_BUF_MAX];
static int  rx_len = 0;

/* ================= attribute helpers ================= */

static bool attrStr(const char *buf, const char *name, char *out, size_t outsz) {
  char key[40];
  snprintf(key, sizeof key, "%s=\"", name);
  const char *p = strstr(buf, key);
  if (!p) return false;
  p += strlen(key);
  size_t n = 0;
  while (*p && *p != '"' && n < outsz - 1) out[n++] = *p++;
  out[n] = 0;
  return true;
}

static bool attrDouble(const char *buf, const char *name, double &out) {
  char key[40];
  snprintf(key, sizeof key, "%s=\"", name);
  const char *p = strstr(buf, key);
  if (!p) return false;
  out = strtod(p + strlen(key), NULL);
  return true;
}

static bool attrInt(const char *buf, const char *name, int &out) {
  char key[40];
  snprintf(key, sizeof key, "%s=\"", name);
  const char *p = strstr(buf, key);
  if (!p) return false;
  out = (int)strtol(p + strlen(key), NULL, 10);
  return true;
}

/* ================= packet parsing ================= */

static void parseRoute(const char *buf) {
  static NavRoute r;   /* NOT on the BLE-host-task stack (NavRoute is ~4KB at 256 pts) */
  r.count = 0;
  r.zoom  = 15;
  attrInt(buf, "z", r.zoom);

  const char *p = buf;
  while (r.count < NAV_MAX_ROUTE_POINTS) {
    p = strstr(p, "<p ");
    if (!p) break;
    double lat = 0, lon = 0;
    if (attrDouble(p, "lat", lat) && attrDouble(p, "lon", lon)) {
      r.pts[r.count].lat = lat;
      r.pts[r.count].lon = lon;
      r.count++;
    }
    p += 3; /* advance past "<p " */
  }
  if (r.count >= 2) {
    g_route      = r;
    g_routeDirty = true;
    ESP_LOGI(TAG, "[nav] route: %d pts, zoom %d", r.count, r.zoom);
  } else {
    ESP_LOGI(TAG, "[nav] route: < 2 points, ignored");
  }
}

static void parseNav(const char *buf) {
  NavManeuver m;
  m.valid   = false;
  m.dist    = 0;
  m.maneuver[0] = 0;
  m.street[0]   = 0;
  if (attrInt(buf, "d", m.dist)) {
    attrStr(buf, "m", m.maneuver, sizeof m.maneuver);
    attrStr(buf, "s", m.street, sizeof m.street);
    m.valid   = true;
    g_man     = m;
    g_manDirty = true;
    ESP_LOGI(TAG, "[nav] maneuver m=%s d=%dm s=%s", m.maneuver, m.dist, m.street);
  }
}

static void parsePos(const char *buf) {
  NavPos p;
  p.valid = false;
  p.spd = 0;
  p.hdg = 0;
  p.limit = 0;
  if (attrDouble(buf, "lat", p.lat) && attrDouble(buf, "lon", p.lon)) {
    attrInt(buf, "spd", p.spd);
    attrInt(buf, "hdg", p.hdg);
    attrInt(buf, "sl", p.limit);   /* optional speed limit km/h */
    p.valid   = true;
    g_pos     = p;
    g_posDirty = true;
    ESP_LOGI(TAG, "[nav] pos %.5f,%.5f %dkm/h hdg=%d sl=%d", p.lat, p.lon, p.spd, p.hdg, p.limit);
  }
}

static void parseEta(const char *buf) {
  NavEta e;
  e.valid = false;
  e.hour = 0; e.minute = 0;
  e.arrive[0] = 0;
  if (attrInt(buf, "h", e.hour) && attrInt(buf, "m", e.minute)) {
    attrStr(buf, "a", e.arrive, sizeof e.arrive);
    e.valid = true;
    g_eta   = e;
    g_etaDirty = true;
    ESP_LOGI(TAG, "[nav] eta %02d:%02d a=%s", e.hour, e.minute, e.arrive);
  }
}

static void parseClock(const char *buf) {
  NavClock c;
  c.valid = false;
  c.hour = 0; c.minute = 0;
  if (attrInt(buf, "h", c.hour) && attrInt(buf, "m", c.minute)) {
    c.valid = true;
    g_clock = c;
    g_clockDirty = true;
    ESP_LOGI(TAG, "[nav] clock %02d:%02d", c.hour, c.minute);
  }
}

/* ================= binary protocol (compact, little-endian) =================
 * Frame : [0xAA 0x55] type len_lo len_hi payload
 *   type 0x01 ROUTE: zoom(u8) count(u16) lat0(i32 x1e7) lon0(i32 x1e7)
 *                    then (count-1) x [dlat(i16 x1e5) dlon(i16 x1e5)]
 *   type 0x02 POS  : lat(i32 x1e7) lon(i32 x1e7) spd(u8) hdg(u16) sl(u8)
 *   type 0x03 NAV  : dist(u16) modId(u8) slen(u8) street[slen]  (UTF-8)
 *   type 0x04 ETA  : h(u8) m(u8) alen(u8) arrive[alen]          (UTF-8)
 *   type 0x05 CLOCK: h(u8) m(u8)
 * Multiple frames may be packed into one write; frames may also span chunks.
 * XML is still accepted as a fallback (buffer not starting with the magic).
 */
#define BIN_MAGIC0 0xAA
#define BIN_MAGIC1 0x55

static const char *navManeuverName(uint8_t id) {
  static const char *names[] = {"straight","left","right","slight-left",
                                "slight-right","u-turn","roundabout","arrive"};
  return (id < 8) ? names[id] : names[0];
}

static bool decodeRouteBin(const uint8_t *p, int len, NavRoute *out) {
  if (len < 3) return false;
  int zoom  = p[0];
  int count = p[1] | (p[2] << 8);
  if (count < 2 || count > NAV_MAX_ROUTE_POINTS) return false;
  int need = 3 + 8 + (count - 1) * 4;
  if (len < need) return false;
  int32_t lat = (int32_t)((uint32_t)p[3] | ((uint32_t)p[4] << 8) | ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 24));
  int32_t lon = (int32_t)((uint32_t)p[7] | ((uint32_t)p[8] << 8) | ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24));
  /* decode DIRECTLY into *out (no local NavRoute) so a 256-pt route (~4KB)
   * never sits on the small BLE host task stack */
  out->count = 0;
  out->zoom  = zoom;
  out->pts[0].lat = lat / 1e7;
  out->pts[0].lon = lon / 1e7;
  out->count = 1;
  for (int i = 1; i < count; i++) {
    int off = 11 + (i - 1) * 4;
    int16_t dl = (int16_t)((uint16_t)p[off] | ((uint16_t)p[off + 1] << 8));
    int16_t dn = (int16_t)((uint16_t)p[off + 2] | ((uint16_t)p[off + 3] << 8));
    lat += (int32_t)dl * 100;   /* delta x1e5 -> x1e7 */
    lon += (int32_t)dn * 100;
    out->pts[i].lat = lat / 1e7;
    out->pts[i].lon = lon / 1e7;
    out->count++;
  }
  return true;
}

static void parseRouteBin(const uint8_t *p, int len) {
  static NavRoute r;   /* static: keeps the 256-pt struct off the BLE task stack */
  if (!decodeRouteBin(p, len, &r)) return;
  g_route      = r;
  g_routeDirty = true;
  ESP_LOGI(TAG, "[nav] route(bin): %d pts, zoom %d", r.count, r.zoom);
}

/* 0x06 — route continuation: same wire format as 0x01, drawn fainter behind
 * the near path-ahead so the driver can see where the road goes next. */
static void parseRouteContBin(const uint8_t *p, int len) {
  static NavRoute r;   /* static: keeps the 256-pt struct off the BLE task stack */
  if (!decodeRouteBin(p, len, &r)) return;
  g_routeCont  = r;
  g_contDirty  = true;
  ESP_LOGI(TAG, "[nav] route-cont(bin): %d pts, zoom %d", r.count, r.zoom);
}

/* 0x07 — weather: tempC(s8) humidity(u8) code(u8) slen(u8) text[UTF-8] */
static void parseWeatherBin(const uint8_t *p, int len) {
  NavWeather w;
  w.valid = false;
  w.tempC = 0; w.humidity = 0; w.code = 0; w.text[0] = 0;
  if (len < 4) return;
  w.tempC    = (int8_t)p[0];
  w.humidity = p[1];
  w.code     = p[2];
  int slen = p[3];
  if (slen > len - 4) slen = len - 4;
  if (slen > (int)sizeof w.text - 1) slen = sizeof w.text - 1;
  memcpy(w.text, p + 4, slen);
  w.text[slen] = 0;
  w.valid      = true;
  g_weather      = w;
  g_weatherDirty = true;
  ESP_LOGI(TAG, "[nav] weather(bin) %dC %d%% code=%d \"%s\"", w.tempC, w.humidity, w.code, w.text);
}

static void parsePosBin(const uint8_t *p, int len) {
  if (len < 12) return;
  int32_t lat = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
  int32_t lon = (int32_t)((uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24));
  uint8_t  spd = p[8];
  uint16_t hdg = (uint16_t)(p[9] | (p[10] << 8));
  uint8_t  sl  = p[11];
  NavPos pos;
  pos.valid = true;
  pos.lat   = lat / 1e7;
  pos.lon   = lon / 1e7;
  pos.spd   = spd;
  pos.hdg   = hdg;
  pos.limit = sl;
  g_pos      = pos;
  g_posDirty = true;
  ESP_LOGI(TAG, "[nav] pos(bin) %.5f,%.5f %dkm/h hdg=%d sl=%d", pos.lat, pos.lon, pos.spd, pos.hdg, pos.limit);
}

static bool decodeNavBinPayload(const uint8_t *p, int len, NavManeuver &m) {
  m.valid = false;
  m.dist  = 0;
  m.maneuver[0] = 0;
  m.street[0]   = 0;
  if (len < 4) return false;
  m.dist = p[0] | (p[1] << 8);
  uint8_t mid = p[2];
  int slen = p[3];
  if (slen > len - 4) slen = len - 4;
  if (slen > (int)sizeof m.street - 1) slen = sizeof m.street - 1;
  memcpy(m.street, p + 4, slen);
  m.street[slen] = 0;
  strncpy(m.maneuver, navManeuverName(mid), sizeof m.maneuver - 1);
  m.maneuver[sizeof m.maneuver - 1] = 0;
  m.valid = true;
  return true;
}

static void parseNavBin(const uint8_t *p, int len) {
  NavManeuver m;
  if (!decodeNavBinPayload(p, len, m)) return;
  g_man      = m;
  g_manDirty = true;
  ESP_LOGI(TAG, "[nav] maneuver(bin) m=%s d=%dm s=%s", m.maneuver, m.dist, m.street);
}

static void parseNav2Bin(const uint8_t *p, int len) {
  NavManeuver m;
  if (!decodeNavBinPayload(p, len, m)) return;
  g_man2      = m;
  g_man2Dirty = true;
  ESP_LOGI(TAG, "[nav] maneuver2(bin) m=%s d=%dm s=%s", m.maneuver, m.dist, m.street);
}

static void parseEtaBin(const uint8_t *p, int len) {
  NavEta e;
  e.valid = false;
  e.hour = 0; e.minute = 0;
  e.arrive[0] = 0;
  if (len < 2) return;
  e.hour   = p[0];
  e.minute = p[1];
  int slen = (len >= 3) ? p[2] : 0;
  if (slen > len - 3) slen = len - 3;
  if (slen > (int)sizeof e.arrive - 1) slen = sizeof e.arrive - 1;
  memcpy(e.arrive, p + 3, slen);
  e.arrive[slen] = 0;
  e.valid    = true;
  g_eta      = e;
  g_etaDirty = true;
  ESP_LOGI(TAG, "[nav] eta(bin) %02d:%02d a=%s", e.hour, e.minute, e.arrive);
}

static void parseClockBin(const uint8_t *p, int len) {
  NavClock c;
  c.valid = false;
  c.hour = 0; c.minute = 0;
  if (len < 2) return;
  c.hour   = p[0];
  c.minute = p[1];
  c.valid      = true;
  g_clock      = c;
  g_clockDirty = true;
  ESP_LOGI(TAG, "[nav] clock(bin) %02d:%02d", c.hour, c.minute);
}

static void dispatchBinary(uint8_t type, const uint8_t *p, int len) {
  switch (type) {
    case 0x01: parseRouteBin(p, len);      break;
    case 0x02: parsePosBin(p, len);        break;
    case 0x03: parseNavBin(p, len);        break;
    case 0x04: parseEtaBin(p, len);        break;
    case 0x05: parseClockBin(p, len);      break;
    case 0x06: parseRouteContBin(p, len);  break;
    case 0x07: parseWeatherBin(p, len);    break;
    case 0x08: parseNav2Bin(p, len);       break;
    default:
      ESP_LOGI(TAG, "[nav] unknown binary type %d (%d bytes)", type, len);
      break;
  }
}

static void finalizePacket(void) {
  if (rx_len <= 0) return;
  rx_buf[rx_len] = 0;
  if (strstr(rx_buf, "<route")) parseRoute(rx_buf);
  else if (strstr(rx_buf, "<nav")) parseNav(rx_buf);
  else if (strstr(rx_buf, "<pos")) parsePos(rx_buf);
  else if (strstr(rx_buf, "<eta")) parseEta(rx_buf);
  else if (strstr(rx_buf, "<clock")) parseClock(rx_buf);
  else ESP_LOGI(TAG, "[nav] unknown packet (%d bytes)", rx_len);
  rx_len = 0;
}

static void rx_put(char b) {
  if (rx_len < RX_BUF_MAX - 1) rx_buf[rx_len++] = b;
  rx_buf[rx_len] = 0;

  /* binary frames — length-framed, may span multiple chunked writes */
  if (rx_len >= 2 && (uint8_t)rx_buf[0] == BIN_MAGIC0 && (uint8_t)rx_buf[1] == BIN_MAGIC1) {
    while (rx_len >= 5) {
      uint8_t type = (uint8_t)rx_buf[2];
      int len = (uint8_t)rx_buf[3] | ((uint8_t)rx_buf[4] << 8);
      if (len > RX_BUF_MAX - 8) { rx_len = 0; return; }   /* bogus frame, drop */
      if (rx_len < 5 + len) return;                        /* wait for more chunks */
      dispatchBinary(type, (const uint8_t *)rx_buf + 5, len);
      int total = 5 + len;
      memmove(rx_buf, rx_buf + total, rx_len - total);
      rx_len -= total;
      rx_buf[rx_len] = 0;
      if (rx_len < 2 || (uint8_t)rx_buf[0] != BIN_MAGIC0 || (uint8_t)rx_buf[1] != BIN_MAGIC1) break;
    }
    return;
  }

  /* legacy XML path (finalize on closing tag / 0x00) */
  if (b == 0) { finalizePacket(); return; }

  int n = rx_len;
  if (n >= 7 && strcmp(rx_buf + n - 7, "</route>") == 0) finalizePacket();
  else if (n >= 6 && strcmp(rx_buf + n - 6, "</nav>") == 0) finalizePacket();
  else if (n >= 6 && strcmp(rx_buf + n - 6, "</pos>") == 0) finalizePacket();
  else if (n >= 6 && strcmp(rx_buf + n - 6, "</eta>") == 0) finalizePacket();
  else if (n >= 8 && strcmp(rx_buf + n - 8, "</clock>") == 0) finalizePacket();
}

/* ================= GATT callbacks (vendor demo pattern) ================= */

class NavCharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    /* Use the RAW value, NOT c_str(): binary frames contain 0x00 bytes in the
     * middle (length high-byte, distances, empty streets...), and c_str()
     * truncates at the first NUL — which made the parser wait forever.
     * getValue() returns an Arduino String built with an explicit length, so
     * length()/operator[] are NUL-safe. */
    String v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) rx_put(v[i]);
  }
};

/* Weather service: same binary framing (0xAA 0x55 type 0x07 ...) — reuse the
 * shared parser via rx_put so weather and nav share one code path. */
class WeatherCharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) rx_put(v[i]);
  }
};

/* Restart advertising AFTER the controller finishes the disconnect teardown.
 * Calling BLEDevice::startAdvertising() directly inside onDisconnect races with
 * the HCI disconnect processing and silently fails on this board, leaving the
 * device unadvertised ("cannot connect") until reboot. A short delay in a
 * dedicated task plus a full advertising reset (forces Bluedroid to rebuild and
 * re-push the adv data to the controller) fixes it reliably. */
static void adv_restart_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(600));   /* let the controller finish teardown */
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->reset();                     /* m_advDataSet=false -> full reconfig */
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    ESP_LOGI(TAG, "[nav] advertising restarted (full reset)");
    vTaskDelete(NULL);
}

class NavSrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    g_connected = true;
    ESP_LOGI(TAG, "[nav] BLE connected");
  }
  void onDisconnect(BLEServer *s) override {
    g_connected = false;
    ESP_LOGI(TAG, "[nav] BLE disconnected");
    xTaskCreate(adv_restart_task, "adv_restart", 4096, NULL, 5, NULL);
  }
};

/* ================= public API ================= */

static void gps_ble_task(void *arg);   /* defined after bleNavBegin */

void bleNavBegin(void) {
  BLEDevice::init(ADV_NAME);

  /* Larger ATT MTU: the route (~44-100 B) then arrives as ONE ATT write instead
   * of several small packets, which avoids the intermittent multi-packet RX path
   * that asserts the BLE controller buffer pool (ble_util_buf.c / rwble.c) and
   * trips the interrupt WDT on this board. */
  esp_ble_gatt_set_local_mtu(517);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new NavSrvCB());

  BLEService *svc = server->createService(SERVICE_UUID);
  BLECharacteristic *chr = svc->createCharacteristic(
      CHAR_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR |
          BLECharacteristic::PROPERTY_NOTIFY);
  chr->setCallbacks(new NavCharCB());
  chr->addDescriptor(new BLE2902());
  chr->setValue("NAV-OSM ready");
  g_navChar = chr;   /* used to push GPS NMEA to the phone (NOTIFY) */
  svc->start();

  /* --- second service: weather (phone -> ESP) --- */
  BLEService *wsvc = server->createService(WEATHER_SERVICE_UUID);
  BLECharacteristic *wchr = wsvc->createCharacteristic(
      WEATHER_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
  wchr->setCallbacks(new WeatherCharCB());
  g_weatherChar = wchr;
  wsvc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->addServiceUUID(WEATHER_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); /* these are the recommended values */
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  /* GPS broadcast: when enabled + a phone is connected, push raw NMEA to the
   * phone on the nav char's NOTIFY (one line per ~1s when a fix is flowing). */
  xTaskCreate(gps_ble_task, "gps_ble", 4096, NULL, 4, NULL);

  ESP_LOGI(TAG, "[nav] BLE up, advertising as %s", ADV_NAME);
}

/* Push queued raw NMEA lines to the phone while broadcast is enabled.
 * notify() is a no-op when no client is subscribed, so the guard is just
 * connection + the user's broadcast toggle. Rate is 1 Hz — the user asked for
 * a 1 Hz GPS broadcast, not 10 Hz (was pdMS_TO_TICKS(100)). */
static void gps_ble_task(void *arg) {
  char line[128];
  for (;;) {
    if (g_connected && gps_broadcast_enabled() && g_navChar) {
      int n = gps_read_nmea(line, sizeof line);
      if (n > 0) {
        g_navChar->setValue((uint8_t *)line, n);
        g_navChar->notify();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));   /* 1 Hz GPS broadcast */
  }
}

bool navConnected(void) { return g_connected; }

bool navRouteDirty(void)   { return g_routeDirty; }
void navRouteClearDirty(void)   { g_routeDirty = false; }
bool navContDirty(void)   { return g_contDirty; }
void navContClearDirty(void)   { g_contDirty = false; }
bool navManeuverDirty(void) { return g_manDirty; }
void navManeuverClearDirty(void) { g_manDirty = false; }
bool navMan2Dirty(void) { return g_man2Dirty; }
void navMan2ClearDirty(void) { g_man2Dirty = false; }
bool navPosDirty(void)      { return g_posDirty; }
void navPosClearDirty(void)     { g_posDirty = false; }

const NavRoute*    navGetRoute(void)    { return &g_route; }
const NavRoute*    navGetRouteCont(void) { return &g_routeCont; }
const NavManeuver* navGetManeuver(void) { return &g_man; }
const NavManeuver* navGetManeuver2(void) { return &g_man2; }
const NavPos*      navGetPos(void)      { return &g_pos; }
const NavEta*      navGetEta(void)      { return &g_eta; }
const NavClock*    navGetClock(void)    { return &g_clock; }
const NavWeather*  navGetWeather(void)  { return &g_weather; }

bool navEtaDirty(void)   { return g_etaDirty; }
void navEtaClearDirty(void)   { g_etaDirty = false; }
bool navClockDirty(void) { return g_clockDirty; }
void navClockClearDirty(void) { g_clockDirty = false; }
bool navWeatherDirty(void) { return g_weatherDirty; }
void navWeatherClearDirty(void) { g_weatherDirty = false; }

void navSetGpsBroadcast(bool on) { gps_set_broadcast(on); }
bool navGpsBroadcast(void) { return gps_broadcast_enabled(); }
