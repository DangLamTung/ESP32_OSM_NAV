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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SERVICE_UUID "5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c"
#define CHAR_UUID    "5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c"
#define ADV_NAME     "NAV-OSM"

/* ---- decoded state ---- */
static NavRoute    g_route;
static NavManeuver g_man;
static NavPos      g_pos;
static volatile bool g_routeDirty = false;
static volatile bool g_manDirty   = false;
static volatile bool g_posDirty   = false;
static volatile bool g_connected  = false;

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
  NavRoute r;
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
    Serial.printf("[nav] route: %d pts, zoom %d\n", r.count, r.zoom);
  } else {
    Serial.println("[nav] route: < 2 points, ignored");
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
    Serial.printf("[nav] maneuver m=%s d=%dm s=%s\n", m.maneuver, m.dist, m.street);
  }
}

static void parsePos(const char *buf) {
  NavPos p;
  p.valid = false;
  p.spd = 0;
  p.hdg = 0;
  if (attrDouble(buf, "lat", p.lat) && attrDouble(buf, "lon", p.lon)) {
    attrInt(buf, "spd", p.spd);
    attrInt(buf, "hdg", p.hdg);
    p.valid   = true;
    g_pos     = p;
    g_posDirty = true;
    Serial.printf("[nav] pos %.5f,%.5f %dkm/h hdg=%d\n", p.lat, p.lon, p.spd, p.hdg);
  }
}

static void finalizePacket(void) {
  if (rx_len <= 0) return;
  rx_buf[rx_len] = 0;
  if (strstr(rx_buf, "<route")) parseRoute(rx_buf);
  else if (strstr(rx_buf, "<nav")) parseNav(rx_buf);
  else if (strstr(rx_buf, "<pos")) parsePos(rx_buf);
  else Serial.printf("[nav] unknown packet (%d bytes)\n", rx_len);
  rx_len = 0;
}

static void rx_put(char b) {
  if (rx_len < RX_BUF_MAX - 1) rx_buf[rx_len++] = b;
  rx_buf[rx_len] = 0;

  if (b == 0) { finalizePacket(); return; }

  int n = rx_len;
  if (n >= 7 && strcmp(rx_buf + n - 7, "</route>") == 0) finalizePacket();
  else if (n >= 6 && strcmp(rx_buf + n - 6, "</nav>") == 0) finalizePacket();
  else if (n >= 6 && strcmp(rx_buf + n - 6, "</pos>") == 0) finalizePacket();
}

/* ================= GATT callbacks (vendor demo pattern) ================= */

class NavCharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    std::string v = std::string(c->getValue().c_str());
    for (size_t i = 0; i < v.size(); i++) rx_put(v[i]);
  }
};

class NavSrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    g_connected = true;
    Serial.println("[nav] BLE connected");
  }
  void onDisconnect(BLEServer *s) override {
    g_connected = false;
    Serial.println("[nav] BLE disconnected");
    BLEDevice::startAdvertising(); /* re-advertise for the next phone */
  }
};

/* ================= public API ================= */

void bleNavBegin(void) {
  BLEDevice::init(ADV_NAME);
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
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); /* these are the recommended values */
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.printf("[nav] BLE up, advertising as %s\n", ADV_NAME);
}

bool navConnected(void) { return g_connected; }

bool navRouteDirty(void)   { return g_routeDirty; }
void navRouteClearDirty(void)   { g_routeDirty = false; }
bool navManeuverDirty(void) { return g_manDirty; }
void navManeuverClearDirty(void) { g_manDirty = false; }
bool navPosDirty(void)      { return g_posDirty; }
void navPosClearDirty(void)     { g_posDirty = false; }

const NavRoute*    navGetRoute(void)    { return &g_route; }
const NavManeuver* navGetManeuver(void) { return &g_man; }
const NavPos*      navGetPos(void)      { return &g_pos; }
