/**
 * ble_scan.cpp — background BLE scanner (vendor Example_25 pattern).
 *
 * Scans 5 s, pauses 5 s, repeats. Results accumulate into a fixed array that
 * main.cpp renders as a list overlay on the OSM map.
 */
#include "ble_scan.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <stdio.h>
#include <string.h>

static ScanDev      g_devs[BLE_SCAN_MAX];
static int          g_count = 0;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_task = NULL;

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    portENTER_CRITICAL(&g_mux);
    if (g_count < BLE_SCAN_MAX) {
      ScanDev *d = &g_devs[g_count];
      d->haveName = dev.haveName();
      d->name[0] = 0;
      if (dev.haveName()) {
        strncpy(d->name, dev.getName().c_str(), sizeof(d->name) - 1);
        d->name[sizeof(d->name) - 1] = 0;
      }
      snprintf(d->addr, sizeof d->addr, "%s", dev.getAddress().toString().c_str());
      d->rssi = dev.haveRSSI() ? dev.getRSSI() : 0;
      g_count++;
    }
    portEXIT_CRITICAL(&g_mux);
  }
};

static void scanTask(void *) {
  BLEScan *scan = BLEDevice::getScan();
  for (;;) {
    portENTER_CRITICAL(&g_mux);
    g_count = 0;                              /* restart each cycle */
    portEXIT_CRITICAL(&g_mux);
    BLEScanResults *res = scan->start(5, false);
    Serial.printf("[scan] %d devices found\n", res->getCount());
    scan->clearResults();                     /* free the scan buffer */
    vTaskDelay(pdMS_TO_TICKS(5000));          /* pause between scans */
  }
}

void bleScanBegin(void) {
  BLEScan *s = BLEDevice::getScan();
  s->setAdvertisedDeviceCallbacks(new ScanCB());
  s->setActiveScan(true);                     /* faster, uses more power */
  s->setInterval(100);
  s->setWindow(99);
  Serial.println("[scan] BLE scanner ready");
}

void bleScanStart(void) {
  if (g_task) return;
  xTaskCreatePinnedToCore(scanTask, "blescan", 4096, NULL, 1, &g_task, 1);
  Serial.println("[scan] scanner task started");
}

int bleScanCount(void) {
  int c;
  portENTER_CRITICAL(&g_mux);
  c = g_count;
  portEXIT_CRITICAL(&g_mux);
  return c > BLE_SCAN_MAX ? BLE_SCAN_MAX : c;
}

const ScanDev *bleScanGet(int i) {
  portENTER_CRITICAL(&g_mux);
  const ScanDev *d = (i >= 0 && i < g_count) ? &g_devs[i] : NULL;
  portEXIT_CRITICAL(&g_mux);
  return d;
}
