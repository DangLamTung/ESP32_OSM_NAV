/**
 * ble_scan.h — background BLE scanner for the OSM tile viewer.
 *
 * Based on the manufacturer's Example_25_BLE_scan (BLEScan +
 * BLEAdvertisedDeviceCallbacks, active scan). Runs in its own low-priority
 * task so it never blocks the map UI. Results are listed in an overlay on the
 * OSM map (toggled by the corner SCAN button in main.cpp).
 *
 * Note: BLE scanning does NOT give GPS coordinates, so discovered devices are
 * shown as a list (name/address/RSSI), not placed geographically.
 */
#ifndef BLE_SCAN_H
#define BLE_SCAN_H

#include <Arduino.h>

#define BLE_SCAN_MAX 12

typedef struct {
  bool  haveName;
  char  name[48];
  char  addr[18];   // "aa:bb:cc:dd:ee:ff"
  int   rssi;
} ScanDev;

void bleScanBegin(void);          // call after bleNavBegin() (shares BLEDevice)
void bleScanStart(void);          // start the periodic scan task
int  bleScanCount(void);
const ScanDev* bleScanGet(int i);

#endif // BLE_SCAN_H
