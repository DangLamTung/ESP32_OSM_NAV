/**
 * ble_nav.h — BLE navigation receiver for the OSM tile viewer.
 *
 * Implements the BLE GATT server exactly like the manufacturer's demo
 * (Example_26_BLE_server: BLEDevice / BLEServer / BLECharacteristic /
 * BLE2902 / notify), but keeps the navbridge-compatible service/char UUIDs
 * from car_nav/src/ble_server.c so the existing phone app connects unchanged.
 *
 * Protocol (compact XML, packet finalized on </route> | </nav> | </pos> | 0x00):
 *   <route z="15"><p lat=".." lon=".."/>...</route>   full route polyline (once)
 *   <nav d="85" m="left" s="Nguyen Hue"/>             current maneuver (~1 Hz)
 *   <pos lat=".." lon=".." spd="34" hdg="312"/>       live position (~1 Hz)
 */
#ifndef BLE_NAV_H
#define BLE_NAV_H

#include <Arduino.h>

#define NAV_MAX_ROUTE_POINTS 512
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
} NavPos;

void bleNavBegin(void);

bool navConnected(void);

bool navRouteDirty(void);
void navRouteClearDirty(void);
bool navManeuverDirty(void);
void navManeuverClearDirty(void);
bool navPosDirty(void);
void navPosClearDirty(void);

const NavRoute*    navGetRoute(void);
const NavManeuver* navGetManeuver(void);
const NavPos*      navGetPos(void);

#endif // BLE_NAV_H
