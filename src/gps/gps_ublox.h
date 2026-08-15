/**
 * gps_ublox.h — U-Blox NMEA GPS receiver over a UART.
 *
 * Reads $GPGGA / $GPRMC sentences and exposes the last fix + fix state.
 * Optional: when [gpsBroadcastEnabled] is set, a periodic NMEA $GPRMC/GGA
 * line is re-emitted on demand so it can be forwarded to the phone over BLE
 * (protocol: raw NMEA text, matching the u-blox module).
 */
#ifndef GPS_UBLOX_H_
#define GPS_UBLOX_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool   valid;        // a fix has been seen
  double lat, lon;     // degrees
  double speedKmh;     // ground speed
  double course;       // true course, degrees 0..359
  int    fixQuality;   // 0 = no fix, 1 = GPS fix, 2 = DGPS
  int    sats;
  int    hour, minute, second;   // UTC time from the last sentence
} GpsFix;

void gps_init(void);                    /* configure UART + start reader task */
bool gps_fix(GpsFix *out);              /* copy the latest fix; false if none */
bool gps_broadcast_enabled(void);
void gps_set_broadcast(bool on);
void gps_toggle_broadcast(void);        /* settings button handler */
int  gps_read_nmea(char *buf, int bufsz); /* next raw NMEA line (broadcast) */

#ifdef __cplusplus
}
#endif

#endif // GPS_UBLOX_H_
