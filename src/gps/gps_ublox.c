/**
 * gps_ublox.c — U-Blox NMEA GPS receiver (UART) with a broadcast queue.
 *
 * The module owns a UART (default UART2, pins 4/5). A reader task consumes
 * NMEA lines, parses $GPGGA (fix quality / sats / lat-lon) and $GPRMC
 * (speed / course / time), and keeps the latest GpsFix. When broadcast is
 * enabled, the raw NMEA line is also copied into a small ring so the BLE
 * server can forward it to the phone verbatim.
 */
#include "gps_ublox.h"

#include <string.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.h"

static const char *TAG = "gps";

#define GPS_LINE_MAX 128
#define GPS_QUEUE_MAX 4

static GpsFix s_fix = {0};
static bool   s_broadcast = false;

/* raw NMEA broadcast ring (single consumer = BLE server) */
static char   s_queue[GPS_QUEUE_MAX][GPS_LINE_MAX];
static int    s_qHead = 0, s_qTail = 0, s_qCount = 0;

/* ---- NMEA helpers ---- */

/* Copy field `idx` (0-based, comma-separated) into out. Returns true if present. */
static bool field(const char *line, int idx, char *out, int outsz) {
  const char *p = line;
  int cur = 0;
  while (p && *p) {
    const char *end = strchr(p, ',');
    if (!end) end = p + strlen(p);
    if (cur == idx) {
      int n = (int)(end - p);
      if (n >= outsz) n = outsz - 1;
      memcpy(out, p, n);
      out[n] = 0;
      return n > 0;
    }
    if (*end == 0) return false;
    p = end + 1;
    cur++;
  }
  return false;
}

/* "ddmm.mmmm" -> decimal degrees. Returns NaN on parse error. */
static double parse_nmea_lat(const char *s) {
  char *end = NULL;
  double v = strtod(s, &end);
  if (end == s) return __builtin_nanf("0");
  int deg = (int)(v / 100.0);
  double min = v - deg * 100.0;
  return deg + min / 60.0;
}

static void parse_line(const char *line) {
  char buf[GPS_LINE_MAX];

  if (strncmp(line, "$GPGGA", 6) == 0) {
    GpsFix f = s_fix;
    if (field(line, 2, buf, sizeof buf)) {           /* lat "ddmm.mmmm" */
      double v = parse_nmea_lat(buf);
      if (v == v) {                                  /* not NaN */
        f.lat = v;
        if (field(line, 3, buf, sizeof buf) && buf[0] == 'S') f.lat = -f.lat;
        if (field(line, 4, buf, sizeof buf)) {       /* lon */
          double lo = parse_nmea_lat(buf);
          if (lo == lo) {
            f.lon = lo;
            if (field(line, 5, buf, sizeof buf) && buf[0] == 'W') f.lon = -f.lon;
          }
        }
      }
    }
    if (field(line, 6, buf, sizeof buf)) f.fixQuality = atoi(buf);
    if (field(line, 7, buf, sizeof buf)) f.sats = atoi(buf);
    if (f.fixQuality > 0) f.valid = true;
    s_fix = f;
  } else if (strncmp(line, "$GPRMC", 6) == 0) {
    GpsFix f = s_fix;
    if (field(line, 1, buf, sizeof buf) && strlen(buf) == 6) {   /* hhmmss */
      f.hour   = (buf[0]-'0')*10 + (buf[1]-'0');
      f.minute = (buf[2]-'0')*10 + (buf[3]-'0');
      f.second = (buf[4]-'0')*10 + (buf[5]-'0');
    }
    if (field(line, 7, buf, sizeof buf)) f.speedKmh = strtod(buf, NULL) * 1.852; /* knots->km/h */
    if (field(line, 8, buf, sizeof buf)) f.course = strtod(buf, NULL);
    if (field(line, 2, buf, sizeof buf) && buf[0] == 'A') f.valid = true;
    s_fix = f;
  }
}

/* ---- UART reader task ---- */
static void gps_task(void *arg) {
  char line[GPS_LINE_MAX];
  int li = 0;
  uint8_t b;
  while (1) {
    int n = uart_read_bytes(GPS_UART_NUM, &b, 1, pdMS_TO_TICKS(50));
    if (n == 1) {
      if (b == '\n') {
        line[li] = 0;
        if (li > 0) {
          parse_line(line);
          if (s_broadcast && s_qCount < GPS_QUEUE_MAX) {
            memcpy(s_queue[s_qHead], line, li + 1);
            s_qHead = (s_qHead + 1) % GPS_QUEUE_MAX;
            s_qCount++;
          }
        }
        li = 0;
      } else if (b == '\r') {
        /* skip */
      } else if (li < GPS_LINE_MAX - 1) {
        line[li++] = (char)b;
      }
    }
  }
}

void gps_init(void) {
  uart_config_t cfg = {
    .baud_rate = GPS_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  esp_err_t e = uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0);
  if (e != ESP_OK) ESP_LOGW(TAG, "uart install: %s", esp_err_to_name(e));
  uart_param_config(GPS_UART_NUM, &cfg);
  uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  ESP_LOGI(TAG, "U-Blox GPS on UART%d (TX=%d RX=%d)", GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN);
  xTaskCreate(gps_task, "gps", 4096, NULL, 5, NULL);
}

bool gps_fix(GpsFix *out) {
  if (!out) return false;
  *out = s_fix;
  return s_fix.valid;
}

bool gps_broadcast_enabled(void) { return s_broadcast; }
void gps_set_broadcast(bool on) { s_broadcast = on; ESP_LOGI(TAG, "broadcast %s", on ? "ON" : "OFF"); }
void gps_toggle_broadcast(void) { gps_set_broadcast(!s_broadcast); }

/* Pop one raw NMEA line for BLE forwarding. Returns length, 0 when empty. */
int gps_read_nmea(char *buf, int bufsz) {
  if (s_qCount == 0 || !buf || bufsz <= 0) return 0;
  int len = (int)strlen(s_queue[s_qTail]);
  if (len >= bufsz) len = bufsz - 1;
  memcpy(buf, s_queue[s_qTail], len);
  buf[len] = 0;
  s_qTail = (s_qTail + 1) % GPS_QUEUE_MAX;
  s_qCount--;
  return len;
}
