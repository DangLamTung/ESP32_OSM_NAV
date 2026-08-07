/**
 * ui_controls.cpp — on-screen controls + overlays module.
 */
#include "ui_controls.h"
#include "app_config.h"
#include "display_panel.h"
#include "mercator.h"
#include "ble_nav.h"
#include "ble_scan.h"
#include "map_view.h"
#include <Arduino.h>
#include <string.h>

static bool g_showScan = false;
static bool s_uiDirty  = true;

bool ui_scan_shown(void)   { return g_showScan; }
bool ui_needs_redraw(void) { return s_uiDirty; }
void ui_clear_redraw(void) { s_uiDirty = false; }
void ui_mark_redraw(void)  { s_uiDirty = true; }

void ui_init(void)
{
    g_showScan = false;
    s_uiDirty  = true;
}

void ui_toggle_scan(void)
{
    g_showScan = !g_showScan;
    Serial.printf("[scan] list %s\n", g_showScan ? "ON" : "OFF");
    /* turning OFF: restore the map under the overlay (re-push cached sprite).
     * turning ON: the loop just draws the panel on top of the map. */
    if (!g_showScan) map_push();
    ui_mark_redraw();
}

/* ---- Vietnamese diacritics -> ASCII (built-in font has no unicode) ---- */
static uint32_t utf8Next(const char **p) {
  const unsigned char *s = (const unsigned char *)*p;
  uint32_t cp = *s;
  if (cp >= 0x80) {
    int extra = 0;
    if ((cp & 0xE0) == 0xC0) { extra = 1; cp &= 0x1F; }
    else if ((cp & 0xF0) == 0xE0) { extra = 2; cp &= 0x0F; }
    else { *p += 1; return '?'; }
    for (int i = 0; i < extra; i++) cp = (cp << 6) | (s[1 + i] & 0x3F);
    *p += extra + 1;
  } else {
    *p += 1;
  }
  return cp;
}

static char vietBase(uint32_t cp) {
  static const uint32_t vp[] = {
    0xE0,0xE1,0x1EA3,0xE3,0x1EA1,
    0x0103,0x1EAF,0x1EB1,0x1EB3,0x1EB5,0x1EB7,
    0x00E2,0x1EA5,0x1EA7,0x1EA9,0x1EAB,0x1EAD,
    0x00E8,0x00E9,0x1EBB,0x1EBD,0x1EB9,
    0x00EA,0x1EBF,0x1EC1,0x1EC3,0x1EC5,0x1EC7,
    0x00EC,0x00ED,0x1EC9,0x0129,0x1ECB,
    0x00F2,0x00F3,0x1ECF,0x00F5,0x1ECD,
    0x00F4,0x1ED1,0x1ED3,0x1ED5,0x1ED7,0x1ED9,
    0x01A1,0x1EDB,0x1EDD,0x1EDF,0x1EE1,0x1EE3,
    0x00F9,0x00FA,0x1EE7,0x0169,0x1EE5,
    0x01B0,0x1EE9,0x1EEB,0x1EED,0x1EEF,0x1EF1,
    0x00FD,0x1EF3,0x1EF7,0x1EF9,0x1EF5,
    0x00C0,0x00C1,0x1EA2,0x00C3,0x1EA0,
    0x0102,0x1EAE,0x1EB0,0x1EB2,0x1EB4,0x1EB6,
    0x00C2,0x1EA4,0x1EA6,0x1EA8,0x1EAA,0x1EAC,
    0x00C8,0x00C9,0x1EBA,0x1EBC,0x1EB8,
    0x00CA,0x1EBE,0x1EC0,0x1EC2,0x1EC4,0x1EC6,
    0x00CC,0x00CD,0x1EC8,0x0128,0x1ECA,
    0x00D2,0x00D3,0x1ECE,0x00D5,0x1ECC,
    0x00D4,0x1ED0,0x1ED2,0x1ED4,0x1ED6,0x1ED8,
    0x01A0,0x1EDA,0x1EDC,0x1EDE,0x1EE0,0x1EE2,
    0x00D9,0x00DA,0x1EE6,0x0168,0x1EE4,
    0x01AF,0x1EE8,0x1EEA,0x1EEC,0x1EEE,0x1EF0,
    0x00DD,0x1EF2,0x1EF6,0x1EF8,0x1EF4,
    0x0111,0x0110,
  };
  static const char *vb = "aaaaa"
                          "aaaaaa"
                          "aaaaaa"
                          "eeeee"
                          "eeeeee"
                          "iiiii"
                          "ooooo"
                          "oooooo"
                          "oooooo"
                          "uuuuu"
                          "uuuuuu"
                          "yyyyy"
                          "AAAAA"
                          "AAAAAA"
                          "AAAAAA"
                          "EEEEE"
                          "EEEEEE"
                          "IIIII"
                          "OOOOO"
                          "OOOOOO"
                          "OOOOOO"
                          "UUUUU"
                          "UUUUUU"
                          "YYYYY"
                          "dD";
  for (size_t i = 0; i < sizeof(vp) / sizeof(vp[0]); i++)
    if (vp[i] == cp) return vb[i];
  return (cp < 128) ? (char)cp : '?';
}

static void stripViet(char *out, const char *in, size_t outsz) {
  size_t n = 0;
  while (*in && n < outsz - 1) {
    uint32_t cp = utf8Next(&in);
    char c = vietBase(cp);
    if (c && n < outsz - 1) out[n++] = c;
  }
  out[n] = 0;
}

/* ---- maneuver HUD (arrow + distance + street) ---- */
static void drawManeuverHUD(LGFX_Sprite &spr, const NavManeuver *man) {
  int bx = 4, by = 4, bw = 150, bh = 46;
  spr.fillRect(bx, by, bw, bh, TFT_BLACK);
  spr.drawRect(bx, by, bw, bh, TFT_YELLOW);

  /* turn arrow (triangle) pointing per maneuver */
  int cx = bx + 24, cy = by + bh / 2;
  int L = 18, W = 13;
  if      (!strcmp(man->maneuver, "left"))    spr.fillTriangle(cx - L, cy, cx, cy - W, cx, cy + W, TFT_GREEN);
  else if (!strcmp(man->maneuver, "right"))   spr.fillTriangle(cx + L, cy, cx, cy - W, cx, cy + W, TFT_GREEN);
  else if (!strcmp(man->maneuver, "u-turn"))  spr.fillTriangle(cx, cy + L, cx - W, cy, cx + W, cy, TFT_GREEN);
  else if (!strcmp(man->maneuver, "arrive"))  spr.fillCircle(cx, cy, 7, TFT_RED);
  else                                         spr.fillTriangle(cx, cy - L, cx - W, cy, cx + W, cy, TFT_GREEN); /* straight (default) */

  /* distance */
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setTextFont(2);
  spr.setCursor(bx + 44, by + 4);
  if (man->dist > 0) spr.print(man->dist);
  spr.print(" m");

  /* street name (Vietnamese -> ASCII so the GLCD font can show it) */
  spr.setTextFont(1);
  spr.setCursor(bx + 44, by + 30);
  char st[NAV_MAX_STREET];
  stripViet(st, man->street, sizeof st);
  spr.print(st);
}

/* ---- nav overlay: route polyline + position marker + maneuver HUD ---- */
void ui_draw_nav_overlay(LGFX_Sprite &spr, double clon, double clat, int zoom)
{
  const NavRoute *rt   = navGetRoute();
  const NavPos   *pos  = navGetPos();
  const NavManeuver *man = navGetManeuver();

  double tlX = lon2wx(clon, zoom) - SCREEN_W / 2.0;
  double tlY = lat2wy(clat, zoom) - SCREEN_H / 2.0;

  /* 1) route polyline (thick blue) */
  if (rt && rt->count >= 2) {
    int px = 0, py = 0;
    for (int i = 0; i < rt->count; i++) {
      int sx = (int)lround(lon2wx(rt->pts[i].lon, zoom) - tlX);
      int sy = (int)lround(lat2wy(rt->pts[i].lat, zoom) - tlY);
      if (i > 0) {
        spr.drawLine(px, py, sx, sy, TFT_BLUE);
        spr.drawLine(px + 1, py, sx + 1, sy, TFT_BLUE);
        spr.drawLine(px, py + 1, sx, sy + 1, TFT_BLUE);
      }
      px = sx; py = sy;
    }
  }

  /* 2) current position marker */
  if (pos && pos->valid) {
    int sx = (int)lround(lon2wx(pos->lon, zoom) - tlX);
    int sy = (int)lround(lat2wy(pos->lat, zoom) - tlY);
    spr.fillCircle(sx, sy, 6, TFT_RED);
    spr.drawCircle(sx, sy, 6, TFT_WHITE);
  }

  /* 3) maneuver HUD */
  if (man && man->valid) drawManeuverHUD(spr, man);
}

/* ---- corner SCAN toggle button ---- */
static void drawScanButton() {
  uint16_t bg = g_showScan ? display.color565(255, 220, 0) : display.color565(80, 80, 80);
  display.fillRect(SCAN_BTN_X, SCAN_BTN_Y, SCAN_BTN_W, SCAN_BTN_H, bg);
  display.setTextColor(TFT_BLACK, bg);
  display.setTextFont(1);
  display.setCursor(SCAN_BTN_X + 6, SCAN_BTN_Y + 6);
  display.print("SCAN");
}

/* ---- tile-source mode button (below SCAN) ---- */
static void drawModeButton() {
  uint16_t bg = display.color565(60, 60, 90);
  display.fillRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H, bg);
  display.drawRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W - 1, MODE_BTN_H - 1, TFT_WHITE);
  display.setTextColor(TFT_WHITE, bg);
  display.setTextFont(1);
  display.setCursor(MODE_BTN_X + 8, MODE_BTN_Y + 6);
  display.print(map_mode_label());
}

/* ---- zoom in/out buttons (bottom-right) ---- */
static void drawZoomButtons() {
  uint16_t grey = display.color565(80, 80, 80);
  /* "+" */
  display.fillRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W, ZOOM_BTN_H, grey);
  display.drawRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  display.fillRect(ZOOM_IN_X + 8,  ZOOM_IN_Y + 17, ZOOM_BTN_W - 16, 6, TFT_BLACK);
  display.fillRect(ZOOM_IN_X + 18, ZOOM_IN_Y + 7,  6, ZOOM_BTN_H - 14, TFT_BLACK);
  /* "-" */
  display.fillRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W, ZOOM_BTN_H, grey);
  display.drawRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  display.fillRect(ZOOM_OUT_X + 8, ZOOM_OUT_Y + 17, ZOOM_BTN_W - 16, 6, TFT_BLACK);
}

void ui_draw_buttons(void)
{
    drawScanButton();
    drawModeButton();
    drawZoomButtons();
}

/* ---- BLE scan overlay: right panel listing discovered devices ---- */
void ui_draw_scan_overlay(void)
{
  int x = SCREEN_W - 158, y = 0, w = 158, h = SCREEN_H;
  uint16_t navy = display.color565(0, 0, 128);
  display.fillRect(x, y, w, h, navy);
  display.drawRect(x, y, w - 1, h - 1, TFT_WHITE);
  display.setTextColor(TFT_WHITE, navy);
  display.setTextFont(2);
  display.setCursor(x + 6, y + 4);
  display.printf("BLE %d", bleScanCount());

  int ry = y + 26;
  display.setTextFont(1);
  int n = bleScanCount();
  for (int i = 0; i < n && i < 12; i++) {
    const ScanDev *d = bleScanGet(i);
    if (!d) break;                            /* list was cleared mid-draw */
    display.fillRect(x + 2, ry, w - 4, 18, TFT_BLACK);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.setCursor(x + 4, ry + 2);
    display.print(d->haveName ? d->name : d->addr);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(x + 4, ry + 10);
    display.printf("%s %d dBm", d->addr, d->rssi);
    ry += 19;
  }
}
