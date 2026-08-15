/**
 * ui_buttons.cpp — corner buttons + settings panel composition (ui_buttons.h).
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#include "ui_buttons.h"
#include "ui_icon_cache.h"   /* ui_icon_push */
#include "ui_settings.h"     /* ui_nav_mode_label, ui_settings_open, ui_brightness */
#include "app_config.h"      /* button geometry */
#include "map_view.h"        /* map_heading_up, map_aa_enabled, map_center_on, ZOOM */
#include "ble_nav.h"         /* navGpsBroadcast, navGetPos */
#include "wifi_net.h"        /* wifi_net_connected/pending/label */
#include <Arduino.h>

/* ---- corner settings (gear) button ---- */
static void drawGearButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(GEAR_BTN_X, GEAR_BTN_Y, GEAR_BTN_W, GEAR_BTN_H, bg);
  dst->drawRect(GEAR_BTN_X, GEAR_BTN_Y, GEAR_BTN_W - 1, GEAR_BTN_H - 1, TFT_BLACK);
  /* white Material settings gear, scaled to the 42x22 button */
  int cx = GEAR_BTN_X + GEAR_BTN_W / 2, cy = GEAR_BTN_Y + GEAR_BTN_H / 2;
  ui_icon_push("gear", dst, cx, cy, 0.5f);
}

/* ---- rotate button (left edge): tap to turn the map ROTATE_STEP_DEG clockwise ---- */
static void drawRotateButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(ROTATE_BTN_X, ROTATE_BTN_Y, ROTATE_BTN_W, ROTATE_BTN_H, bg);
  dst->drawRect(ROTATE_BTN_X, ROTATE_BTN_Y, ROTATE_BTN_W - 1, ROTATE_BTN_H - 1, TFT_BLACK);
  int cx = ROTATE_BTN_X + ROTATE_BTN_W / 2;
  int cy = ROTATE_BTN_Y + ROTATE_BTN_H / 2;
  /* circular "rotate" glyph: ring + clockwise arrowhead at the top */
  const int R = 6;
  dst->drawCircle(cx, cy, R, TFT_BLACK);
  dst->fillTriangle(cx + 6, cy - R, cx, cy - R - 3, cx, cy - R + 3, TFT_BLACK);
  dst->fillCircle(cx, cy, 1, TFT_BLACK);
}

/* ---- heading-up toggle (left edge, under rotate): green + "H" when the map
 *      auto-rotates with GPS heading, grey + "N" when fixed north-up ---- */
static void drawHeadingButton(LovyanGFX *dst) {
  bool on = map_heading_up();
  uint16_t bg = on ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
  dst->fillRect(HDG_BTN_X, HDG_BTN_Y, HDG_BTN_W, HDG_BTN_H, bg);
  dst->drawRect(HDG_BTN_X, HDG_BTN_Y, HDG_BTN_W - 1, HDG_BTN_H - 1, TFT_BLACK);
  int cx = HDG_BTN_X + HDG_BTN_W / 2;
  int cy = HDG_BTN_Y + HDG_BTN_H / 2;
  /* up arrow (travel direction) + mode letter */
  dst->fillTriangle(cx, cy - 5, cx - 4, cy, cx + 4, cy, TFT_WHITE);
  dst->setTextColor(TFT_WHITE, bg);
  dst->setTextFont(1);
  dst->setCursor(cx - 3, cy + 1);
  dst->print(on ? "H" : "N");
}

/* ---- center button (left edge, under heading): snap the view back onto the
 *      car (ui_recenter) — crosshair glyph ---- */
static void drawCenterButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(CENTER_BTN_X, CENTER_BTN_Y, CENTER_BTN_W, CENTER_BTN_H, bg);
  dst->drawRect(CENTER_BTN_X, CENTER_BTN_Y, CENTER_BTN_W - 1, CENTER_BTN_H - 1, TFT_BLACK);
  int cx = CENTER_BTN_X + CENTER_BTN_W / 2;
  int cy = CENTER_BTN_Y + CENTER_BTN_H / 2;
  dst->drawCircle(cx, cy, 5, TFT_BLACK);
  dst->drawLine(cx - 9, cy, cx - 6, cy, TFT_BLACK);
  dst->drawLine(cx + 6, cy, cx + 9, cy, TFT_BLACK);
  dst->drawLine(cx, cy - 9, cx, cy - 6, TFT_BLACK);
  dst->drawLine(cx, cy + 6, cx, cy + 9, TFT_BLACK);
  dst->fillCircle(cx, cy, 1, TFT_BLACK);
}

/* ---- zoom in/out buttons (bottom-right) ---- */
static void drawZoomButtons(LovyanGFX *dst) {
  uint16_t grey  = dst->color565(80, 80, 80);   /* enabled */
  uint16_t dim   = dst->color565(38, 38, 38);   /* disabled bg */
  uint16_t glyph = TFT_BLACK;                       /* enabled glyph */
  uint16_t dimG  = dst->color565(90, 90, 90);    /* disabled glyph */
  bool canIn  = ZOOM < ZOOM_MAX;                    /* + allowed? */
  bool canOut = ZOOM > ZOOM_MIN;                    /* - allowed? */

  /* "+" */
  dst->fillRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W, ZOOM_BTN_H, canIn ? grey : dim);
  dst->drawRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  dst->fillRect(ZOOM_IN_X + 8,  ZOOM_IN_Y + 17, ZOOM_BTN_W - 16, 6, canIn ? glyph : dimG);
  dst->fillRect(ZOOM_IN_X + 18, ZOOM_IN_Y + 7,  6, ZOOM_BTN_H - 14, canIn ? glyph : dimG);

  /* "-" */
  dst->fillRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W, ZOOM_BTN_H, canOut ? grey : dim);
  dst->drawRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  dst->fillRect(ZOOM_OUT_X + 8, ZOOM_OUT_Y + 17, ZOOM_BTN_W - 16, 6, canOut ? glyph : dimG);
}

/* ---- settings panel: WiFi + brightness (drawn over the bottom strip) ---- */
static void ui_draw_settings_panel(LovyanGFX *dst)
{
    const uint16_t panelBG = dst->color565(22, 24, 30);
    dst->fillRect(0, SETTINGS_PANEL_Y, SCREEN_W, SETTINGS_PANEL_H, panelBG);
    dst->drawRect(0, SETTINGS_PANEL_Y, SCREEN_W - 1, SETTINGS_PANEL_H - 1, TFT_WHITE);

    /* WiFi button: green=on, amber=connecting, grey=off */
    bool on      = wifi_net_connected();
    bool pending = wifi_net_pending();
    uint16_t wbg = on      ? dst->color565(40, 150, 70)
                 : pending ? dst->color565(200, 140, 20)
                           : dst->color565(70, 74, 82);
    dst->fillRect(WIFI_BTN_X, WIFI_BTN_Y, WIFI_BTN_W, WIFI_BTN_H, wbg);
    dst->drawRect(WIFI_BTN_X, WIFI_BTN_Y, WIFI_BTN_W - 1, WIFI_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, wbg);
    dst->setTextFont(1);
    dst->setCursor(WIFI_BTN_X + 6, WIFI_BTN_Y + 6);
    dst->print(wifi_net_label());

    /* GPS broadcast toggle (green when broadcasting) */
    bool gpsOn = navGpsBroadcast();
    uint16_t gbg = gpsOn ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
    dst->fillRect(GPS_BTN_X, GPS_BTN_Y, GPS_BTN_W, GPS_BTN_H, gbg);
    dst->drawRect(GPS_BTN_X, GPS_BTN_Y, GPS_BTN_W - 1, GPS_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, gbg);
    dst->setTextFont(1);
    dst->setCursor(GPS_BTN_X + 6, GPS_BTN_Y + 6);
    dst->print(gpsOn ? "GPS BCAST" : "GPS off");

    /* nav HUD mode cycle (2-STEP / FULL / SIMPLE) */
    uint16_t mbg = dst->color565(70, 74, 82);
    dst->fillRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H, mbg);
    dst->drawRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W - 1, MODE_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, mbg);
    dst->setTextFont(1);
    dst->setCursor(MODE_BTN_X + 4, MODE_BTN_Y + 6);
    dst->print("NAV ");
    dst->print(ui_nav_mode_label());

    /* rotation quality: AA (crisp) vs fast (nearest-neighbour) */
    bool aa = map_aa_enabled();
    uint16_t abg = aa ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
    dst->fillRect(AA_BTN_X, AA_BTN_Y, AA_BTN_W, AA_BTN_H, abg);
    dst->drawRect(AA_BTN_X, AA_BTN_Y, AA_BTN_W - 1, AA_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, abg);
    dst->setTextFont(1);
    dst->setCursor(AA_BTN_X + 6, AA_BTN_Y + 6);
    dst->print(aa ? "ROT: CRISP" : "ROT: FAST");

    /* brightness slider */
    dst->setTextColor(TFT_WHITE, panelBG);
    dst->setCursor(SLIDER_X, SLIDER_Y - 12);
    dst->print("Brightness");
    dst->fillRect(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H,
                  dst->color565(60, 62, 70));
    int thumbX = SLIDER_X + (SLIDER_W - 6) * ui_brightness() / 255;
    dst->fillRect(thumbX, SLIDER_Y - 2, 6, SLIDER_H + 4, TFT_WHITE);
}

void ui_draw_buttons(LovyanGFX *dst)
{
    drawRotateButton(dst);
    drawHeadingButton(dst);
    drawCenterButton(dst);
    drawGearButton(dst);
    if (ui_settings_open())
        ui_draw_settings_panel(dst);   /* bottom overlay covers the zoom column */
    else
        drawZoomButtons(dst);
}

/* Snap the view back onto the car (center button). */
void ui_recenter(void)
{
    const NavPos *p = navGetPos();
    if (p && p->valid)
    {
        map_center_on(p->lat, p->lon);
        Serial.println("[ui] recenter on car");
    }
}
