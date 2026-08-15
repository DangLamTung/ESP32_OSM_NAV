/**
 * ui_hud.cpp — navigation HUD drawing (ui_hud.h).
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#include "ui_hud.h"
#include "ui_icon_cache.h"   /* ui_icon_push, ui_draw_arrow_icon */
#include "ui_settings.h"     /* ui_nav_mode, ui_nav_mode_label, UI_MODE_* */
#include "app_config.h"      /* SCREEN_W/H, NAV_MAX_* */
#include "display_panel.h"   /* display (font selftest sprite parent) */
#include "mercator.h"        /* lon2wx / lat2wy */
#include "ble_nav.h"         /* navGet* navigation state */
#include "map_view.h"        /* map_rotation(), map_ref_lon/lat() */
#include "nav_font_vn.h"     /* FontVN (Vietnamese unifont) — only TU with the font data */
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"

/* Smooth arrow heading: the GPS POS frame arrives ~1 Hz, so drawing the arrow
 * at the raw heading each time makes it JUMP between fixes. Instead we keep the
 * currently-drawn heading and ease it toward the target every redraw (the loop
 * redraws far faster than GPS), so the arrow glides smoothly like a real car.
 * Angles are kept in [0,360); s_arrowValid guards the very first fix. */
static float  s_arrowHdg   = 0.0f;
static bool   s_arrowValid = false;
#define ARROW_EASE_FACTOR  0.22f   /* per-redraw fraction toward the target */
#define ARROW_DEADBAND_DEG 3.0f    /* ignore heading wobble < 3deg (no arrow jitter while the map holds) */

/* ================= next-movement banner arrow =================
 * Green rounded badge with a white turn arrow (like the navbridge back-arrow):
 * straight up for straight/arrive, bent left/right for turns, left for u-turn.
 */
static void drawBannerArrow(LGFX_Sprite &dst, int cx, int cy, const char *man, float scale = 0.7f) {
  /* White Material turn arrow (banner uses 0.7f; the 2-step HUD passes bigger). */
  const char *icon = "straight";
  if      (!strcmp(man, "left"))         icon = "turn_left";
  else if (!strcmp(man, "slight-left"))  icon = "turn_slight_left";
  else if (!strcmp(man, "right"))        icon = "turn_right";
  else if (!strcmp(man, "slight-right")) icon = "turn_slight_right";
  else if (!strcmp(man, "u-turn"))       icon = "u_turn";
  ui_icon_push(icon, &dst, cx, cy, scale);
}

/* Build the guidance line: show the FULL street name. The turn arrow badge
 * already shows the maneuver, so the verbose "Turn left onto" / "Stay on"
 * prefixes are dropped — they were eating the banner width and forcing the
 * street name to be truncated. */
static void buildInstruction(const NavManeuver *man, char *out, size_t n) {
  const char *street = man->street; /* raw UTF-8 — rendered by FontVN */
  const char *m = man->maneuver;
  if (street[0]) {
    snprintf(out, n, "%s", street);
    return;
  }
  /* no street info — fall back to a short maneuver label */
  const char *lbl = "Straight";
  if      (!strcmp(m, "left"))         lbl = "Turn left";
  else if (!strcmp(m, "right"))        lbl = "Turn right";
  else if (!strcmp(m, "slight-left"))  lbl = "Slight left";
  else if (!strcmp(m, "slight-right")) lbl = "Slight right";
  else if (!strcmp(m, "u-turn"))       lbl = "U-turn";
  else if (!strcmp(m, "roundabout"))   lbl = "Roundabout";
  else if (!strcmp(m, "arrive"))       lbl = "Arrive";
  snprintf(out, n, "%s", lbl);
}

/* ---- weather: procedural icon + compact widget in its own box (top-right,
 *      below the 100px guidance panel, so it never overlaps the guidance) ---- */
static void drawWeatherIcon(LGFX_Sprite &spr, int cx, int cy, int code)
{
  const uint16_t SUN   = spr.color565(255, 214, 70);
  const uint16_t CLOUD = spr.color565(214, 218, 224);
  const uint16_t RAIN  = spr.color565(120, 180, 255);
  const uint16_t BOLT  = spr.color565(255, 200, 40);

  int type;   /* classify WMO weather code -> icon type */
  if (code == 0) type = 0;                                      /* clear */
  else if (code <= 2) type = 1;                                 /* partly cloudy */
  else if (code <= 3) type = 2;                                 /* overcast */
  else if (code <= 48) type = 3;                                /* fog */
  else if (code <= 67 || (code >= 80 && code <= 82)) type = 4;  /* rain */
  else if (code <= 77) type = 5;                                /* snow */
  else type = 6;                                                /* thunderstorm */

  if (type == 0 || type == 1) {                                 /* sun */
    spr.fillCircle(cx, cy, 7, SUN);
    for (int a = 0; a < 8; a++) {
      float r = a * M_PI / 4.0f;
      int x0 = cx + (int)roundf(11.0f * cosf(r));
      int y0 = cy + (int)roundf(11.0f * sinf(r));
      int x1 = cx + (int)roundf(16.0f * cosf(r));
      int y1 = cy + (int)roundf(16.0f * sinf(r));
      spr.drawLine(x0, y0, x1, y1, SUN);
    }
  }
  if (type >= 1) {                                              /* cloud */
    spr.fillCircle(cx - 1, cy + 1, 8, CLOUD);
    spr.fillCircle(cx - 6, cy + 3, 6, CLOUD);
    spr.fillCircle(cx + 5, cy + 3, 6, CLOUD);
    spr.fillRect(cx - 10, cy + 2, 21, 6, CLOUD);
    if (type == 1) spr.fillCircle(cx - 1, cy + 1, 6, SUN);      /* sun peeking */
  }
  if (type == 4) {                                              /* rain */
    for (int i = 0; i < 3; i++) {
      int dx = cx - 5 + i * 5;
      spr.drawLine(dx, cy + 9, dx - 2, cy + 14, RAIN);
    }
  } else if (type == 5) {                                       /* snow */
    for (int i = 0; i < 3; i++) spr.fillCircle(cx - 5 + i * 5, cy + 11, 2, TFT_WHITE);
  } else if (type == 6) {                                       /* thunderstorm */
    spr.fillTriangle(cx, cy + 8, cx + 7, cy + 8, cx + 2, cy + 14, BOLT);
    spr.fillTriangle(cx + 3, cy + 10, cx + 9, cy + 10, cx + 6, cy + 16, BOLT);
  } else if (type == 3) {                                       /* fog */
    spr.drawLine(cx - 8, cy + 9, cx + 8, cy + 9, CLOUD);
    spr.drawLine(cx - 6, cy + 12, cx + 6, cy + 12, CLOUD);
  }
}

static void drawWeatherWidget(LGFX_Sprite &spr)
{
  const NavWeather *wx = navGetWeather();
  if (!wx || !wx->valid) return;

  const uint16_t BG = 0x2104;                  /* dark graphite (matches banner) */
  const int w = 98, h = 40;
  const int bx = SCREEN_W - w - 6;
  const int by = 50;                           /* below the 46px guidance strip */
  spr.fillRoundRect(bx, by, w, h, 10, BG);
  spr.drawRoundRect(bx, by, w, h, 10, 0x39E7); /* subtle border */

  drawWeatherIcon(spr, bx + 18, by + 18, wx->code);

  spr.setTextColor(TFT_WHITE, BG);
  spr.setTextFont(2);
  spr.setCursor(bx + 36, by + 7);
  spr.print(wx->tempC);
  spr.print("C");
  spr.setTextFont(1);
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(bx + 36, by + 27);
  spr.print(wx->humidity);
  spr.print("%");
}

/* ---- speed-camera alert: amber badge with a camera glyph + distance.
 *      Drawn when the phone app announces a camera ahead (BLE 0x09). ---- */
static void drawCameraWidget(LGFX_Sprite &spr, int cx, int cy)
{
  const NavCamera *cam = navGetCamera();
  if (!cam || !cam->valid) return;

  const uint16_t AMBER = spr.color565(255, 176, 0);
  const uint16_t DARK  = 0x2104;

  char label[16];
  snprintf(label, sizeof label, "%s", cam->type == 1 ? "MOBILE CAM" : "CAMERA");
  char dist[16];
  snprintf(dist, sizeof dist, "%d m", cam->dist);

  spr.setFont(&FontVN);
  spr.setTextSize(1);
  int wl = spr.textWidth(label);
  int wd = spr.textWidth(dist);
  int w  = wl + wd + 34;                 /* glyph + paddings */
  int x0 = cx - w / 2, y0 = cy - 13;
  if (x0 < 4) x0 = 4;

  /* amber warning badge */
  spr.fillRoundRect(x0, y0, w, 26, 6, AMBER);
  spr.drawRoundRect(x0, y0, w, 26, 6, TFT_WHITE);

  /* camera glyph: dark body + viewfinder + lens */
  int gx = x0 + 14, gy = y0 + 13;
  spr.fillRoundRect(gx - 7, gy - 4, 14, 9, 2, DARK);
  spr.fillRect(gx - 1, gy - 6, 2, 2, DARK);        /* viewfinder */
  spr.fillCircle(gx + 1, gy, 3, DARK);             /* lens body */
  spr.drawCircle(gx + 1, gy, 3, TFT_WHITE);        /* lens ring */

  spr.setTextColor(TFT_BLACK, AMBER);
  spr.setCursor(x0 + 24, y0 + 7);
  spr.print(label);
  spr.setTextColor(0x5A3800, AMBER);
  spr.setCursor(x0 + w - 4 - wd, y0 + 7);
  spr.print(dist);
  spr.setTextFont(1);
}

/* ---- bottom bar: speed / time / ETA strip ---- */
static void drawBottomBar(LGFX_Sprite &spr) {
  const NavPos   *pos  = navGetPos();
  const NavEta   *eta  = navGetEta();
  const NavClock *clk  = navGetClock();

  bool hasPos = (pos && pos->valid);
  bool hasClk = (clk && clk->valid);
  bool hasEta = (eta && eta->valid);
  if (!hasPos && !hasClk && !hasEta) return;   /* only show when there is data */

  const uint16_t BG = 0x2104;                /* dark graphite (black-ish) */
  /* Keep the pill clear of the zoom-button column (ZOOM_*_X = SCREEN_W - 50 =
   * 270). bw=216 centers it at x52..268, so it never covers the "-" button
   * (was bw=248 -> x36..284, which overlapped the zoom-out button). */
  int bw = 216, bh = 32;
  int bx = (SCREEN_W - bw) / 2;              /* centered pill */
  int by = SCREEN_H - bh - 8;
  spr.fillRoundRect(bx, by, bw, bh, 16, BG);
  spr.setTextColor(TFT_WHITE, BG);

  if (ui_nav_mode() == UI_MODE_SIMPLE) {
    /* simple: one consistent Font2 row — speed | time | ETA */
    spr.setTextFont(2);
    if (hasPos) { spr.setCursor(bx + 16, by + 8); spr.print(pos->spd); spr.print("k"); }
    if (hasClk) {
      spr.setCursor(bx + 84, by + 8);
      if (clk->hour < 10) spr.print("0");
      spr.print(clk->hour); spr.print(":");
      if (clk->minute < 10) spr.print("0");
      spr.print(clk->minute);
    }
    if (hasEta) {
      spr.setCursor(bx + 140, by + 8);
      spr.print("E ");
      if (eta->hour < 10) spr.print("0");
      spr.print(eta->hour); spr.print(":");
      if (eta->minute < 10) spr.print("0");
      spr.print(eta->minute);
    }
    spr.setTextFont(1);
    return;
  }

  /* full mode: speed-limit & speedometer icons, then clock + ETA BOTH Font2
   * (same size, so they line up and read consistently). */
  spr.setTextFont(1);
  if (hasPos) {
    bool hasSign = (pos->limit > 0);
    if (hasSign) {
      int lx = bx + 24, ly = by + 16;
      /* red-ring road-sign PNG (Material-style); number overlaid in the centre */
      ui_icon_push("speed_limit", &spr, lx, ly, 0.65f);
      int v = pos->limit;
      spr.setTextColor(TFT_BLACK);
      if (v >= 100) { spr.setTextFont(1); spr.setCursor(lx - 12, ly - 4); }
      else          { spr.setTextFont(2); spr.setCursor(lx - (v >= 10 ? 10 : 5), ly - 8); }
      spr.print(v);
      spr.setTextFont(1);
      spr.setTextColor(TFT_WHITE, BG);
    }
    /* speedometer icon + live speed (km/h) */
    ui_icon_push("speed", &spr, hasSign ? bx + 46 : bx + 18, by + 16, 0.4f);
    spr.setCursor(hasSign ? bx + 56 : bx + 30, by + 12);
    spr.print(pos->spd);                       /* km/h implied (Vietnam) */
  }

  /* current time — Font2 */
  if (hasClk) {
    spr.setTextFont(2);
    spr.setCursor(bx + 92, by + 8);
    if (clk->hour < 10) spr.print("0");
    spr.print(clk->hour); spr.print(":");
    if (clk->minute < 10) spr.print("0");
    spr.print(clk->minute);
  }
  /* ETA — SAME Font2 as the clock (previously Font1 → inconsistent size) */
  if (hasEta) {
    spr.setTextFont(2);
    spr.setCursor(bx + 140, by + 8);
    spr.print("ETA ");
    if (eta->hour < 10) spr.print("0");
    spr.print(eta->hour); spr.print(":");
    if (eta->minute < 10) spr.print("0");
    spr.print(eta->minute);
  }
  spr.setTextFont(1);
  spr.setTextColor(TFT_WHITE, BG);
}

/* Draw the full route the client sends (matches firmware NAV_MAX_ROUTE_POINTS
 * = 256). A plain loop of bounded-per-call wide-line segments is fine on the
 * 8KB loopTask stack; keep an eye on the stack-high-water log if routes get
 * much longer. */
#define MAX_ROUTE_DRAW_PTS 256

/* ---- nav route: drawn on the NORTH-UP world sprite so it rotates with the
 *      map. `spr` (mapWorld) is centered on (refLon,refLat) — the point the
 *      world was last composed at (map_ref_lon/lat) — so the route aligns with
 *      the tiles even while the view scrolls via the blit offset. Called only
 *      when the world is recomposed so the route can't ghost. The car marker
 *      is drawn separately, screen-fixed (ui_draw_nav_marker). ---- */
void ui_draw_nav_route(LGFX_Sprite &spr, double refLon, double refLat, int zoom)
{
  const NavRoute *rt   = navGetRoute();

  double tlX = lon2wx(refLon, zoom) - spr.width() / 2.0;
  double tlY = lat2wy(refLat, zoom) - spr.height() / 2.0;

  /* helper: draw a route polyline as a thick lane. */
  auto drawPoly = [&](const NavRoute *r, float casing, float coreW,
                      uint16_t coreCol, uint16_t casingCol) {
    if (!r || r->count < 2) return;
    int px = 0, py = 0;
    int n = r->count;
    if (n > MAX_ROUTE_DRAW_PTS) n = MAX_ROUTE_DRAW_PTS;
    for (int i = 0; i < n; i++) {
      int sx = (int)lround(lon2wx(r->pts[i].lon, zoom) - tlX);
      int sy = (int)lround(lat2wy(r->pts[i].lat, zoom) - tlY);
      if (i > 0) {
        spr.drawWideLine(px, py, sx, sy, casing, casingCol);
        spr.drawWideLine(px, py, sx, sy, coreW, coreCol);
      }
      px = sx; py = sy;
    }
  };

  /* 0) route CONTINUATION drawn first (faint grey) — where the road goes next,
   *    beyond the near path-ahead. The bright near route draws over it. */
  const NavRoute *rtc = navGetRouteCont();
  if (rtc && rtc->count >= 2) {
    ESP_LOGI("ui", "route-cont draw: %d pts", rtc->count);
    drawPoly(rtc, 2.0f, 1.2f, spr.color565(0xB0, 0xB8, 0xC4),
             spr.color565(0x80, 0x88, 0x94));
  }

  /* 1) route as a thick "lane" (navy casing + bright blue core), ~street width */
  if (rt && rt->count >= 2) {
    uint16_t core = spr.color565(0x1A, 0x73, 0xE8);   /* bright Google blue */
    int px = 0, py = 0;
    int n = rt->count;
    if (n > MAX_ROUTE_DRAW_PTS) n = MAX_ROUTE_DRAW_PTS;   /* small path ahead only */
    ESP_LOGI("ui", "route draw: %d pts, first screen=(%d,%d) zoom=%d", n,
             (int)lround(lon2wx(rt->pts[0].lon, zoom) - tlX),
             (int)lround(lat2wy(rt->pts[0].lat, zoom) - tlY), zoom);
    for (int i = 0; i < n; i++) {
      int sx = (int)lround(lon2wx(rt->pts[i].lon, zoom) - tlX);
      int sy = (int)lround(lat2wy(rt->pts[i].lat, zoom) - tlY);
      if (i > 0) {
        /* Thick lane via wide lines: casing (5 px) then core (3 px). One call
         * per segment instead of the old 18 drawLine offset-loop -> far less
         * PSRAM traffic during the redraw burst, so core 0's BLE controller
         * ISR isn't starved (was tripping "Interrupt wdt timeout on CPU0"). */
        spr.drawWideLine(px, py, sx, sy, 2.5f, TFT_NAVY);
        spr.drawWideLine(px, py, sx, sy, 1.5f, core);
      }
      px = sx; py = sy;
    }
  }

}

/* ---- car marker: screen-fixed, drawn AFTER map_render() ----
 * The view is always centered on the car, so the arrow goes at the sprite
 * center. Its screen rotation = eased GPS heading + the map's own rotation
 * (up in heading-up mode, along the road in north-up — same as drawing it on
 * the world then rotating). map_render() fully overwrites mapSprite each
 * frame, so this never leaves a ghost. */
void ui_draw_nav_marker(LGFX_Sprite &spr)
{
  const NavPos *pos = navGetPos();
  if (!pos || !pos->valid) return;

  int sx = spr.width() / 2, sy = spr.height() / 2;
  spr.drawCircle(sx, sy, 18, TFT_WHITE);   /* contrast ring */
  /* Ease the drawn heading toward the target GPS heading (shortest-path
   * wrap) so the arrow rotates smoothly instead of jumping every fix. */
  float target = (float)pos->hdg;
  if (!s_arrowValid) {
    s_arrowHdg   = target;
    s_arrowValid = true;
  } else {
    float d = (float)fmod((double)(target - s_arrowHdg + 540.0f), 360.0f) - 180.0f;
    /* deadband: hold the arrow still for small heading wobble so it doesn't
     * shimmer while the map is otherwise static */
    if (fabsf(d) > ARROW_DEADBAND_DEG)
      s_arrowHdg = fmod((double)(s_arrowHdg + d * ARROW_EASE_FACTOR + 360.0f), 360.0f);
  }
  ui_draw_arrow_icon(spr, sx, sy, s_arrowHdg + map_rotation(), 1.0f);
}

/* compact text label for a maneuver — used in SIMPLE (text-only) mode so the
 * turn is clear without the arrow icon */
static const char *maneuverText(const char *m) {
  if      (!strcmp(m, "left"))         return "Turn left";
  else if (!strcmp(m, "right"))        return "Turn right";
  else if (!strcmp(m, "slight-left"))  return "Slight left";
  else if (!strcmp(m, "slight-right")) return "Slight right";
  else if (!strcmp(m, "u-turn"))       return "U-turn";
  else if (!strcmp(m, "roundabout"))   return "Roundabout";
  else if (!strcmp(m, "arrive"))       return "Arrive";
  return "Go straight";
}

/* ---- next + next-next guidance, drawn over the FULL map ----
 * A THIN top strip (like a classic nav banner) so it barely covers the map:
 * the next-maneuver arrow badge on the LEFT, the street on the top line and
 * "dist - then <next-next>" on the bottom line. Both lines use the Vietnamese
 * font so accents render correctly (Font1 is ASCII-only). Uses
 * navGetManeuver() / navGetManeuver2() (BLE 0x03/0x08). */
static void drawTwoStepHUD(LGFX_Sprite &spr)
{
  const NavManeuver *n1 = navGetManeuver();
  if (!n1 || !n1->valid) return;
  const NavManeuver *n2 = navGetManeuver2();

  const uint16_t BG = 0x2104;                 /* strip background */
  const int ph = 46;                           /* thin guidance strip */
  spr.fillRect(0, 0, SCREEN_W, ph, BG);
  spr.drawRect(0, 0, SCREEN_W - 1, ph, 0x39E7);

  /* 1) next-maneuver arrow badge on the LEFT (FULL mode's guidance strip) */
  drawBannerArrow(spr, 32, ph / 2, n1->maneuver, 0.8f);

  /* 2) next street name (full, Vietnamese) — top line */
  char instr[NAV_MAX_STREET + 24];
  buildInstruction(n1, instr, sizeof instr);
  spr.setTextColor(TFT_WHITE, BG);
  spr.setFont(&FontVN);
  spr.setTextSize(1);
  spr.setCursor(56, 3);
  spr.print(instr);

  /* 3) distance - then next-next street — bottom line, SAME Vietnamese font */
  char line[NAV_MAX_STREET * 2 + 40];
  int used = 0;
  if (n1->dist > 0) used = snprintf(line, sizeof line, "%d m", n1->dist);
  else              line[0] = 0;
  if (n2 && n2->valid) {
    const char *s2 = n2->street[0] ? n2->street : n2->maneuver;
    used = snprintf(line + used, sizeof line - used,
                    "%s%s", used ? " - then " : "then ", s2);
    if (n2->dist > 0)
      snprintf(line + used, sizeof line - used, " (%d m)", n2->dist);
  }
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(56, 27);
  spr.print(line);

  /* tiny mode tag (top-right of the strip) so toggling NAV mode is clearly
   * visible — "FULL" vs "SIMPLE" flips right here when the button is tapped */
  spr.setTextFont(1);
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(SCREEN_W - 52, 4);
  spr.print(ui_nav_mode_label());
  spr.setTextSize(1);
}

/* ---- screen-fixed HUD: maneuver banner + weather + bottom bar.
 *      Drawn onto the visible sprite AFTER map_render() so these stay upright
 *      while the map turns beneath them. ---- */
/* ---- SIMPLE mode: text-only navigation screen (NO map) ----
 * Fills the whole screen: time/ETA up top, a BIG turn arrow in the middle
 * (same Material icons as the FULL banner — "arrow in the middle" the user
 * asked for), the maneuver + street, distance, next-next, and speed. EVERY
 * line uses FontVN so Vietnamese accents render (Font1/Font2 are ASCII-only).
 * drawMap() does NOT fetch or render the map in this mode — the route is
 * still cached. */
static void drawTextOnlyHUD(LGFX_Sprite &spr)
{
  const uint16_t BG = 0x2104;
  spr.fillScreen(BG);

  const NavManeuver *n1 = navGetManeuver();
  const NavPos   *pos = navGetPos();
  const NavClock *clk = navGetClock();
  const NavEta   *eta = navGetEta();

  spr.setFont(&FontVN);
  spr.setTextSize(1);

  /* top line: time - ETA (ASCII separators — some non-Latin-1 glyphs like
   * the middle-dot/em-dash render as boxes in the unifont subset) */
  char top[64] = "";
  if (clk && clk->valid) snprintf(top, sizeof top, "%02d:%02d", clk->hour, clk->minute);
  if (eta && eta->valid) { int n = (int)strlen(top); snprintf(top + n, sizeof top - n, "  -  ETA %02d:%02d", eta->hour, eta->minute); }
  spr.setTextColor(0xCED8, BG);
  if (top[0]) { int w = spr.textWidth(top); spr.setCursor((SCREEN_W - w) / 2, 6); spr.print(top); }

  /* weather widget, top-right (below the time/ETA line, clear of the arrow) */
  drawWeatherWidget(spr);

  /* speed-camera alert (if announced ahead), centred above the big arrow */
  drawCameraWidget(spr, SCREEN_W / 2, 28);

  if (n1 && n1->valid) {
    /* BIG next-maneuver arrow, dead centre of the screen (40px icon * 1.6) */
    drawBannerArrow(spr, SCREEN_W / 2, 74, n1->maneuver, 1.6f);

    /* maneuver + street, big text under the arrow */
    char big[NAV_MAX_STREET + 40];
    const char *mv = maneuverText(n1->maneuver);
    bool showMv = (strcmp(n1->maneuver, "straight") != 0) && (strcmp(n1->maneuver, "arrive") != 0);
    if (n1->street[0])
      snprintf(big, sizeof big, "%s%s%s", showMv ? mv : "", showMv ? " - " : "", n1->street);
    else
      snprintf(big, sizeof big, "%s", mv);
    spr.setTextColor(TFT_WHITE, BG);
    int w = spr.textWidth(big);
    spr.setCursor((SCREEN_W - w) / 2, 124);
    spr.print(big);

    /* distance */
    if (n1->dist > 0) {
      char d[16]; snprintf(d, sizeof d, "%d m", n1->dist);
      spr.setTextColor(0xCED8, BG);
      int w2 = spr.textWidth(d);
      spr.setCursor((SCREEN_W - w2) / 2, 146);
      spr.print(d);
    }

    /* next-next: small arrow + "then <street> - dist" on ONE centred line */
    const NavManeuver *n2 = navGetManeuver2();
    if (n2 && n2->valid) {
      char nn[NAV_MAX_STREET + 40];
      const char *s2 = n2->street[0] ? n2->street : maneuverText(n2->maneuver);
      snprintf(nn, sizeof nn, "then %s", s2);
      if (n2->dist > 0) { char d2[24]; snprintf(d2, sizeof d2, "  -  %d m", n2->dist); strncat(nn, d2, sizeof nn - strlen(nn) - 1); }
      spr.setTextColor(0xCED8, BG);
      int w3 = spr.textWidth(nn);
      const int arrowW = 20, gap = 6;   /* 40px icon * 0.5 + spacer */
      int groupW = w3 + arrowW + gap;
      int gx = (SCREEN_W - groupW) / 2;
      drawBannerArrow(spr, gx + arrowW / 2, 178, n2->maneuver, 0.5f);
      spr.setCursor(gx + arrowW + gap, 170);
      spr.print(nn);
    }
  }

  /* bottom: red speed-limit sign + speedometer + live speed (centred group) */
  if (pos && pos->valid) {
    const int by = 204;
    bool hasSign = (pos->limit > 0);
    int scx  = SCREEN_W / 2 - 26;                  /* red-ring sign centre */
    int spdX = SCREEN_W / 2 + (hasSign ? 10 : 0);  /* speedometer centre   */
    if (hasSign) {
      ui_icon_push("speed_limit", &spr, scx, by + 8, 0.65f);
      int v = pos->limit;
      spr.setTextColor(TFT_BLACK);
      if (v >= 100) { spr.setTextFont(1); spr.setCursor(scx - 12, by + 4); }
      else          { spr.setTextFont(2); spr.setCursor(scx - (v >= 10 ? 10 : 5), by); }
      spr.print(v);
      spr.setTextFont(1);
      spr.setTextColor(TFT_WHITE, BG);
    }
    ui_icon_push("speed", &spr, spdX, by + 8, 0.4f);
    spr.setTextFont(2);
    spr.setTextColor(TFT_WHITE, BG);
    spr.setCursor(spdX + 10, by + 4);
    spr.print(pos->spd);
    spr.setTextFont(1);
  }
  spr.setTextFont(1);
}

void ui_draw_nav_hud(LGFX_Sprite &spr)
{
  /* SIMPLE = text-only navigation screen (no map). */
  if (ui_nav_mode() == UI_MODE_SIMPLE) {
    drawTextOnlyHUD(spr);
    return;
  }

  /* FULL: guidance strip + weather + bottom bar over the map */
  const NavManeuver *man = navGetManeuver();
  if (man && man->valid) drawTwoStepHUD(spr);
  drawWeatherWidget(spr);
  drawBottomBar(spr);

  /* speed-camera alert (if announced), top-left under the guidance strip */
  drawCameraWidget(spr, 100, 62);
}

/* ===== DEBUG (temp): verify FontVN glyph coverage. Renders each Vietnamese
 * char + the em-dash separator individually and reports ink + bounding box:
 * ink≈0 => glyph missing from the font. NOTE: uses ESP_LOGI (Serial.* does not
 * reach the host on this USB-Serial/JTAG board). Kept here (ui_hud) so the
 * embedded font data lives in only one translation unit. ===== */
void ui_font_selftest(void)
{
    static const char *chars = "ệộạồừĐễợưươớáéíóúýàèìòùâêôơậặằẵăń–—·Ag";
    LGFX_Sprite spr(&display);
    spr.setPsram(true);
    spr.setColorDepth(16);
    spr.createSprite(48, 32);   /* taller + wider: unifont is 16px, don't clip */
    spr.setFont(&FontVN);
    spr.setTextSize(1);
    spr.setTextColor(0xFFFF, 0x0000);
    ESP_LOGI("font", "SELFTEST start (48x32, cursor 2,26)");
    for (const char *p = chars; *p; ) {
        /* decode one UTF-8 char */
        uint8_t c = (uint8_t)*p;
        int len = 1;
        if      (c >= 0xF0) { len = 4; }
        else if (c >= 0xE0) { len = 3; }
        else if (c >= 0xC0) { len = 2; }
        char one[8] = {0};
        memcpy(one, p, len);
        p += len;

        spr.fillSprite(0x0000);
        spr.setCursor(2, 26);
        spr.print(one);
        uint32_t ink = 0; int minx = 99, miny = 99, maxx = -1, maxy = -1;
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 48; x++)
                if (spr.readPixelValue(x, y) != 0x0000) {
                    ink++;
                    if (x < minx) { minx = x; }
                    if (x > maxx) { maxx = x; }
                    if (y < miny) { miny = y; }
                    if (y > maxy) { maxy = y; }
                }
        ESP_LOGI("font", "SELFTEST %s ink=%u box=(%d,%d)-(%d,%d)",
                 one, (unsigned)ink, minx, miny, maxx, maxy);
        /* ASCII-art shape dump for a few key chars so we can SEE the glyph */
        if (!strcmp(one, "ệ") || !strcmp(one, "ạ") || !strcmp(one, "A")) {
            for (int y = 0; y < 32; y++) {
                char row[49];
                for (int x = 0; x < 48; x++)
                    row[x] = (spr.readPixelValue(x, y) != 0x0000) ? '#' : '.';
                row[48] = 0;
                ESP_LOGI("font", "SHAPE %s |%s|", one, row);
            }
        }
    }
    spr.deleteSprite();
    ESP_LOGI("font", "SELFTEST end");
}
