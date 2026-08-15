/**
 * routing.cpp — offline routing over the real OSM road graph (routing.h).
 *
 * The graph is loaded from /sdcard/routing.rng (see route_graph.cpp): either a
 * small whole-file RNG1 region or a window of the whole-country RNG2 file.
 * The ROUTE feature is gated on the OFFLINE RTE setting — when off, the button
 * is disabled and no graph is loaded (fast boot).
 *
 * UI flow (touch):
 *   ROUTE button -> PICK_START -> (tap) -> PICK_STOP -> (tap) -> compute and
 *   draw the magenta path on the map + show run stats (pts/ms/m). Picks are
 *   rejected on control buttons, and the RNG2 window reloads around the current
 *   view so routing works anywhere in the country.
 */
#include "routing.h"
#include "route_graph.h"      /* real OSM road graph (.rng from SD) */
#include "app_config.h"      /* SCREEN_W/H, MAP_CENTER_* */
#include "map_view.h"        /* map_screen_to_latlon, map_force_recompose, map_set_* */
#include "ui_controls.h"     /* ui_mark_redraw() */
#include "mercator.h"        /* lon2wx / lat2wy for world drawing */
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

static const char *TAG = "route";

/* ---- ROUTE button (bottom-left, clear of the bottom-bar pill & zoom col) ---- */
#define ROUTE_BTN_X 6
#define ROUTE_BTN_Y (SCREEN_H - 42)   /* 198 — tall enough to be easy to tap */
#define ROUTE_BTN_W 44
#define ROUTE_BTN_H 36

/* ---- path / scratch buffers ---- */
#define MAX_PATH_PTS 4096

/* start = cyan, stop = yellow (clearly distinct on the map) */
#define ROUTE_COL_START 0x07FF
#define ROUTE_COL_STOP  0xFFE0

static enum { ROUTE_IDLE, ROUTE_PICK_START, ROUTE_PICK_STOP, ROUTE_DONE } s_mode = ROUTE_IDLE;
static bool s_useReal = false;   /* real OSM graph loaded (gated on OFFLINE RTE) */
static uint32_t s_announceMS = 0;   /* show the ±range announcement briefly */

bool routing_active(void) { return s_mode != ROUTE_IDLE; }

static double s_startLat, s_startLon, s_stopLat, s_stopLon;
static int    s_startX = -1, s_startY = -1, s_stopX = -1, s_stopY = -1;

static struct { double lat, lon; } s_path[MAX_PATH_PTS];
static int s_pathN = 0;
static uint32_t s_lastVisited = 0, s_lastMs = 0, s_lastDistM = 0;
static double *s_rlat = NULL, *s_rlon = NULL;   /* real-graph path scratch (PSRAM) */

void routing_init(void)
{
  s_rlat = (double *)heap_caps_malloc(MAX_PATH_PTS * sizeof(double), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  s_rlon = (double *)heap_caps_malloc(MAX_PATH_PTS * sizeof(double), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_rlat) s_rlat = (double *)malloc(MAX_PATH_PTS * sizeof(double));
  if (!s_rlon) s_rlon = (double *)malloc(MAX_PATH_PTS * sizeof(double));
  /* no synthetic grid: the ROUTE feature is gated on the real graph, which is
   * loaded by routing_set_offline() (OFFLINE RTE setting) */
}

/* Enable/disable offline (real-road) routing at runtime. ON loads the graph
 * from SD; OFF unloads it (faster boot, less PSRAM). The ROUTE button is
 * disabled entirely when the real graph isn't active. Called from the
 * settings panel and from setup() with the persisted setting. */
void routing_set_offline(bool on)
{
  if (on == s_useReal)
    return;
  if (on) {
    if (rg_load("/sdcard/routing.rng") && rg_astar_init()) {
      s_useReal = true;
      ESP_LOGI(TAG, "offline routing ON: real graph active (%u nodes)",
               (unsigned)rg_node_count());
    } else {
      rg_unload();
      s_useReal = false;
      s_mode = ROUTE_IDLE;
      ESP_LOGE(TAG, "offline routing ON but graph load failed - ROUTE disabled");
    }
  } else {
    rg_unload();
    s_useReal = false;
    s_mode = ROUTE_IDLE;   /* exit any in-progress route */
    s_pathN = 0;
    ESP_LOGI(TAG, "offline routing OFF - ROUTE disabled");
  }
  ui_mark_redraw();
}

/* ---------------- A* ---------------- */
static bool routing_compute(void)
{
  /* the ROUTE feature only works with the real graph active (OFFLINE RTE on) */
  if (!s_useReal) {
    s_pathN = 0;
    return false;
  }

  /* ---- real OSM road network (from SD) ---- */
  {
    double distM = 0.0;
    uint32_t ms = 0;
    int n = rg_route(s_startLat, s_startLon, s_stopLat, s_stopLon,
                     s_rlat, s_rlon, MAX_PATH_PTS, &distM, &ms);
    s_lastDistM = (uint32_t)lround(distM);
    s_lastMs = ms;
    s_pathN = 0;
    for (int i = 0; i < n && i < MAX_PATH_PTS; i++) {
      s_path[i].lat = s_rlat[i]; s_path[i].lon = s_rlon[i]; s_pathN++;
    }
    s_lastVisited = 0;
    if (s_pathN >= 1) {
      ui_mark_redraw();   /* path drawn screen-fixed in the overlay */
      return true;
    }
    return false;
  }
}

/* Boot self-test: route across the loaded graph window and log the real
 * on-device timing. Skipped when offline routing is off. */
void routing_selftest(void)
{
  if (!s_useReal) {
    ESP_LOGI(TAG, "offline routing off - routing selftest skipped");
    return;
  }
  /* route between two CENTRAL points (35% / 65% across the window) so both
   * ends are in the main connected component — corner-to-corner can hit
   * bbox-truncation islands and report a bogus no-path. */
  double mnla, mnlo, mxla, mxlo;
  rg_bbox(&mnla, &mnlo, &mxla, &mxlo);
  s_startLat = mnla + (mxla - mnla) * 0.35; s_startLon = mnlo + (mxlo - mnlo) * 0.35;
  s_stopLat  = mnla + (mxla - mnla) * 0.65; s_stopLon  = mnlo + (mxlo - mnlo) * 0.65;
  ESP_LOGI(TAG, "selftest: real graph center route (%.4f,%.4f)->(%.4f,%.4f)",
           s_startLat, s_startLon, s_stopLat, s_stopLon);
  routing_compute();
}

/* ---------------- touch ---------------- */
/* Reload the whole-country window centred on the CURRENT map centre so offline
 * routing is correct wherever the user is. Always reloads: the old covers-check
 * used a 0.07deg margin vs a 0.06deg loaded box, so a moderate pan left the
 * view outside the loaded cells -> start snap failed -> no route. */
static void routing_reload_window(void)
{
  if (!s_useReal || !rg_is_windowed())
    return;
  rg_unload();
  if (rg_load("/sdcard/routing.rng") && rg_astar_init()) {
    s_pathN = 0;
    ESP_LOGI(TAG, "window reloaded around (%.4f,%.4f) - %u nodes",
             centerLat, centerLon, (unsigned)rg_node_count());
  } else {
    rg_unload(); s_useReal = false; s_mode = ROUTE_IDLE;
    ESP_LOGE(TAG, "window reload failed - ROUTE disabled");
  }
  ui_mark_redraw();
}

/* true if (x,y) is on a control button — such taps must NOT place a crosshair
 * during pick mode; they fall through to the button handler instead. */
static bool routing_tap_on_button(int x, int y)
{
  if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
      y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) return true;
  if (x >= ROTATE_BTN_X && x < ROTATE_BTN_X + ROTATE_BTN_W &&
      y >= ROTATE_BTN_Y && y < ROTATE_BTN_Y + ROTATE_BTN_H) return true;
  if (x >= HDG_BTN_X && x < HDG_BTN_X + HDG_BTN_W &&
      y >= HDG_BTN_Y && y < HDG_BTN_Y + HDG_BTN_H) return true;
  if (x >= CENTER_BTN_X && x < CENTER_BTN_X + CENTER_BTN_W &&
      y >= CENTER_BTN_Y && y < CENTER_BTN_Y + CENTER_BTN_H) return true;
  if (x >= GEAR_BTN_X && x < GEAR_BTN_X + GEAR_BTN_W &&
      y >= GEAR_BTN_Y && y < GEAR_BTN_Y + GEAR_BTN_H) return true;
  if (x >= ZOOM_IN_X && x < ZOOM_IN_X + ZOOM_BTN_W &&
      y >= ZOOM_IN_Y && y < ZOOM_IN_Y + ZOOM_BTN_H) return true;
  if (x >= ZOOM_OUT_X && x < ZOOM_OUT_X + ZOOM_BTN_W &&
      y >= ZOOM_OUT_Y && y < ZOOM_OUT_Y + ZOOM_BTN_H) return true;
  if (ui_settings_open()) return true;   /* the whole settings panel is controls */
  return false;
}

bool routing_handle_tap(int x, int y)
{
  ESP_LOGI(TAG, "tap(%d,%d) mode=%d", x, y, (int)s_mode);   /* DEBUG: watch taps */
  if (s_mode == ROUTE_IDLE) {
    /* only the ROUTE button is consumed while idle; disabled unless the real
     * graph is active (OFFLINE RTE on) */
    if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
        y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) {
      if (!s_useReal) {
        ESP_LOGI(TAG, "route disabled (offline routing off)");
        return true;   /* consume; do nothing */
      }
      routing_reload_window();   /* pan-to-anywhere support */
      s_mode = ROUTE_PICK_START;
      s_pathN = 0;
      /* routing needs the map: if we are in the text-only SIMPLE screen,
       * switch to FULL so the crosshairs + path are visible */
      if (ui_nav_mode() == UI_MODE_SIMPLE) ui_cycle_nav_mode();
      map_set_heading_up(false);   /* crosshair mapping assumes north-up */
      map_set_rotation(0);
      s_announceMS = millis();
      ESP_LOGI(TAG, "pick start (tap the map)");
      ui_mark_redraw();
      return true;
    }
    return false;
  }

  if (s_mode == ROUTE_PICK_START) {
    if (routing_tap_on_button(x, y)) return false;   /* don't pick on a button */
    s_startX = x; s_startY = y;
    map_screen_to_latlon(x, y, &s_startLat, &s_startLon);
    s_mode = ROUTE_PICK_STOP;
    ESP_LOGI(TAG, "start %.6f,%.6f - pick stop", s_startLat, s_startLon);
    ui_mark_redraw();
    return true;
  }
  if (s_mode == ROUTE_PICK_STOP) {
    if (routing_tap_on_button(x, y)) return false;   /* don't pick on a button */
    s_stopX = x; s_stopY = y;
    map_screen_to_latlon(x, y, &s_stopLat, &s_stopLon);
    ESP_LOGI(TAG, "stop %.6f,%.6f - computing", s_stopLat, s_stopLon);
    routing_compute();   /* A* now; on success we jump straight to DONE */
    s_mode = ROUTE_DONE; /* no confirm dialog: show the path + run stats */
    ui_mark_redraw();
    return true;
  }
  /* ROUTE_DONE: the path PERSISTS (drawn screen-fixed, follows pan/zoom).
   * Tapping ROUTE again starts a fresh route and clears the old path; every
   * other tap falls through so zoom/pan/settings still work — previously ANY
   * tap cleared the path, so zooming in deleted the route. */
  if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
      y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) {
    if (!s_useReal) return true;   /* disabled unless the real graph is active */
    routing_reload_window();   /* pan-to-anywhere support */
    s_mode = ROUTE_PICK_START;
    s_pathN = 0;
    if (ui_nav_mode() == UI_MODE_SIMPLE) ui_cycle_nav_mode();
    map_set_heading_up(false);
    map_set_rotation(0);
    s_announceMS = millis();
    ESP_LOGI(TAG, "pick start (tap the map)");
    ui_mark_redraw();
    return true;
  }
  return false;
}

/* ---------------- drawing ---------------- */
static void drawCrosshair(LGFX_Sprite &spr, int x, int y, uint16_t col) {
  const int L = 14;                      /* arm length */
  for (int i = -1; i <= 1; i++) {        /* 3px thick + sign */
    spr.drawLine(x - L, y + i, x + L, y + i, col);   /* horizontal bar */
    spr.drawLine(x + i, y - L, x + i, y + L, col);   /* vertical bar */
  }
  spr.fillRect(x - 2, y - 2, 5, 5, TFT_WHITE);       /* centre */
}

static void drawStatusAt(LGFX_Sprite &spr, const char *txt, int y) {
  spr.setTextFont(1);
  spr.setTextColor(TFT_WHITE, 0x2104);
  int w = spr.textWidth(txt);
  spr.fillRect(SCREEN_W / 2 - w / 2 - 8, y, w + 16, 15, 0x2104);
  spr.drawRect(SCREEN_W / 2 - w / 2 - 8, y, w + 16, 15, 0x39E7);
  spr.setCursor(SCREEN_W / 2 - w / 2, y + 3);
  spr.print(txt);
}

static void drawStatus(LGFX_Sprite &spr, const char *txt) {
  drawStatusAt(spr, txt, 2);
}

/* draw the computed path + start/stop markers screen-fixed (always visible) */
static void drawPathOverlay(LGFX_Sprite &spr) {
  if (s_pathN >= 2) {
    int px = 0, py = 0;
    for (int i = 0; i < s_pathN; i++) {
      int sx, sy;
      map_latlon_to_screen(s_path[i].lat, s_path[i].lon, &sx, &sy);
      if (i > 0) {
        spr.drawWideLine(px, py, sx, sy, 4.0f, 0xF81F);   /* magenta halo */
        spr.drawWideLine(px, py, sx, sy, 2.0f, TFT_WHITE);/* white core */
      }
      px = sx; py = sy;
    }
  }
  int sx, sy;
  map_latlon_to_screen(s_startLat, s_startLon, &sx, &sy);
  spr.fillCircle(sx, sy, 5, ROUTE_COL_START);
  spr.drawCircle(sx, sy, 7, TFT_WHITE);
  map_latlon_to_screen(s_stopLat, s_stopLon, &sx, &sy);
  spr.fillCircle(sx, sy, 5, ROUTE_COL_STOP);
  spr.drawCircle(sx, sy, 7, TFT_WHITE);
}

void routing_draw_overlay(LGFX_Sprite &spr)
{
  /* ROUTE button (always visible; dimmed + "ROUTE OFF" when the real graph
   * isn't loaded — the feature is fully disabled then) */
  uint16_t bg;
  if (!s_useReal)
    bg = spr.color565(48, 50, 56);
  else
    bg = (s_mode != ROUTE_IDLE) ? spr.color565(40, 150, 70) : spr.color565(70, 74, 82);
  spr.fillRoundRect(ROUTE_BTN_X, ROUTE_BTN_Y, ROUTE_BTN_W, ROUTE_BTN_H, 4, bg);
  spr.drawRoundRect(ROUTE_BTN_X, ROUTE_BTN_Y, ROUTE_BTN_W, ROUTE_BTN_H, 4, TFT_BLACK);
  spr.setTextColor(TFT_WHITE, bg);
  spr.setTextFont(1);
  spr.setCursor(ROUTE_BTN_X + 4, ROUTE_BTN_Y + 7);
  spr.print(s_useReal ? "ROUTE" : "OFF");
  spr.setTextFont(1);

  switch (s_mode) {
  case ROUTE_PICK_START: {
    /* announce the offline window range for a few seconds on entry */
    if (s_announceMS && (millis() - s_announceMS) < 3500) {
      char ann[40];
      snprintf(ann, sizeof ann, "OFFLINE ROUTE MAX ~%d KM", ROUTE_WINDOW_RADIUS_KM);
      drawStatusAt(spr, ann, 2);
      drawStatusAt(spr, "TAP START POINT", 19);
    } else {
      drawStatus(spr, "PICK START");
    }
    return;
  }
  case ROUTE_PICK_STOP:
    drawStatus(spr, "PICK STOP");
    if (s_startX >= 0) drawCrosshair(spr, s_startX, s_startY, ROUTE_COL_START);
    return;
  case ROUTE_DONE: {
    char tag[48];
    snprintf(tag, sizeof tag, "ROUTED %u pts  %u ms  %u m", s_pathN, (unsigned)s_lastMs, (unsigned)s_lastDistM);
    drawStatus(spr, tag);
    drawPathOverlay(spr);   /* keep the path + markers visible */
    return;
  }
  default:
    return;
  }
}
