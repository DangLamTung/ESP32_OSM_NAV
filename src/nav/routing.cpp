/**
 * routing.cpp — offline-routing prototype (routing.h).
 *
 * A* with a decrease-key binary heap over a synthetic 8-connected grid graph
 * held in PSRAM. The graph spans a lat/lon box around Bến Thành so screen taps
 * (converted via map_screen_to_latlon) land on it. This validates the engine,
 * heap, timing (ms) and memory on the real ESP32 before a real OSM graph is
 * wired in.
 *
 * UI flow (touch):
 *   ROUTE button -> PICK_START -> (tap) -> PICK_STOP -> (tap) -> CONFIRM
 *     Yes -> run A*, draw path (magenta) on the world, log stats
 *     No  -> save start/stop to /sdcard/routing_selection.txt and exit
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

/* ---- synthetic test grid (box centred on the CURRENT map centre so taps
 *      always land inside it — start/stop stay distinct) ---- */
#define GRID_W 240
#define GRID_H 160
#define GRID_N (GRID_W * GRID_H)
#define MAX_PATH_PTS 4096
#define INF 0xFFFFFFFFu
static double g_lat0, g_lat1, g_lon0, g_lon1;   /* grid box (re-centred on the car) */

/* start = cyan, stop = yellow (clearly distinct on the map) */
#define ROUTE_COL_START 0x07FF
#define ROUTE_COL_STOP  0xFFE0

static enum { ROUTE_IDLE, ROUTE_PICK_START, ROUTE_PICK_STOP, ROUTE_DONE } s_mode = ROUTE_IDLE;
static bool s_useReal = false;   /* real OSM graph loaded (else synthetic grid) */
static uint32_t s_announceMS = 0;   /* show the ±range announcement briefly */

bool routing_active(void) { return s_mode != ROUTE_IDLE; }

static double s_startLat, s_startLon, s_stopLat, s_stopLon;
static int    s_startX = -1, s_startY = -1, s_stopX = -1, s_stopY = -1;

static struct { double lat, lon; } s_path[MAX_PATH_PTS];
static int s_pathN = 0;
static uint32_t s_lastVisited = 0, s_lastMs = 0, s_lastDistM = 0;
static double *s_rlat = NULL, *s_rlon = NULL;   /* real-graph path scratch (PSRAM) */

/* ---- A* arrays (PSRAM) ---- */
static uint32_t *g_dist;      /* g score (cost units) */
static uint32_t *g_f;         /* f = g + h */
static int32_t  *g_prev;      /* reconstruction */
static uint8_t  *g_closed;
static int32_t  *g_heap;      /* binary heap of node ids */
static int32_t  *g_heapPos;   /* node -> heap index (-1 = not in heap) */
static int       g_heapN = 0;

static inline int idx(int col, int row) { return row * GRID_W + col; }

/* node -> lat/lon (grid cell centers) */
static inline double nodeLat(int row) { return g_lat0 + (g_lat1 - g_lat0) * row / (GRID_H - 1); }
static inline double nodeLon(int col) { return g_lon0 + (g_lon1 - g_lon0) * col / (GRID_W - 1); }

static inline int colOf(double lon) {
  double c = (lon - g_lon0) / (g_lon1 - g_lon0) * (GRID_W - 1);
  if (c < 0) { c = 0; }
  if (c > GRID_W - 1) { c = GRID_W - 1; }
  return (int)lround(c);
}
static inline int rowOf(double lat) {
  double r = (lat - g_lat0) / (g_lat1 - g_lat0) * (GRID_H - 1);
  if (r < 0) { r = 0; }
  if (r > GRID_H - 1) { r = GRID_H - 1; }
  return (int)lround(r);
}

/* re-centre the ~5.5km box on the current map centre so taps near the car map
 * to distinct grid nodes (fixes start==stop -> no path). */
static void grid_recenter(void) {
  const double spanLat = 0.05, spanLon = 0.05;   /* ~5.5km x ~5.5km */
  g_lat0 = centerLat - spanLat / 2.0;
  g_lat1 = centerLat + spanLat / 2.0;
  g_lon0 = centerLon - spanLon / 2.0;
  g_lon1 = centerLon + spanLon / 2.0;
}

/* exact 8-neighbour admissible heuristic: 10 per straight + 14 per diagonal */
static uint32_t heuristic(int col, int row, int tCol, int tRow) {
  int dc = abs(col - tCol), dr = abs(row - tRow);
  int mn = (dr < dc) ? dr : dc;
  return (uint32_t)(10 * (dr + dc - 2 * mn) + 14 * mn);
}

/* (re)build the synthetic test grid — the fallback used when the real graph
 * is not loaded (offline routing OFF). Returns false on OOM. */
static bool synthetic_alloc(void)
{
  g_dist    = (uint32_t *)heap_caps_malloc(GRID_N * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_f       = (uint32_t *)heap_caps_malloc(GRID_N * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_prev    = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_closed  = (uint8_t  *)heap_caps_malloc(GRID_N * sizeof(uint8_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_heap    = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_heapPos = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!g_dist || !g_f || !g_prev || !g_closed || !g_heap || !g_heapPos) {
    ESP_LOGE(TAG, "failed to allocate A* arrays (%u nodes)", (unsigned)GRID_N);
    return false;
  }
  grid_recenter();
  ESP_LOGI(TAG, "test grid %dx%d = %u nodes, arrays %.1f KB PSRAM, box %.1fx%.1f km",
           GRID_W, GRID_H, (unsigned)GRID_N,
           (double)(GRID_N * (4 + 4 + 4 + 1 + 4 + 4)) / 1024.0,
           (g_lon1 - g_lon0) * 109.3, (g_lat1 - g_lat0) * 111.3);
  return true;
}

static void synthetic_free(void)
{
  free(g_dist); free(g_f); free(g_prev); free(g_closed); free(g_heap); free(g_heapPos);
  g_dist = NULL; g_f = NULL; g_prev = NULL; g_heap = NULL; g_heapPos = NULL; g_closed = NULL;
}

void routing_init(void)
{
  s_rlat = (double *)heap_caps_malloc(MAX_PATH_PTS * sizeof(double), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  s_rlon = (double *)heap_caps_malloc(MAX_PATH_PTS * sizeof(double), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_rlat) s_rlat = (double *)malloc(MAX_PATH_PTS * sizeof(double));
  if (!s_rlon) s_rlon = (double *)malloc(MAX_PATH_PTS * sizeof(double));
  synthetic_alloc();   /* fallback grid; the real graph loads via routing_set_offline */
}

/* Enable/disable offline (real-road) routing at runtime. ON loads the graph
 * from SD and frees the synthetic grid; OFF unloads it (faster boot, less
 * PSRAM when offline routing isn't needed). Called from the settings panel
 * and from setup() with the persisted setting. */
void routing_set_offline(bool on)
{
  if (on == s_useReal)
    return;
  if (on) {
    synthetic_free();   /* make room FIRST so the real A* arrays fit */
    if (rg_load("/sdcard/routing.rng") && rg_astar_init()) {
      s_useReal = true;
      ESP_LOGI(TAG, "offline routing ON: real graph active (%u nodes)",
               (unsigned)rg_node_count());
    } else {
      rg_unload();
      synthetic_alloc();   /* keep the fallback grid working */
      ESP_LOGE(TAG, "offline routing ON but graph load failed - synthetic grid");
    }
  } else {
    rg_unload();
    s_useReal = false;
    s_pathN = 0;
    synthetic_alloc();
    ESP_LOGI(TAG, "offline routing OFF - synthetic grid");
  }
  ui_mark_redraw();
}

/* ---------------- binary heap (min by g_f, decrease-key) ---------------- */
static void heapPush(int32_t node) {
  g_heapPos[node] = g_heapN;
  int i = g_heapN++;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (g_f[g_heap[p]] <= g_f[node]) break;
    g_heap[i] = g_heap[p]; g_heapPos[g_heap[p]] = i;
    i = p;
  }
  g_heap[i] = node; g_heapPos[node] = i;
}
static int32_t heapPop(void) {
  int32_t top = g_heap[0];
  int32_t last = g_heap[--g_heapN];
  if (g_heapN > 0) {
    int i = 0;
    for (;;) {
      int l = 2 * i + 1, r = l + 1, m = i;
      if (l < g_heapN && g_f[g_heap[l]] < g_f[g_heap[m]]) m = l;
      if (r < g_heapN && g_f[g_heap[r]] < g_f[g_heap[m]]) m = r;
      if (m == i) break;
      g_heap[i] = g_heap[m]; g_heapPos[g_heap[m]] = i;
      i = m;
    }
    g_heap[i] = last; g_heapPos[last] = i;
  }
  g_heapPos[top] = -1;
  return top;
}
static void heapDecreaseKey(int32_t node) {
  int i = g_heapPos[node];
  while (i > 0) {
    int p = (i - 1) / 2;
    if (g_f[g_heap[p]] <= g_f[node]) break;
    g_heap[i] = g_heap[p]; g_heapPos[g_heap[p]] = i;
    i = p;
  }
  g_heap[i] = node; g_heapPos[node] = i;
}

/* ---------------- A* ---------------- */
static bool routing_compute(void)
{
  /* ---- real OSM road network (from SD) ---- */
  if (s_useReal) {
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

  /* ---- synthetic test grid (fallback) ---- */
  int sCol = colOf(s_startLon), sRow = rowOf(s_startLat);
  int tCol = colOf(s_stopLon),  tRow = rowOf(s_stopLat);
  int start = idx(sCol, sRow), stop = idx(tCol, tRow);

  s_pathN = 0;
  if (start == stop) {
    s_path[0].lat = s_startLat; s_path[0].lon = s_startLon;
    s_pathN = 1;
    ESP_LOGI(TAG, "start==stop, trivial");
    return true;
  }

  g_heapN = 0;
  for (int i = 0; i < GRID_N; i++) {
    g_dist[i] = INF; g_f[i] = INF; g_prev[i] = -1; g_closed[i] = 0; g_heapPos[i] = -1;
  }
  g_dist[start] = 0;
  g_f[start] = heuristic(sCol, sRow, tCol, tRow);
  heapPush(start);

  const int dc[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  const int dr[8] = {0, 1, 0, -1, 1, -1, 1, -1};
  const uint32_t cc[8] = {10, 10, 10, 10, 14, 14, 14, 14};

  uint32_t visited = 0;
  uint32_t t0 = esp_timer_get_time();
  bool found = false;

  while (g_heapN > 0) {
    int32_t u = heapPop();
    if (g_closed[u]) continue;
    g_closed[u] = 1;
    visited++;
    if (u == stop) { found = true; break; }

    int ur = u / GRID_W, uc = u % GRID_W;
    for (int k = 0; k < 8; k++) {
      int nc = uc + dc[k], nr = ur + dr[k];
      if (nc < 0 || nc >= GRID_W || nr < 0 || nr >= GRID_H) continue;
      int v = idx(nc, nr);
      if (g_closed[v]) continue;
      uint32_t nd = g_dist[u] + cc[k];
      if (nd < g_dist[v]) {
        g_dist[v] = nd;
        g_prev[v] = u;
        g_f[v] = nd + heuristic(nc, nr, tCol, tRow);
        if (g_heapPos[v] < 0) heapPush(v); else heapDecreaseKey(v);
      }
    }
  }
  uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

  if (!found) {
    ESP_LOGW(TAG, "A* no path found (visited %u in %ums)", (unsigned)visited, (unsigned)ms);
    s_lastVisited = visited; s_lastMs = ms; s_lastDistM = 0;
    return false;
  }

  /* reconstruct: stop -> start via prev */
  static int rev[MAX_PATH_PTS];
  int revN = 0;
  int32_t cur = stop;
  while (cur >= 0 && revN < MAX_PATH_PTS) {
    rev[revN++] = (int)cur;
    if (cur == start) break;
    cur = g_prev[cur];
  }
  /* reverse into s_path[] with real lat/lon */
  s_pathN = 0;
  for (int i = revN - 1; i >= 0 && s_pathN < MAX_PATH_PTS; i--) {
    int n = rev[i];
    s_path[s_pathN].lat = nodeLat(n / GRID_W);
    s_path[s_pathN].lon = nodeLon(n % GRID_W);
    s_pathN++;
  }

  /* meters: sum cell sizes along the path */
  double mppLat = (g_lat1 - g_lat0) / (GRID_H - 1) * 111320.0;
  double mppLon = (g_lon1 - g_lon0) / (GRID_W - 1) * 111320.0 * cos(s_startLat * M_PI / 180.0);
  uint32_t meters = 0;
  for (int i = 1; i < s_pathN; i++) {
    double dLat = (s_path[i].lat - s_path[i - 1].lat) / ((g_lat1 - g_lat0) / (GRID_H - 1)) * mppLat;
    double dLon = (s_path[i].lon - s_path[i - 1].lon) / ((g_lon1 - g_lon0) / (GRID_W - 1)) * mppLon;
    meters += (uint32_t)lround(sqrt(dLat * dLat + dLon * dLon));
  }

  s_lastVisited = visited; s_lastMs = ms; s_lastDistM = meters;
  ESP_LOGI(TAG, "A* done: visited=%u time=%ums path=%dpts dist=%um sram=%u",
           (unsigned)visited, (unsigned)ms, s_pathN, (unsigned)meters,
           (unsigned)esp_get_free_heap_size());
  ui_mark_redraw();   /* path is drawn screen-fixed in the overlay -> no recompose */
  return true;
}

/* Boot self-test: A* corner-to-corner of the test grid box, logs the real
 * on-device timing/memory so we can validate the engine without the UI. */
void routing_selftest(void)
{
  if (s_useReal) {
    /* route between two CENTRAL points (35% / 65% across the box) so both
     * ends are in the main connected component — corner-to-corner can hit
     * bbox-truncation islands and report a bogus no-path. */
    double mnla, mnlo, mxla, mxlo;
    rg_bbox(&mnla, &mnlo, &mxla, &mxlo);
    s_startLat = mnla + (mxla - mnla) * 0.35; s_startLon = mnlo + (mxlo - mnlo) * 0.35;
    s_stopLat  = mnla + (mxla - mnla) * 0.65; s_stopLon  = mnlo + (mxlo - mnlo) * 0.65;
    ESP_LOGI(TAG, "selftest: real graph center route (%.4f,%.4f)->(%.4f,%.4f)",
             s_startLat, s_startLon, s_stopLat, s_stopLon);
  } else {
    s_startLat = g_lat0 + 0.001; s_startLon = g_lon0 + 0.001;
    s_stopLat  = g_lat1 - 0.001; s_stopLon  = g_lon1 - 0.001;
    ESP_LOGI(TAG, "selftest: A* corner-to-corner of the %.1fkm x %.1fkm box",
             (g_lon1 - g_lon0) * 109.3, (g_lat1 - g_lat0) * 111.3);
  }
  routing_compute();
}

/* ---------------- touch ---------------- */
/* Reload the whole-country window around the current map centre so offline
 * routing works wherever the user has panned to (the RNG2 window is only
 * centred on the load-time map centre). No-op if it still covers us. */
static void routing_reload_window(void)
{
  if (!s_useReal || !rg_is_windowed())
    return;
  if (rg_window_covers(centerLat, centerLon, 0.01))
    return;   /* still inside the loaded window */
  rg_unload();
  if (rg_load("/sdcard/routing.rng") && rg_astar_init()) {
    s_pathN = 0;
    ESP_LOGI(TAG, "window reloaded around (%.4f,%.4f) - %u nodes",
             centerLat, centerLon, (unsigned)rg_node_count());
  } else {
    rg_unload(); s_useReal = false; synthetic_alloc();
    ESP_LOGE(TAG, "window reload failed - synthetic grid");
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
    /* only the ROUTE button is consumed while idle */
    if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
        y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) {
      routing_reload_window();   /* pan-to-anywhere support */
      s_mode = ROUTE_PICK_START;
      s_pathN = 0;
      grid_recenter();   /* box follows the car so taps land on distinct nodes */
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
    routing_reload_window();   /* pan-to-anywhere support */
    s_mode = ROUTE_PICK_START;
    s_pathN = 0;
    grid_recenter();
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
  /* ROUTE button (always visible) */
  uint16_t bg = (s_mode != ROUTE_IDLE) ? spr.color565(40, 150, 70) : spr.color565(70, 74, 82);
  spr.fillRoundRect(ROUTE_BTN_X, ROUTE_BTN_Y, ROUTE_BTN_W, ROUTE_BTN_H, 4, bg);
  spr.drawRoundRect(ROUTE_BTN_X, ROUTE_BTN_Y, ROUTE_BTN_W, ROUTE_BTN_H, 4, TFT_BLACK);
  spr.setTextColor(TFT_WHITE, bg);
  spr.setTextFont(1);
  spr.setCursor(ROUTE_BTN_X + 4, ROUTE_BTN_Y + 7);
  spr.print("ROUTE");
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

/* Draw the computed path as a magenta polyline on the north-up world sprite
 * (rotates/scrolls with the map, same as the nav route). */
void routing_draw_world(LGFX_Sprite &world, double refLon, double refLat, int zoom)
{
  if (s_pathN < 2) return;
  double tlX = lon2wx(refLon, zoom) - world.width() / 2.0;
  double tlY = lat2wy(refLat, zoom) - world.height() / 2.0;
  int px = 0, py = 0;
  for (int i = 0; i < s_pathN; i++) {
    int sx = (int)lround(lon2wx(s_path[i].lon, zoom) - tlX);
    int sy = (int)lround(lat2wy(s_path[i].lat, zoom) - tlY);
    if (i > 0) {
      world.drawWideLine(px, py, sx, sy, 3.0f, TFT_NAVY);
      world.drawWideLine(px, py, sx, sy, 1.5f, 0xF81F);   /* magenta */
    }
    px = sx; py = sy;
  }
}
