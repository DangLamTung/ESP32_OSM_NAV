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

/* ---- confirm dialog ---- */
#define PROMPT_X 60
#define PROMPT_Y 88
#define PROMPT_W 200
#define PROMPT_H 64
#define YES_X (PROMPT_X + 16)
#define YES_Y (PROMPT_Y + 34)
#define YES_W 72
#define YES_H 22
#define NO_X  (PROMPT_X + PROMPT_W - 16 - 72)
#define NO_Y  (PROMPT_Y + 34)
#define NO_W  72
#define NO_H  22

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

static enum { ROUTE_IDLE, ROUTE_PICK_START, ROUTE_PICK_STOP, ROUTE_CONFIRM, ROUTE_DONE } s_mode = ROUTE_IDLE;

static double s_startLat, s_startLon, s_stopLat, s_stopLon;
static int    s_startX = -1, s_startY = -1, s_stopX = -1, s_stopY = -1;

static struct { double lat, lon; } s_path[MAX_PATH_PTS];
static int s_pathN = 0;
static uint32_t s_lastVisited = 0, s_lastMs = 0, s_lastDistM = 0;

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

void routing_init(void)
{
  g_dist    = (uint32_t *)heap_caps_malloc(GRID_N * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_f       = (uint32_t *)heap_caps_malloc(GRID_N * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_prev    = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_closed  = (uint8_t  *)heap_caps_malloc(GRID_N * sizeof(uint8_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_heap    = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_heapPos = (int32_t  *)heap_caps_malloc(GRID_N * sizeof(int32_t),  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!g_dist || !g_f || !g_prev || !g_closed || !g_heap || !g_heapPos) {
    ESP_LOGE(TAG, "failed to allocate A* arrays (%u nodes)", (unsigned)GRID_N);
    return;
  }
  grid_recenter();
  ESP_LOGI(TAG, "test grid %dx%d = %u nodes, arrays %.1f KB PSRAM, box %.1fx%.1f km",
           GRID_W, GRID_H, (unsigned)GRID_N,
           (double)(GRID_N * (4 + 4 + 4 + 1 + 4 + 4)) / 1024.0,
           (g_lon1 - g_lon0) * 109.3, (g_lat1 - g_lat0) * 111.3);
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
  s_startLat = g_lat0 + 0.001; s_startLon = g_lon0 + 0.001;
  s_stopLat  = g_lat1 - 0.001; s_stopLon  = g_lon1 - 0.001;
  ESP_LOGI(TAG, "selftest: A* corner-to-corner of the %.1fkm x %.1fkm box",
           (g_lon1 - g_lon0) * 109.3, (g_lat1 - g_lat0) * 111.3);
  routing_compute();
}

/* ---------------- SD save ---------------- */
static void routing_save_to_sd(void)
{
  FILE *f = fopen("/sdcard/routing_selection.txt", "w");
  if (!f) { ESP_LOGE(TAG, "cannot open /sdcard/routing_selection.txt"); return; }
  ESP_LOGI(TAG, "saving selection to /sdcard/routing_selection.txt");
  fprintf(f, "start %.6f %.6f\n", s_startLat, s_startLon);
  fprintf(f, "stop  %.6f %.6f\n", s_stopLat, s_stopLon);
  fclose(f);
  ESP_LOGI(TAG, "saved selection to /sdcard/routing_selection.txt");
}

/* ---------------- touch ---------------- */
bool routing_handle_tap(int x, int y)
{
  ESP_LOGI(TAG, "tap(%d,%d) mode=%d", x, y, (int)s_mode);   /* DEBUG: watch taps */
  if (s_mode == ROUTE_IDLE) {
    /* only the ROUTE button is consumed while idle */
    if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
        y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) {
      s_mode = ROUTE_PICK_START;
      s_pathN = 0;
      grid_recenter();   /* box follows the car so taps land on distinct nodes */
      /* routing needs the map: if we are in the text-only SIMPLE screen,
       * switch to FULL so the crosshairs + path are visible */
      if (ui_nav_mode() == UI_MODE_SIMPLE) ui_cycle_nav_mode();
      map_set_heading_up(false);   /* crosshair mapping assumes north-up */
      map_set_rotation(0);
      ESP_LOGI(TAG, "pick start (tap the map)");
      ui_mark_redraw();
      return true;
    }
    return false;
  }

  if (s_mode == ROUTE_PICK_START) {
    s_startX = x; s_startY = y;
    map_screen_to_latlon(x, y, &s_startLat, &s_startLon);
    s_mode = ROUTE_PICK_STOP;
    ESP_LOGI(TAG, "start %.6f,%.6f - pick stop", s_startLat, s_startLon);
    ui_mark_redraw();
    return true;
  }
  if (s_mode == ROUTE_PICK_STOP) {
    s_stopX = x; s_stopY = y;
    map_screen_to_latlon(x, y, &s_stopLat, &s_stopLon);
    s_mode = ROUTE_CONFIRM;
    ESP_LOGI(TAG, "stop %.6f,%.6f - computing preview", s_stopLat, s_stopLon);
    routing_compute();   /* run A* now so the confirm dialog shows the real path */
    ui_mark_redraw();
    return true;
  }
  if (s_mode == ROUTE_CONFIRM) {
    if (x >= YES_X && x < YES_X + YES_W && y >= YES_Y && y < YES_Y + YES_H) {
      s_mode = ROUTE_DONE;
      routing_compute();
      return true;
    }
    if (x >= NO_X && x < NO_X + NO_W && y >= NO_Y && y < NO_Y + NO_H) {
      routing_save_to_sd();
      s_mode = ROUTE_IDLE;
      ui_mark_redraw();
      return true;
    }
    ESP_LOGI(TAG, "confirm tap at (%d,%d) - hit Yes/No", x, y);
    return true;   /* consumed; must hit Yes/No */
  }
  /* ROUTE_DONE: the path PERSISTS (drawn screen-fixed, follows pan/zoom).
   * Tapping ROUTE again starts a fresh route and clears the old path; every
   * other tap falls through so zoom/pan/settings still work — previously ANY
   * tap cleared the path, so zooming in deleted the route. */
  if (x >= ROUTE_BTN_X && x < ROUTE_BTN_X + ROUTE_BTN_W &&
      y >= ROUTE_BTN_Y && y < ROUTE_BTN_Y + ROUTE_BTN_H) {
    s_mode = ROUTE_PICK_START;
    s_pathN = 0;
    grid_recenter();
    if (ui_nav_mode() == UI_MODE_SIMPLE) ui_cycle_nav_mode();
    map_set_heading_up(false);
    map_set_rotation(0);
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

static void drawStatus(LGFX_Sprite &spr, const char *txt) {
  spr.setTextFont(1);
  spr.setTextColor(TFT_WHITE, 0x2104);
  int w = spr.textWidth(txt);
  spr.fillRect(SCREEN_W / 2 - w / 2 - 8, 2, w + 16, 15, 0x2104);
  spr.drawRect(SCREEN_W / 2 - w / 2 - 8, 2, w + 16, 15, 0x39E7);
  spr.setCursor(SCREEN_W / 2 - w / 2, 5);
  spr.print(txt);
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
  case ROUTE_PICK_START:
    drawStatus(spr, "PICK START");
    return;
  case ROUTE_PICK_STOP:
    drawStatus(spr, "PICK STOP");
    if (s_startX >= 0) drawCrosshair(spr, s_startX, s_startY, ROUTE_COL_START);
    return;
  case ROUTE_CONFIRM: {
    char conf[48];
    if (s_pathN >= 2)
      snprintf(conf, sizeof conf, "CONFIRM - %u pts, %u m", s_pathN, (unsigned)s_lastDistM);
    else
      snprintf(conf, sizeof conf, "CONFIRM");
    drawStatus(spr, conf);
    drawPathOverlay(spr);   /* preview the route before confirming */
    if (s_startX >= 0) drawCrosshair(spr, s_startX, s_startY, ROUTE_COL_START);
    if (s_stopX >= 0)  drawCrosshair(spr, s_stopX,  s_stopY,  ROUTE_COL_STOP);
    const uint16_t dlg = spr.color565(22, 24, 30);
    spr.fillRoundRect(PROMPT_X, PROMPT_Y, PROMPT_W, PROMPT_H, 6, dlg);
    spr.drawRoundRect(PROMPT_X, PROMPT_Y, PROMPT_W, PROMPT_H, 6, TFT_WHITE);
    spr.setTextColor(TFT_WHITE, dlg);
    spr.setTextFont(2);
    spr.setCursor(PROMPT_X + 20, PROMPT_Y + 8);
    spr.print("Add path?");
    spr.fillRoundRect(YES_X, YES_Y, YES_W, YES_H, 4, spr.color565(40, 150, 70));
    spr.setTextColor(TFT_WHITE, spr.color565(40, 150, 70));
    spr.setCursor(YES_X + 20, YES_Y + 5);
    spr.print("Yes");
    spr.fillRoundRect(NO_X, NO_Y, NO_W, NO_H, 4, spr.color565(180, 60, 60));
    spr.setTextColor(TFT_WHITE, spr.color565(180, 60, 60));
    spr.setCursor(NO_X + 24, NO_Y + 5);
    spr.print("No");
    spr.setTextFont(1);
    return;
  }
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
