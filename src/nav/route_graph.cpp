/**
 * route_graph.cpp — real OSM road graph load + A* (see route_graph.h).
 *
 * Memory (PSRAM):
 *   graph  : lat[4N] + lon[4N] + first[4(N+1)] + to[4E] + w[2E]
 *   snap   : cellFirst[4*C] + cellNode[4N]
 *   A*     : dist[4N] + f[4N] + prev[4N] + closed[N] + heapPos[4N] + heap[4N]
 * A ~0.10° city box (N≈100k, E≈195k) totals ≈ 4.4 MB — fits the ESP32-S3's
 * 8 MB PSRAM after routing.cpp frees the synthetic grid.
 */
#include "route_graph.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "rgraph";

/* ---- graph storage (PSRAM) ---- */
static int32_t  *g_lat   = NULL;   /* e7 */
static int32_t  *g_lon   = NULL;
static uint32_t *g_first = NULL;   /* CSR, N+1 entries */
static uint32_t *g_to    = NULL;
static uint16_t *g_w     = NULL;   /* 0.1 s */
static uint32_t  g_N = 0, g_E = 0;
static int32_t   g_minlat = 0, g_minlon = 0, g_maxlat = 0, g_maxlon = 0;
static bool      g_loaded = false;

/* ---- tap-snap cell index (PSRAM) ---- */
#define CELL_DEG 0.0040            /* ~440 m cells */
static uint16_t g_cellW = 0, g_cellH = 0;
static uint32_t *g_cellFirst = NULL;
static uint32_t *g_cellNode  = NULL;

/* ---- A* working arrays (PSRAM, allocated once on load) ---- */
static uint32_t *g_dist    = NULL;
static uint32_t *g_f       = NULL;
static int32_t  *g_prev    = NULL;
static uint8_t  *g_closed  = NULL;
static int32_t  *g_heap    = NULL;
static int32_t  *g_heapPos = NULL;
static int       g_heapN   = 0;

/* path reconstruction buffer (PSRAM) */
static int32_t  *s_rev = NULL;
#define REV_MAX 4096

#define INF 0xFFFFFFFFu

/* ---- helpers ---- */
static inline double havR(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0;
    double p1 = lat1 * M_PI / 180.0, p2 = lat2 * M_PI / 180.0;
    double dp = (lat2 - lat1) * M_PI / 180.0, dl = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dp / 2) * sin(dp / 2) +
               cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2 * R * asin(sqrt(a));
}

double rg_lat(uint32_t n) { return g_lat ? (double)g_lat[n] / 1e7 : 0.0; }
double rg_lon(uint32_t n) { return g_lon ? (double)g_lon[n] / 1e7 : 0.0; }
bool   rg_loaded(void)    { return g_loaded; }
uint32_t rg_node_count(void) { return g_N; }

void rg_bbox(double *minLat, double *minLon, double *maxLat, double *maxLon)
{
    *minLat = (double)g_minlat / 1e7; *minLon = (double)g_minlon / 1e7;
    *maxLat = (double)g_maxlat / 1e7; *maxLon = (double)g_maxlon / 1e7;
}

static inline uint32_t nodeCell(double lat, double lon)
{
    int cx = (int)((lon - (double)g_minlon / 1e7) / CELL_DEG);
    int cy = (int)((lat - (double)g_minlat / 1e7) / CELL_DEG);
    if (cx < 0) cx = 0;
    if (cx >= (int)g_cellW) cx = g_cellW - 1;
    if (cy < 0) cy = 0;
    if (cy >= (int)g_cellH) cy = g_cellH - 1;
    return (uint32_t)(cy * g_cellW + cx);
}

/* ---- load ---- */
bool rg_load(const char *path)
{
    if (g_loaded)
        return true;
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        ESP_LOGW(TAG, "no %s — keeping synthetic grid", path);
        return false;
    }

    char magic[4];
    uint32_t ver = 0, N = 0, E = 0;
    int32_t minlat = 0, minlon = 0, maxlat = 0, maxlon = 0;
    bool ok = (fread(magic, 1, 4, fp) == 4 && memcmp(magic, "RNG1", 4) == 0 &&
               fread(&ver, 4, 1, fp) == 1 && fread(&N, 4, 1, fp) == 1 &&
               fread(&E, 4, 1, fp) == 1 && fread(&minlat, 4, 1, fp) == 1 &&
               fread(&minlon, 4, 1, fp) == 1 && fread(&maxlat, 4, 1, fp) == 1 &&
               fread(&maxlon, 4, 1, fp) == 1);
    if (!ok || ver != 1 || N == 0 || N > 10000000 || E > 20000000) {
        ESP_LOGE(TAG, "bad .rng header (ver=%u N=%u E=%u)", (unsigned)ver, (unsigned)N, (unsigned)E);
        fclose(fp);
        return false;
    }

    /* set bbox globals NOW — nodeCell() uses g_minlat/g_minlon to build the
     * snap cell index below, and they must be correct before that loop. */
    g_minlat = minlat; g_minlon = minlon; g_maxlat = maxlat; g_maxlon = maxlon;

    int32_t  *lat   = (int32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int32_t  *lon   = (int32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *first = (uint32_t *)heap_caps_malloc((size_t)(N + 1) * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *to    = (uint32_t *)heap_caps_malloc((size_t)E * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint16_t *w     = (uint16_t *)heap_caps_malloc((size_t)E * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!lat || !lon || !first || !to || !w) {
        ESP_LOGE(TAG, "graph alloc failed (N=%u E=%u)", (unsigned)N, (unsigned)E);
        free(lat); free(lon); free(first); free(to); free(w);
        fclose(fp);
        return false;
    }
    ok = (fread(lat, 4, N, fp) == N && fread(lon, 4, N, fp) == N &&
          fread(first, 4, N + 1, fp) == N + 1 &&
          fread(to, 4, E, fp) == E && fread(w, 2, E, fp) == E);
    fclose(fp);
    if (!ok) {
        ESP_LOGE(TAG, "graph read failed");
        free(lat); free(lon); free(first); free(to); free(w);
        return false;
    }

    /* ---- tap-snap cell index ---- */
    g_cellW = (uint16_t)(((double)(maxlon - minlon) / 1e7) / CELL_DEG) + 1;
    g_cellH = (uint16_t)(((double)(maxlat - minlat) / 1e7) / CELL_DEG) + 1;
    uint32_t cells = (uint32_t)g_cellW * g_cellH;
    uint32_t *cellFirst = (uint32_t *)heap_caps_calloc(cells + 1, 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *cellNode  = (uint32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!cellFirst || !cellNode) {
        ESP_LOGE(TAG, "cell index alloc failed");
        free(lat); free(lon); free(first); free(to); free(w);
        free(cellFirst); free(cellNode);
        return false;
    }
    for (uint32_t i = 0; i < N; i++)
        cellFirst[nodeCell((double)lat[i] / 1e7, (double)lon[i] / 1e7) + 1]++;
    for (uint32_t c = 0; c < cells; c++)
        cellFirst[c + 1] += cellFirst[c];
    {
        uint32_t *cur = (uint32_t *)heap_caps_malloc(cells * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memcpy(cur, cellFirst, cells * 4);
        for (uint32_t i = 0; i < N; i++) {
            uint32_t c = nodeCell((double)lat[i] / 1e7, (double)lon[i] / 1e7);
            cellNode[cur[c]++] = i;
        }
        free(cur);
    }

    g_lat = lat; g_lon = lon; g_first = first; g_to = to; g_w = w;
    g_N = N; g_E = E;
    g_cellFirst = cellFirst; g_cellNode = cellNode;
    g_loaded = true;

    size_t mb = ((size_t)N * 4 + N * 4 + (N + 1) * 4 + E * 4 + E * 2 +
                 cells * 4 + N * 4) / 1024 / 1024;
    ESP_LOGI(TAG, "loaded %s: N=%u E=%u (%.1f MB PSRAM graph+cell) bbox %.5f..%.5f / %.5f..%.5f",
             path, (unsigned)N, (unsigned)E, (double)mb,
             (double)minlat / 1e7, (double)maxlat / 1e7,
             (double)minlon / 1e7, (double)maxlon / 1e7);
    return true;
}

/* Allocate the A* working arrays (PSRAM). Call AFTER freeing the synthetic
 * grid so the real graph + A* arrays fit together. Returns false on OOM. */
bool rg_astar_init(void)
{
    if (!g_loaded || g_dist)
        return g_loaded && g_dist;
    uint32_t N = g_N;
    uint32_t *dist    = (uint32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *farr    = (uint32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int32_t  *prev    = (int32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t  *closed  = (uint8_t *)heap_caps_malloc((size_t)N, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int32_t  *heap    = (int32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int32_t  *heapPos = (int32_t *)heap_caps_malloc((size_t)N * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!dist || !farr || !prev || !closed || !heap || !heapPos) {
        ESP_LOGE(TAG, "A* arrays alloc failed (N=%u)", (unsigned)N);
        free(dist); free(farr); free(prev); free(closed); free(heap); free(heapPos);
        return false;
    }
    g_dist = dist; g_f = farr; g_prev = prev; g_closed = closed;
    g_heap = heap; g_heapPos = heapPos;
    s_rev = (int32_t *)heap_caps_malloc(REV_MAX * sizeof(int32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rev)
        s_rev = (int32_t *)malloc(REV_MAX * sizeof(int32_t));
    ESP_LOGI(TAG, "A* arrays: %.1f MB PSRAM", (double)(N * (4 + 4 + 4 + 1 + 4 + 4) + REV_MAX * 4) / 1048576.0);
    return true;
}

/* Free the whole graph + A* arrays (used to fall back to the synthetic grid
 * when there is not enough PSRAM). */
void rg_unload(void)
{
    free(g_lat); free(g_lon); free(g_first); free(g_to); free(g_w);
    free(g_cellFirst); free(g_cellNode);
    free(g_dist); free(g_f); free(g_prev); free(g_closed); free(g_heap); free(g_heapPos); free(s_rev);
    g_lat = g_lon = NULL; g_first = NULL; g_to = NULL; g_w = NULL;
    g_cellFirst = NULL; g_cellNode = NULL;
    g_dist = g_f = NULL; g_prev = NULL; g_closed = NULL; g_heap = NULL; g_heapPos = NULL; s_rev = NULL;
    g_N = g_E = 0; g_loaded = false;
}

/* ---- nearest node ---- */
int rg_nearest(double lat, double lon, double maxRadDeg)
{
    if (!g_loaded)
        return -1;
    int ccx = (int)((lon - (double)g_minlon / 1e7) / CELL_DEG);
    int ccy = (int)((lat - (double)g_minlat / 1e7) / CELL_DEG);
    int R = (int)ceil(maxRadDeg / CELL_DEG);
    int best = -1;
    double bestD = INF;
    for (int dy = -R; dy <= R; dy++) {
        for (int dx = -R; dx <= R; dx++) {
            int cx = ccx + dx, cy = ccy + dy;
            if (cx < 0 || cx >= (int)g_cellW || cy < 0 || cy >= (int)g_cellH)
                continue;
            uint32_t c = (uint32_t)(cy * g_cellW + cx);
            for (uint32_t k = g_cellFirst[c]; k < g_cellFirst[c + 1]; k++) {
                uint32_t n = g_cellNode[k];
                double d = havR(lat, lon, (double)g_lat[n] / 1e7, (double)g_lon[n] / 1e7);
                if (d < bestD) { bestD = d; best = (int)n; }
            }
        }
    }
    if (best < 0 || bestD > maxRadDeg * 111320.0)
        return -1;
    return best;
}

/* ---- binary heap (min by f, decrease-key) — same as the synthetic A* ---- */
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

/* ---- A* ---- */
int rg_route(double sLat, double sLon, double tLat, double tLon,
             double *outLat, double *outLon, int maxPts,
             double *distM, uint32_t *msOut)
{
    if (!g_loaded)
        return 0;
    int s = rg_nearest(sLat, sLon, 0.01);
    int t = rg_nearest(tLat, tLon, 0.01);
    if (s < 0 || t < 0) {
        ESP_LOGW(TAG, "snap failed s=%d t=%d", s, t);
        return 0;
    }
    if (s == t) {
        outLat[0] = rg_lat((uint32_t)s); outLon[0] = rg_lon((uint32_t)s);
        if (distM) *distM = 0.0;
        if (msOut) *msOut = 0;
        return 1;
    }

    g_heapN = 0;
    for (uint32_t i = 0; i < g_N; i++) {
        g_dist[i] = INF; g_f[i] = INF; g_prev[i] = -1; g_closed[i] = 0; g_heapPos[i] = -1;
    }
    g_dist[s] = 0;
    g_f[s] = 0;   /* f recomputed on pop via heuristic */
    heapPush(s);

    /* cheap admissible time heuristic: straight-line distance (no trig) */
    const double sTgtLat = rg_lat((uint32_t)t), sTgtLon = rg_lon((uint32_t)t);
    const double sTgtCos = cos(sTgtLat * M_PI / 180.0);
    const double hScale = 0.036;   /* 100 km/h -> 0.1 s per ~0.28 m */

    uint32_t visited = 0;
    uint32_t t0 = esp_timer_get_time();
    bool found = false;

    while (g_heapN > 0) {
        int32_t u = heapPop();
        if (g_closed[u]) continue;
        g_closed[u] = 1;
        visited++;
        if (u == t) { found = true; break; }

        uint32_t gu = g_dist[u];
        for (uint32_t k = g_first[u]; k < g_first[u + 1]; k++) {
            uint32_t v = g_to[k];
            if (g_closed[v]) continue;
            uint32_t nd = gu + g_w[k];
            if (nd < g_dist[v]) {
                g_dist[v] = nd;
                g_prev[v] = u;
                double dLat = (rg_lat(v) - sTgtLat) * 111320.0;
                double dLon = (rg_lon(v) - sTgtLon) * 111320.0 * sTgtCos;
                uint32_t h = (uint32_t)(sqrt(dLat * dLat + dLon * dLon) * hScale);
                g_f[v] = nd + h;
                if (g_heapPos[v] < 0) heapPush((int32_t)v); else heapDecreaseKey((int32_t)v);
            }
        }
    }
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    if (!found) {
        ESP_LOGW(TAG, "A* no path (visited %u in %ums)", (unsigned)visited, (unsigned)ms);
        if (msOut) *msOut = ms;
        return 0;
    }

    /* reconstruct s..t */
    if (!s_rev)
        return 0;
    int revN = 0;
    int32_t cur = t;
    while (cur >= 0 && revN < REV_MAX) {
        s_rev[revN++] = cur;
        if (cur == s) break;
        cur = g_prev[cur];
    }
    if (revN > maxPts) revN = maxPts;
    double meters = 0.0;
    int n = 0;
    for (int i = revN - 1; i >= 0; i--, n++) {
        outLat[n] = rg_lat((uint32_t)s_rev[i]);
        outLon[n] = rg_lon((uint32_t)s_rev[i]);
        if (i < revN - 1) {
            int j = i + 1;
            meters += havR(outLat[n], outLon[n], rg_lat((uint32_t)s_rev[j]), rg_lon((uint32_t)s_rev[j]));
        }
    }
    if (distM) *distM = meters;
    if (msOut) *msOut = ms;
    ESP_LOGI(TAG, "A* real: visited=%u time=%ums path=%dpts dist=%.0fm s=%d t=%d",
             (unsigned)visited, (unsigned)ms, n, meters, s, t);
    return n;
}
