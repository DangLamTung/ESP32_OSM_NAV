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
#include "map_view.h"   /* centerLat / centerLon for the RNG2 window */

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
static bool rg_load_rng1(FILE *fp, const char *path);
static bool rg_load_rng2(FILE *fp, const char *path);

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
    if (fread(magic, 1, 4, fp) != 4) {
        fclose(fp);
        return false;
    }
    fseek(fp, 0, SEEK_SET);
    if (memcmp(magic, "RNG1", 4) == 0)
        return rg_load_rng1(fp, path);
    if (memcmp(magic, "RNG2", 4) == 0)
        return rg_load_rng2(fp, path);
    ESP_LOGE(TAG, "bad graph magic");
    fclose(fp);
    return false;
}

/* RNG1: small whole-file graph (single region). Loads everything into PSRAM. */
static bool rg_load_rng1(FILE *fp, const char *path)
{
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

/* RNG2: whole-country graph on SD — load only the cells covering the map
 * centre as an active window in PSRAM (SD = memory, PSRAM = window).
 *
 * The file keeps nodes reordered by cell and edges sorted by (source node)
 * so each cell's nodes AND edges are contiguous ranges we can fseek + fread.
 * Loaded nodes get compact local ids; edges whose target is outside the
 * window are dropped (the window is big enough that boundary effects are
 * minor). */
static bool rg_load_rng2(FILE *fp, const char *path)
{
    uint32_t ver = 0, N = 0, E = 0;
    int32_t mnla = 0, mnlo = 0, mxla = 0, mxlo = 0;
    uint32_t cell = 0;
    uint16_t gw = 0, gh = 0;
    if (fread(&ver, 4, 1, fp) != 1 || fread(&N, 4, 1, fp) != 1 ||
        fread(&E, 4, 1, fp) != 1 || fread(&mnla, 4, 1, fp) != 1 ||
        fread(&mnlo, 4, 1, fp) != 1 || fread(&mxla, 4, 1, fp) != 1 ||
        fread(&mxlo, 4, 1, fp) != 1 || fread(&cell, 4, 1, fp) != 1 ||
        fread(&gw, 2, 1, fp) != 1 || fread(&gh, 2, 1, fp) != 1 ||
        ver != 1 || N == 0 || E == 0) {
        ESP_LOGE(TAG, "bad RNG2 header");
        fclose(fp);
        return false;
    }
    uint64_t nCells = (uint64_t)gw * gh;
    uint64_t latOff  = 40 + (nCells + 1) * 4;
    uint64_t lonOff  = latOff + (uint64_t)N * 4;
    uint64_t firstOff = lonOff + (uint64_t)N * 4;
    uint64_t toOff   = firstOff + (uint64_t)(N + 1) * 4;
    uint64_t wOff    = toOff + (uint64_t)E * 4;

    /* covered cells around the map centre */
    double cLat = centerLat, cLon = centerLon;
    double rad = 0.06;   /* deg (~13 km box) */
    int cx0 = (int)floor((cLon - rad - (double)mnlo / 1e7) / ((double)cell / 1e7));
    int cx1 = (int)floor((cLon + rad - (double)mnlo / 1e7) / ((double)cell / 1e7));
    int cy0 = (int)floor((cLat - rad - (double)mnla / 1e7) / ((double)cell / 1e7));
    int cy1 = (int)floor((cLat + rad - (double)mnla / 1e7) / ((double)cell / 1e7));
    if (cx0 < 0) { cx0 = 0; }
    if (cx1 >= (int)gw) { cx1 = gw - 1; }
    if (cy0 < 0) { cy0 = 0; }
    if (cy1 >= (int)gh) { cy1 = gh - 1; }
    int ncx = cx1 - cx0 + 1, ncy = cy1 - cy0 + 1;
    int nCovered = ncx * ncy;
    if (nCovered <= 0) {
        ESP_LOGW(TAG, "RNG2: map centre outside graph bbox");
        fclose(fp);
        return false;
    }

    /* read cellNodeFirst per covered ROW (contiguous cols per row) */
    uint32_t *cnf = (uint32_t *)heap_caps_malloc((uint32_t)(nCovered + ncy + 1) * 4,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!cnf) { ESP_LOGE(TAG, "RNG2 cnf alloc failed"); fclose(fp); return false; }
    {
        uint32_t *rowCnf = (uint32_t *)heap_caps_malloc((uint32_t)(ncx + 1) * 4,
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!rowCnf) { free(cnf); fclose(fp); ESP_LOGE(TAG, "RNG2 rowCnf alloc failed"); return false; }
        for (int r = 0; r < ncy; r++) {
            int cy = cy0 + r;
            uint64_t off = 40 + ((uint64_t)cy * gw + cx0) * 4;
            fseek(fp, (long)off, SEEK_SET);
            if (fread(rowCnf, 4, ncx + 1, fp) != (size_t)(ncx + 1)) {
                free(cnf); free(rowCnf); fclose(fp); ESP_LOGE(TAG, "RNG2 cnf read failed"); return false;
            }
            for (int c = 0; c <= ncx; c++)
                cnf[r * (ncx + 1) + c] = rowCnf[c];
        }
        free(rowCnf);
    }

    /* old-id -> local-id via the covered cells' node ranges (disjoint, sorted) */
    typedef struct { uint32_t start, count, base; } RngRange;
    RngRange *ranges = (RngRange *)heap_caps_malloc((uint32_t)nCovered * sizeof(RngRange),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ranges) { free(cnf); fclose(fp); ESP_LOGE(TAG, "RNG2 ranges alloc failed"); return false; }
    uint32_t M = 0, keptEdges = 0;
    for (int r = 0; r < ncy; r++) {
        for (int c = 0; c < ncx; c++) {
            uint32_t s = cnf[r * (ncx + 1) + c];
            uint32_t e = cnf[r * (ncx + 1) + c + 1];
            ranges[r * ncx + c].start = s;
            ranges[r * ncx + c].count = e - s;
            ranges[r * ncx + c].base = M;
            M += e - s;
        }
    }

    /* allocate local window arrays */
    g_lat   = (int32_t *)heap_caps_malloc((size_t)M * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_lon   = (int32_t *)heap_caps_malloc((size_t)M * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_first = (uint32_t *)heap_caps_malloc((size_t)(M + 1) * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_lat || !g_lon || !g_first) {
        ESP_LOGE(TAG, "RNG2 node alloc failed (M=%u)", (unsigned)M);
        rg_unload(); free(cnf); free(ranges); fclose(fp);
        return false;
    }

    /* temp buffers for one cell's first[] + edge slice */
    uint32_t *tFirst = (uint32_t *)heap_caps_malloc(4096 * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t *tTo    = (uint32_t *)heap_caps_malloc(65536 * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint16_t *tW     = (uint16_t *)heap_caps_malloc(65536 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tFirst || !tTo || !tW) {
        ESP_LOGE(TAG, "RNG2 temp alloc failed");
        rg_unload(); free(cnf); free(ranges); free(tFirst); free(tTo); free(tW);
        fclose(fp);
        return false;
    }

    /* pass 1: count the window's edge slice so g_to/g_w are WINDOW-sized
     * (allocating the whole-country E would OOM). */
    uint64_t totalEdges = 0;
    bool fail = false;
    for (int r = 0; r < ncy && !fail; r++) {
        for (int c = 0; c < ncx && !fail; c++) {
            uint32_t s = cnf[r * (ncx + 1) + c];
            uint32_t cnt = ranges[r * ncx + c].count;
            if (cnt == 0)
                continue;
            if (cnt + 1 > 4096) { fail = true; break; }
            fseek(fp, (long)(firstOff + (uint64_t)s * 4), SEEK_SET);
            if (fread(tFirst, 4, cnt + 1, fp) != cnt + 1) { fail = true; break; }
            totalEdges += (uint64_t)tFirst[cnt] - tFirst[0];
        }
    }
    if (fail || totalEdges == 0 || totalEdges > 8000000) {
        ESP_LOGE(TAG, "RNG2 edge-count pass failed (totalEdges=%llu)",
                 (unsigned long long)totalEdges);
        rg_unload(); free(cnf); free(ranges); free(tFirst); free(tTo); free(tW);
        fclose(fp);
        return false;
    }
    g_to = (uint32_t *)heap_caps_malloc((size_t)totalEdges * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_w  = (uint16_t *)heap_caps_malloc((size_t)totalEdges * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_to || !g_w) {
        ESP_LOGE(TAG, "RNG2 edge alloc failed (E=%llu)", (unsigned long long)totalEdges);
        rg_unload(); free(cnf); free(ranges); free(tFirst); free(tTo); free(tW);
        fclose(fp);
        return false;
    }

    g_first[0] = 0;
    uint32_t gE = 0;         /* kept edges so far */
    int32_t wMinLat = INT32_MAX, wMinLon = INT32_MAX, wMaxLat = INT32_MIN, wMaxLon = INT32_MIN;

    for (int r = 0; r < ncy && !fail; r++) {
        for (int c = 0; c < ncx && !fail; c++) {
            uint32_t s = cnf[r * (ncx + 1) + c];      /* old node id range start */
            uint32_t cnt = ranges[r * ncx + c].count; /* nodes in this cell */
            uint32_t base = ranges[r * ncx + c].base;
            if (cnt == 0)
                continue;

            /* node coords */
            fseek(fp, (long)(latOff + (uint64_t)s * 4), SEEK_SET);
            if (fread(g_lat + base, 4, cnt, fp) != cnt) { fail = true; break; }
            fseek(fp, (long)(lonOff + (uint64_t)s * 4), SEEK_SET);
            if (fread(g_lon + base, 4, cnt, fp) != cnt) { fail = true; break; }

            /* first[] for these nodes (cnt+1 entries) */
            if (cnt + 1 > 4096) { ESP_LOGE(TAG, "cell too big"); fail = true; break; }
            fseek(fp, (long)(firstOff + (uint64_t)s * 4), SEEK_SET);
            if (fread(tFirst, 4, cnt + 1, fp) != cnt + 1) { fail = true; break; }
            uint32_t e0 = tFirst[0];
            uint32_t e1 = tFirst[cnt];
            uint32_t eN = e1 - e0;
            if (eN > 65536) { ESP_LOGE(TAG, "cell edges too big"); fail = true; break; }
            fseek(fp, (long)(toOff + (uint64_t)e0 * 4), SEEK_SET);
            if (fread(tTo, 4, eN, fp) != eN) { fail = true; break; }
            fseek(fp, (long)(wOff + (uint64_t)e0 * 2), SEEK_SET);
            if (fread(tW, 2, eN, fp) != eN) { fail = true; break; }

            /* for each node in this cell, copy its edges, remapping targets */
            for (uint32_t j = 0; j < cnt && !fail; j++) {
                uint32_t loc = base + j;
                uint32_t row0 = tFirst[j] - e0, row1 = tFirst[j + 1] - e0;
                for (uint32_t k = row0; k < row1; k++) {
                    uint32_t tgt = tTo[k];
                    /* binary search target in covered ranges */
                    int lo = 0, hi = nCovered - 1, found = -1;
                    while (lo <= hi) {
                        int mid = (lo + hi) / 2;
                        if (tgt < ranges[mid].start) hi = mid - 1;
                        else if (tgt >= ranges[mid].start + ranges[mid].count) lo = mid + 1;
                        else { found = mid; break; }
                    }
                    if (found < 0)
                        continue;   /* target outside window -> drop edge */
                    uint32_t tl = ranges[found].base + (tgt - ranges[found].start);
                    g_to[gE] = tl;
                    g_w[gE] = tW[k];
                    gE++;
                }
                g_first[loc + 1] = gE;
            }

            /* window bbox (for the snap index) */
            for (uint32_t j = 0; j < cnt && !fail; j++) {
                int32_t la = g_lat[base + j], lo2 = g_lon[base + j];
                if (la < wMinLat) { wMinLat = la; }
                if (la > wMaxLat) { wMaxLat = la; }
                if (lo2 < wMinLon) { wMinLon = lo2; }
                if (lo2 > wMaxLon) { wMaxLon = lo2; }
            }
        }
    }

    free(tFirst); free(tTo); free(tW); free(cnf); free(ranges);
    if (fail) {
        ESP_LOGE(TAG, "RNG2 read failed");
        rg_unload(); fclose(fp);
        return false;
    }
    fclose(fp);
    g_N = M; g_E = gE;
    g_minlat = wMinLat; g_minlon = wMinLon; g_maxlat = wMaxLat; g_maxlon = wMaxLon;

    /* ---- tap-snap cell index on the window ---- */
    g_cellW = (uint16_t)(((double)(g_maxlon - g_minlon) / 1e7) / CELL_DEG) + 1;
    g_cellH = (uint16_t)(((double)(g_maxlat - g_minlat) / 1e7) / CELL_DEG) + 1;
    uint32_t cells = (uint32_t)g_cellW * g_cellH;
    g_cellFirst = (uint32_t *)heap_caps_calloc(cells + 1, 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_cellNode  = (uint32_t *)heap_caps_malloc((size_t)M * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_cellFirst || !g_cellNode) {
        ESP_LOGE(TAG, "RNG2 snap index alloc failed");
        rg_unload();
        return false;
    }
    for (uint32_t i = 0; i < M; i++)
        g_cellFirst[nodeCell((double)g_lat[i] / 1e7, (double)g_lon[i] / 1e7) + 1]++;
    for (uint32_t c = 0; c < cells; c++)
        g_cellFirst[c + 1] += g_cellFirst[c];
    {
        uint32_t *cur = (uint32_t *)heap_caps_malloc(cells * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memcpy(cur, g_cellFirst, cells * 4);
        for (uint32_t i = 0; i < M; i++) {
            uint32_t c = nodeCell((double)g_lat[i] / 1e7, (double)g_lon[i] / 1e7);
            g_cellNode[cur[c]++] = i;
        }
        free(cur);
    }
    g_loaded = true;

    size_t mb = ((size_t)M * 4 + M * 4 + (M + 1) * 4 + gE * 4 + gE * 2 +
                 cells * 4 + M * 4) / 1024 / 1024;
    ESP_LOGI(TAG, "window loaded %s: N=%u E=%u (%.1f MB) bbox %.5f..%.5f / %.5f..%.5f",
             path, (unsigned)M, (unsigned)gE, (double)mb,
             (double)g_minlat / 1e7, (double)g_maxlat / 1e7,
             (double)g_minlon / 1e7, (double)g_maxlon / 1e7);
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
