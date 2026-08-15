/**
 * route_graph.h — real OSM road graph for offline routing.
 *
 * Loads the compact binary graph built by tools/build_routing_graph.py
 * (.rng, magic "RNG1") from the SD card into PSRAM, and runs A* on the real
 * road network (CSR adjacency, time weights from road-class speeds — the same
 * "car fastest" profile the NavBridge app uses).
 *
 * When no .rng file is present the app falls back to the synthetic test grid
 * in routing.cpp, so the board still works without a prepared SD card.
 */
#ifndef ROUTE_GRAPH_H_
#define ROUTE_GRAPH_H_

#include <stdbool.h>
#include <stdint.h>

/* RNG2 windowed-loading radius (~0.06 deg) in km, announced in the routing UI. */
#define ROUTE_WINDOW_RADIUS_KM 7

/* true if the loaded graph is an RNG2 window (whole-country SD file) rather
 * than a whole-file RNG1 — the window only covers the map centre at load. */
bool rg_is_windowed(void);

/* Load /sdcard/<path> routing graph into PSRAM (coords + CSR + snap cell
 * index). Call rg_astar_init() afterwards to allocate the A* working arrays
 * (do that AFTER freeing the synthetic grid so they fit together). */
bool rg_load(const char *path);

/* Allocate the A* working arrays (PSRAM). Returns false on OOM. */
bool rg_astar_init(void);

/* Free the graph + A* arrays (fall back to the synthetic grid). */
void rg_unload(void);

/* true if a real graph is loaded (routing.cpp then uses rg_route). */
bool rg_loaded(void);

/* number of graph nodes (0 until loaded). */
uint32_t rg_node_count(void);

/* lat/lon (decimal degrees) of a node index. */
double rg_lat(uint32_t node);
double rg_lon(uint32_t node);

/* graph bbox (decimal degrees). Only valid when rg_loaded(). */
void rg_bbox(double *minLat, double *minLon, double *maxLat, double *maxLon);

/* Nearest graph node to (lat,lon), searched over a maxRadDeg box. Returns -1
 * if the graph is not loaded or nothing within range. */
int rg_nearest(double lat, double lon, double maxRadDeg);

/* A* on the real graph from the snapped nearest node of (sLat,sLon) to that
 * of (tLat,tLon). Fills outLat[]/outLon[] (0..n-1) and returns the point
 * count (0 on failure, 1 if start==stop). Optionally reports route distance
 * in metres and computation time in ms. */
int rg_route(double sLat, double sLon, double tLat, double tLon,
             double *outLat, double *outLon, int maxPts,
             double *distM, uint32_t *msOut);

#endif // ROUTE_GRAPH_H_
