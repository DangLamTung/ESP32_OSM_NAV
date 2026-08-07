/**
 * map_view.h — map state + tile rendering module.
 * Owns the OSM fetcher, the map sprite, the view center/zoom and the
 * tile-source mode (AUTO / SD / NET).
 */
#ifndef MAP_VIEW_H_
#define MAP_VIEW_H_

#include <stdbool.h>
#include <stdint.h>
#include <LovyanGFX.hpp>
#include <OpenStreetMap-esp32.hpp>
#include "mercator.h"

extern OpenStreetMap osm;          /* vendored tile library */
extern LGFX_Sprite   mapSprite;    /* offscreen map sprite (PSRAM) */
extern double        centerLat;    /* view center (Bến Thành) */
extern double        centerLon;
extern int           ZOOM;         /* active zoom (buttons change it) */
extern bool          s_mapDirty;   /* set to force a full map redraw */

bool  map_init(void);               /* set size + tile cache (after display up) */
bool  map_fetch(void);              /* fetchMap into mapSprite (blocking) */
void  map_push(void);               /* pushSprite(0,0) */
void  map_pan(int dx, int dy);      /* pan by pixel delta (Mercator) */
void  map_zoom(int dir);            /* +1 / -1, clamped to [ZOOM_MIN, ZOOM_MAX] */
void  map_cycle_tile_mode(void);    /* AUTO -> SD -> NET -> AUTO */
const char *map_mode_label(void);   /* "AUTO" | "SD" | "NET" */

#endif // MAP_VIEW_H_
