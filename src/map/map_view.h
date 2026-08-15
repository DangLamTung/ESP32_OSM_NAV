/**
 * map_view.h — map state + tile rendering module.
 * Owns the OSM fetcher, the map sprite, the view center/zoom, the rotation
 * state and the tile-source mode (AUTO / SD / NET).
 *
 * Rotation model: the tile fetcher composes a square `mapWorld` sprite
 * (MAP_WORLD_SIZE) centered on the view, north-up. `map_render()` then rotates
 * that world into the visible 320x240 `mapSprite` (0° = fast blit of the
 * central window). The nav route + position marker are drawn onto `mapWorld`
 * in north-up map coordinates BEFORE rotation, so they rotate with the map;
 * the screen-fixed HUD (banner / weather / bottom bar) is drawn onto
 * `mapSprite` AFTER rotation.
 */
#ifndef MAP_VIEW_H_
#define MAP_VIEW_H_

#include <stdbool.h>
#include <stdint.h>
#include <LovyanGFX.hpp>
#include <OpenStreetMap-esp32.hpp>
#include "mercator.h"

extern OpenStreetMap osm;          /* vendored tile library */
extern LGFX_Sprite   mapSprite;    /* visible 320x240 sprite (PSRAM) */
extern LGFX_Sprite   mapWorld;     /* square north-up world sprite (PSRAM) */
extern double        centerLat;    /* view center (Bến Thành) */
extern double        centerLon;
extern int           ZOOM;         /* active zoom (buttons change it) */
extern bool          s_mapDirty;   /* set to force a full map redraw */

bool  map_init(void);               /* set size + tile cache + sprites (after display up) */
bool  map_fetch(void);              /* compose the world (throttled); returns true if world valid */
bool  map_world_changed(void);      /* last map_fetch recomposed the world (caller redraws route) */
void  map_force_recompose(void);    /* next map_fetch composes (route/batch/zoom/mode changed) */
double map_ref_lon(void);           /* centre the world is currently composed at */
double map_ref_lat(void);
void  map_preload(void);            /* warm the tile cache ahead of the car (throttled) */
void  map_render(void);             /* rotate mapWorld -> mapSprite (no tile fetch) */
void  map_push(void);               /* pushSprite(0,0) */
void  map_screen_to_latlon(int sx, int sy, double *lat, double *lon);  /* screen tap -> geo (north-up) */
void  map_latlon_to_screen(double lat, double lon, int *sx, int *sy);  /* geo -> screen (north-up) */
void  map_pan(int dx, int dy);      /* pan by pixel delta (Mercator) */
void  map_zoom(int dir);            /* +1 / -1, clamped to [ZOOM_MIN, ZOOM_MAX] */
void  map_cycle_tile_mode(void);    /* AUTO -> SD -> NET -> AUTO */
void  map_set_tile_mode(OpenStreetMap::TileMode m);  /* set + force refresh */
const char *map_mode_label(void);   /* "AUTO" | "SD" | "NET" */

/* ---- rotation quality (runtime toggle) ---- */
bool map_aa_enabled(void);          /* anti-aliased rotation active? */
void map_set_aa(bool on);           /* crisp (AA) vs fast (nearest-neighbour) */

/* ---- rotation ---- */
float map_rotation(void);               /* current (eased) rotation: 0 = north up, + = clockwise */
void  map_rotate(float deltaDeg);       /* nudge manual rotation (turns off heading-up) */
void  map_set_rotation(float deg);      /* absolute manual rotation */
bool  map_heading_up(void);             /* heading-up mode active? */
void  map_set_heading_up(bool on);      /* on: rotation tracks GPS heading */
void  map_apply_heading(int hdgDeg);    /* feed GPS heading (used in heading-up mode) */
/* Ease the current rotation one step toward the target. Returns true while
 * still gliding so the caller keeps redrawing until it settles. */
bool  map_ease_rotation(void);
/* True while the map rotation is still gliding toward its target (a turn is in
 * progress). Used to drop AA during big rotation to keep fps, then restore it
 * once the angle settles. */
bool  map_rotation_easing(void);
/* Record a GPS fix for smooth-follow interpolation (prev/cur + timestamps). */
void  map_set_fix(double lat, double lon);
/* Advance the view center along the interpolated fix path (interpolate between
 * the last two fixes, extrapolate past the current one). Returns true while the
 * car is moving so the caller keeps redrawing -> continuous, no "run-stop". */
bool  map_follow_interp(void);
/* Snap the view center to a fixed lat/lon (recenter button). */
void  map_center_on(double lat, double lon);

#endif // MAP_VIEW_H_
