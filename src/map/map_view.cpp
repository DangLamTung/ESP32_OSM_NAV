/**
 * map_view.cpp — map state + tile rendering module.
 */
#include "map_view.h"
#include "app_config.h"
#include "display_panel.h"
#include "ble_nav.h"   /* navGetPos() for the preload lookahead direction */
#include <Arduino.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "map";

OpenStreetMap osm;
LGFX_Sprite   mapSprite(&display);   /* visible 320x240 view (PSRAM) */
LGFX_Sprite   mapWorld(&display);    /* square north-up world (PSRAM)   */
double        centerLat = MAP_CENTER_LAT;
double        centerLon = MAP_CENTER_LON;
int           ZOOM      = ZOOM_DEFAULT;
bool          s_mapDirty = true;

/* ---- rotation state ----
 * s_mapRotation: 0 = north up, positive = clockwise (matches LovyanGFX
 * pushRotateZoomWithAA, verified against its make_rotation_matrix). The world
 * sprite is composed north-up, then rotated into mapSprite by this angle.
 *
 * Smooth (eased) rotation: s_mapRotation is the CURRENT displayed angle and
 * glides toward s_targetRotation by ROTATION_EASE_FACTOR on each redraw (see
 * map_ease_rotation). map_apply_heading()/map_rotate() only move the target,
 * so turning the car (or tapping rotate) sweeps the map like Google Maps
 * instead of snapping. The main loop keeps redrawing while easing is active. */
static float s_mapRotation   = 0.0f;
static float s_targetRotation = 0.0f;
static bool  s_headingUp     = false;
/* Rotation quality: anti-aliased (crisp, heavier) vs nearest-neighbour (fast).
 * Default comes from MAP_AA_ROTATION; the settings panel can toggle it. */
static bool  s_aaRotation    = (MAP_AA_ROTATION != 0);

/* ---- throttled world composition ----
 * Composing the 448x448 world from 4 tiles costs ~28ms (PSRAM bandwidth), so
 * it dominates every frame. But the car moves only ~0.2px/frame at z15, so we
 * recompose ONLY when the view centre has drifted more than COMPOSE_THRESH_PX
 * from the point the world was last composed at, and scroll the visible window
 * by a blit offset (s_offX/Y) in between. The world has 64px horizontal / 104px
 * vertical margin, so this is invisible and cuts the effective compose cost ~8x.
 * The car marker is drawn screen-fixed (ui_draw_nav_marker) so nothing ghosts. */
#define COMPOSE_THRESH_PX 48
static double s_refLat = 1e9, s_refLon = 1e9;   /* invalid -> first fetch composes */
static int    s_offX = 0, s_offY = 0;           /* view centre offset from ref, world px */
static bool   s_worldChanged = false;
static bool   s_forceRecompose = false;

bool map_world_changed(void) { return s_worldChanged; }
void map_force_recompose(void) { s_forceRecompose = true; s_mapDirty = true; }
double map_ref_lon(void) { return s_refLon; }
double map_ref_lat(void) { return s_refLat; }

/* No-WiFi test build: default to SD-only tiles so the map never touches the
 * network stack (TILE_AUTO would fall back to HTTP when an SD tile is missing,
 * and the WiFi/lwIP stack is disabled here -> hard assert). */
static OpenStreetMap::TileMode s_mode = OpenStreetMap::TILE_SD_ONLY;

static bool createWorldSprite(void)
{
    if (mapWorld.getBuffer() && mapWorld.width() == MAP_WORLD_SIZE)
        return true;
    mapWorld.deleteSprite();
    mapWorld.setPsram(true);
    mapWorld.setColorDepth(lgfx::rgb565_2Byte);
    if (!mapWorld.createSprite(MAP_WORLD_SIZE, MAP_WORLD_SIZE) || !mapWorld.getBuffer())
    {
        ESP_LOGE(TAG, "could not allocate %dx%d world sprite (PSRAM)",
                 MAP_WORLD_SIZE, MAP_WORLD_SIZE);
        return false;
    }
    mapWorld.fillSprite(OSM_BGCOLOR);
    ESP_LOGI(TAG, "world sprite: %dx%d (%u KB PSRAM)",
             MAP_WORLD_SIZE, MAP_WORLD_SIZE,
             (unsigned)((size_t)MAP_WORLD_SIZE * MAP_WORLD_SIZE * 2 / 1024));
    return true;
}

static bool createViewSprite(void)
{
    if (mapSprite.getBuffer() && mapSprite.width() == SCREEN_W)
        return true;
    mapSprite.deleteSprite();
    mapSprite.setPsram(true);
    mapSprite.setColorDepth(lgfx::rgb565_2Byte);
    if (!mapSprite.createSprite(SCREEN_W, SCREEN_H) || !mapSprite.getBuffer())
    {
        ESP_LOGE(TAG, "could not allocate view sprite (PSRAM)");
        return false;
    }
    return true;
}

bool map_init(void)
{
    if (!createWorldSprite() || !createViewSprite())
        return false;
    osm.setSize(MAP_WORLD_SIZE, MAP_WORLD_SIZE);
    osm.setTileMode(s_mode);   /* apply SD-only at boot so the map never touches the network */
    if (!osm.resizeTilesCache(TILE_CACHE_SLOTS))
    {
        ESP_LOGW(TAG, "could not allocate tile cache (check PSRAM)");
        return false;
    }
    ESP_LOGI(TAG, "tiles needed: %u (cache=%d)",
             osm.tilesNeeded(MAP_WORLD_SIZE, MAP_WORLD_SIZE), TILE_CACHE_SLOTS);
    return true;
}

bool map_fetch(void)
{
    uint32_t t0 = millis();
    /* pixel offset of the current view centre from the composed reference */
    double dx = lon2wx(centerLon, ZOOM) - lon2wx(s_refLon, ZOOM);
    double dy = lat2wy(centerLat, ZOOM) - lat2wy(s_refLat, ZOOM);
    bool recompose = s_forceRecompose || fabs(dx) > COMPOSE_THRESH_PX || fabs(dy) > COMPOSE_THRESH_PX;
    s_forceRecompose = false;

    if (recompose)
    {
        /* Compose the north-up world sprite (centered on the view). The caller
         * draws the route onto mapWorld AFTER this returns and BEFORE
         * map_render(), so it rotates with the map. Rendering is a separate
         * step so a heading change can re-rotate without re-fetching. */
        bool ok = osm.fetchMap(mapWorld, centerLon, centerLat, ZOOM, 0);
        s_refLat = centerLat; s_refLon = centerLon;
        s_offX = 0; s_offY = 0;
        s_worldChanged = true;
        if (ok)
            ESP_LOGI(TAG, "map fetched (%lu ms)", (unsigned long)(millis() - t0));
        else
            ESP_LOGE(TAG, "fetchMap failed");
        return ok;
    }

    /* reuse the composed world; scroll the view via the blit offset */
    s_offX = (int)lround(dx);
    s_offY = (int)lround(dy);
    s_worldChanged = false;
    return true;
}

/* Rotate the north-up mapWorld into the visible mapSprite. North-up is the
 * common case (rotation 0) and uses a cheap memcpy blit of the central
 * 320x240 window; any other angle uses LovyanGFX's AA rotation at 1:1 scale,
 * which keeps the map scale identical (the square world's half-diagonal 317px
 * clears the 200px window corner at every angle, so no blank corners). */
void map_render(void)
{
    if (!mapWorld.getBuffer() || !mapSprite.getBuffer())
        return;
    uint32_t t0 = millis();

    if (fabsf(s_mapRotation) < 0.5f)
    {
        /* Blit the 320x240 window centred on the VIEW centre (world centre +
         * the s_offX/Y scroll offset) to mapSprite. pushSprite places the
         * source's (0,0) at (x,y) in the destination, so the offsets are
         * NEGATIVE: -(world centre - half window - offset). */
        mapWorld.pushSprite(&mapSprite,
                            -((MAP_WORLD_SIZE - SCREEN_W) / 2) - s_offX,
                            -((MAP_WORLD_SIZE - SCREEN_H) / 2) - s_offY);
    }
    else
    {
        /* Hybrid quality: use anti-aliasing for a crisp settled map, but drop
         * to nearest-neighbour WHILE the map is actively turning (big rotation)
         * so the turn keeps full fps. Returns to AA once the angle settles. */
        bool crisp = s_aaRotation && !map_rotation_easing();
        /* shift the rotation anchor so the view stays centred on the car even
         * though the world is composed at the reference centre (not current) */
        float A = s_mapRotation * M_PI / 180.0f;
        int ax = SCREEN_W / 2 - (int)lroundf(s_offX * cosf(A) - s_offY * sinf(A));
        int ay = SCREEN_H / 2 - (int)lroundf(s_offX * sinf(A) + s_offY * cosf(A));
        if (crisp)
        {
            /* Anti-aliased rotation: crisp text/labels when the map is settled. */
            mapWorld.pushRotateZoomWithAA(&mapSprite, ax, ay,
                                          s_mapRotation, 1.0f, 1.0f, OSM_BGCOLOR);
        }
        else
        {
            /* Nearest-neighbour: fast during turns (jaggy, but keeps fps). */
            mapWorld.pushRotateZoom(&mapSprite, ax, ay,
                                    s_mapRotation, 1.0f, 1.0f, OSM_BGCOLOR);
        }
    }

    /* OSM/CARTO attribution: composeMap drew it at the world's bottom edge,
     * which is cropped out of the view at any rotation — redraw it on the
     * visible sprite bottom-left so it stays legal. */
    if (osm.getAttribution())
    {
        mapSprite.setTextColor(TFT_WHITE, OSM_BGCOLOR);
        mapSprite.setTextSize(0.7f);
        mapSprite.drawString(osm.getAttribution(), 2, mapSprite.height() - 8, &DejaVu9Modded);
        mapSprite.setTextSize(1.0f);
        mapSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    ESP_LOGD(TAG, "render rot=%.1f (%lu ms)", s_mapRotation,
             (unsigned long)(millis() - t0));
}

void map_push(void)
{
    mapSprite.pushSprite(0, 0);
}

void map_pan(int dx, int dy)
{
    if (!dx && !dy) return;
    /* longitude: linear in tile units (1 tile = 256 px) */
    double txx = (centerLon + 180.0) / 360.0 * (1 << ZOOM) - (double)dx / 256.0;
    centerLon = txx / (1 << ZOOM) * 360.0 - 180.0;
    /* latitude: invert the mercator projection */
    double latRad = centerLat * M_PI / 180.0;
    double tyy = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / M_PI) / 2.0 *
                     (1 << ZOOM) -
                 (double)dy / 256.0;
    double n = M_PI - 2.0 * M_PI * tyy / (1 << ZOOM);
    centerLat = 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
    s_mapDirty = true;
}

void map_zoom(int dir)
{
    int nz = ZOOM + dir;
    if (nz < ZOOM_MIN || nz > ZOOM_MAX) return;
    ZOOM = nz;
    s_forceRecompose = true;   /* pixel scale changed -> must recompose */
    s_mapDirty = true;
    Serial.printf("[zoom] %d\n", ZOOM);
}

void map_cycle_tile_mode(void)
{
    switch (s_mode)
    {
    case OpenStreetMap::TILE_SD_ONLY:  s_mode = OpenStreetMap::TILE_NET_ONLY; break;
    case OpenStreetMap::TILE_NET_ONLY: s_mode = OpenStreetMap::TILE_AUTO;     break;
    default:                           s_mode = OpenStreetMap::TILE_SD_ONLY;  break;
    }
    osm.setTileMode(s_mode);
    /* NOTE: no full-map refresh here - the new mode applies on the next
     * pan/zoom, so the button just flips its label instantly (no re-decode). */
    Serial.printf("[src] mode=%s\n", map_mode_label());
}

void map_set_tile_mode(OpenStreetMap::TileMode m)
{
    s_mode = m;
    osm.setTileMode(m);
    s_forceRecompose = true;   /* force a re-fetch so missing tiles load right away */
    s_mapDirty = true;
    Serial.printf("[src] mode=%s\n", map_mode_label());
}

const char *map_mode_label(void)
{
    switch (s_mode)
    {
    case OpenStreetMap::TILE_SD_ONLY:  return "SD";
    case OpenStreetMap::TILE_NET_ONLY: return "NET";
    default:                           return "AUTO";
    }
}

/* ---- rotation ---- */

float map_rotation(void)
{
    return s_mapRotation;
}

bool map_aa_enabled(void) { return s_aaRotation; }
void map_set_aa(bool on)
{
    s_aaRotation = on;
    s_mapDirty = true;
    Serial.printf("[rot] aa=%d\n", (int)on);
}

static float wrap360(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

/* Nudge the TARGET by delta; the current rotation glides toward it. */
void map_set_rotation(float deg)
{
    s_targetRotation = wrap360(deg);
    s_mapDirty = true;
}

void map_rotate(float deltaDeg)
{
    if (s_headingUp)
    {
        s_headingUp = false;         /* manual rotate hands control back to the user */
        Serial.println("[rot] manual (heading-up off)");
    }
    s_targetRotation = wrap360(s_targetRotation + deltaDeg);
    s_mapDirty = true;
    Serial.printf("[rot] target %.1f deg\n", s_targetRotation);
}

bool map_heading_up(void)
{
    return s_headingUp;
}

void map_set_heading_up(bool on)
{
    s_headingUp = on;
    if (!on)
    {
        s_targetRotation = 0.0f;     /* back to north-up */
        s_mapRotation    = 0.0f;
    }
    s_mapDirty = true;
    Serial.printf("[rot] heading-up %s\n", on ? "on" : "off");
}

/* In heading-up mode the map rotates so the direction of travel (GPS heading,
 * compass degrees clockwise from north) always points up on screen. The world
 * is north-up, so screen-up = north + rotation; we need heading + rotation =
 * 0 -> rotation = (360 - heading) % 360. Only the target moves; the display
 * glides to it. Ignored unless heading-up is on.
 * The raw heading is low-pass filtered (EMA) first so small fix-to-fix wobble
 * doesn't make the rotation target jitter. */
static float  s_smHdg  = 0.0f;
static bool   s_hdgInit = false;

void map_apply_heading(int hdgDeg)
{
    if (!s_headingUp)
        return;

    float h = (float)hdgDeg;
    if (!s_hdgInit)
    {
        s_smHdg = h;
        s_hdgInit = true;
    }
    else
    {
        float d = fmodf(h - s_smHdg + 540.0f, 360.0f) - 180.0f;
        s_smHdg = wrap360(s_smHdg + d * HEADING_LP_ALPHA);
    }

    float r = wrap360(360.0f - s_smHdg);
    /* deadband: ignore small heading wobble (GPS/sim noise) so the map does
     * not micro-rotate every fix — only swing on real turns. */
    if (fabsf(r - s_targetRotation) > HEADING_DEADBAND_DEG)
    {
        s_targetRotation = r;
        s_mapDirty = true;
    }
}

/* Advance the current rotation one step toward the target along the shortest
 * arc (wrap-aware). Returns true while still gliding so the caller keeps
 * redrawing; returns false once settled (and snaps exactly to the target). */
bool map_rotation_easing(void)
{
    float d = fmodf(s_targetRotation - s_mapRotation + 540.0f, 360.0f) - 180.0f;
    return fabsf(d) >= ROTATION_SETTLE_DEG;
}

bool map_ease_rotation(void)
{
    if (!map_rotation_easing())
    {
        s_mapRotation = s_targetRotation;
        return false;
    }
    float d = fmodf(s_targetRotation - s_mapRotation + 540.0f, 360.0f) - 180.0f;
    s_mapRotation = wrap360(s_mapRotation + d * ROTATION_EASE_FACTOR);
    return true;
}

/* ---- smooth follow: Catmull-Rom spline through the GPS fixes ----
 * The old linear-interp + hard-extrapolate model had a velocity discontinuity
 * at EVERY fix and the extrapolation cap briefly FROZE the camera before the
 * next fix -> "speed up, stop, start". Instead we fit a Catmull-Rom spline
 * through the last fixes: the view centre passes through each fix with
 * CONTINUOUS velocity (C1) and, past the newest fix, continues smoothly along
 * the spline (no freeze). On straight roads the map glides; at a real turn the
 * direction change is smooth, not a jump. */
#define FIX_RING_MAX 4
static struct { double lat, lon; uint32_t t; } s_fixRing[FIX_RING_MAX];
static int s_fixRingN = 0;   /* how many fixes stored (grows to FIX_RING_MAX) */

void map_set_fix(double lat, double lon)
{
    /* ring: index 0 = newest. Shift, then insert the new fix. */
    if (s_fixRingN < FIX_RING_MAX) s_fixRingN++;
    for (int i = s_fixRingN - 1; i > 0; i--) s_fixRing[i] = s_fixRing[i - 1];
    s_fixRing[0].lat = lat;
    s_fixRing[0].lon = lon;
    s_fixRing[0].t   = millis();
}

bool map_follow_interp(void)
{
    if (s_fixRingN < 2) return false;   /* need two fixes to interpolate */

    const auto &p1 = s_fixRing[1];      /* segment start (second-newest fix) */
    const auto &p2 = s_fixRing[0];      /* segment end   (newest fix) */
    double seg = (double)(p2.t - p1.t); /* ms across the segment */
    if (seg <= 0.0) { centerLat = p2.lat; centerLon = p2.lon; return false; }

    uint32_t now = millis();
    double u = (double)(now - p1.t) / seg;
    if (u < 0.0) u = 0.0;
    if (u > 2.0) u = 2.0;               /* extrapolate at most 2 fix intervals */

    /* control points: p0 before the segment (flat if we don't have it yet),
     * p3 = virtual extension continuing the segment — this is what keeps the
     * extrapolation past the newest fix smooth instead of freezing. */
    double p0lat = (s_fixRingN >= 3) ? s_fixRing[2].lat : p1.lat;
    double p0lon = (s_fixRingN >= 3) ? s_fixRing[2].lon : p1.lon;
    double p3lat = p2.lat + (p2.lat - p1.lat);
    double p3lon = p2.lon + (p2.lon - p1.lon);

    /* uniform Catmull-Rom: p(u) between p1 and p2 (C1 continuous velocity) */
    double u2 = u * u, u3 = u2 * u;
    double lat = 0.5 * ((2.0 * p1.lat) + (-p0lat + p2.lat) * u +
                        (2.0 * p0lat - 5.0 * p1.lat + 4.0 * p2.lat - p3lat) * u2 +
                        (-p0lat + 3.0 * p1.lat - 3.0 * p2.lat + p3lat) * u3);
    double lon = 0.5 * ((2.0 * p1.lon) + (-p0lon + p2.lon) * u +
                        (2.0 * p0lon - 5.0 * p1.lon + 4.0 * p2.lon - p3lon) * u2 +
                        (-p0lon + 3.0 * p1.lon - 3.0 * p2.lon + p3lon) * u3);

    /* camera deadband: only commit once the spline position has moved past
     * CAMERA_DEADBAND_PX of map pixels (holds rock-steady for tiny drift). */
    double cdx = lon2wx(lon, ZOOM) - lon2wx(centerLon, ZOOM);
    double cdy = lat2wy(lat, ZOOM) - lat2wy(centerLat, ZOOM);
    if (cdx * cdx + cdy * cdy >= CAMERA_DEADBAND_PX * CAMERA_DEADBAND_PX)
    {
        centerLat = lat;
        centerLon = lon;
    }

    /* keep redrawing while the car is moving (recent fixes differ AND the feed
     * is fresh — stop once the position stream goes quiet) */
    bool moving = (s_fixRing[0].lat != s_fixRing[1].lat ||
                   s_fixRing[0].lon != s_fixRing[1].lon) &&
                  (now - s_fixRing[0].t) < 1500;
    return moving;
}

/* Recenter button: snap the view straight to a lat/lon (the car). */
void map_center_on(double lat, double lon)
{
    centerLat = lat;
    centerLon = lon;
    map_set_fix(lat, lon);   /* restart interpolation from the car */
    s_forceRecompose = true;
    s_mapDirty = true;
}

/* ---- tile preload: warm the cache AHEAD of the car so the per-frame
 *      map_fetch() only ever does cache hits (no SD read mid-frame -> no
 *      leading-edge stutter). The lookahead distance scales with speed so a
 *      fast drive warms further ahead; throttled to ~4 Hz so it only does real
 *      work when the car is actually crossing into new tiles. ---- */
void map_preload(void)
{
    static uint32_t s_lastMS = 0;
    uint32_t now = millis();
    if (now - s_lastMS < 250) return;   /* ~4 Hz is plenty */
    s_lastMS = now;

    const NavPos *p = navGetPos();
    bool haveFix = (p && p->valid);
    double hdg = haveFix ? (double)p->hdg : 0.0;
    double spd = haveFix ? (double)p->spd : 0.0;   /* km/h */

    /* how far ahead to warm: 3 s of travel, clamped 400 m..2 km (z15 tile ~1.2 km) */
    double look = spd / 3.6 * 3.0;
    if (look < 400.0)  look = 400.0;
    if (look > 2000.0) look = 2000.0;

    double h = hdg * M_PI / 180.0;
    double cosLat = cos(centerLat * M_PI / 180.0);
    double dLat = look * cos(h) / 111320.0;
    double dLon = look * sin(h) / (111320.0 * (cosLat > 0.05 ? cosLat : 1.0));
    osm.prefetchTiles(centerLon + dLon, centerLat + dLat, ZOOM);
}
