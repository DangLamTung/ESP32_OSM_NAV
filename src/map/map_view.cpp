/**
 * map_view.cpp — map state + tile rendering module.
 */
#include "map_view.h"
#include "app_config.h"
#include "display_panel.h"
#include <Arduino.h>
#include "esp_log.h"

static const char *TAG = "map";

OpenStreetMap osm;
LGFX_Sprite   mapSprite(&display);
double        centerLat = MAP_CENTER_LAT;
double        centerLon = MAP_CENTER_LON;
int           ZOOM      = ZOOM_DEFAULT;
bool          s_mapDirty = true;

static OpenStreetMap::TileMode s_mode = OpenStreetMap::TILE_AUTO;

bool map_init(void)
{
    osm.setSize(SCREEN_W, SCREEN_H);
    if (!osm.resizeTilesCache(TILE_CACHE_SLOTS))
    {
        ESP_LOGW(TAG, "could not allocate tile cache (check PSRAM)");
        return false;
    }
    ESP_LOGI(TAG, "tiles needed: %u (cache=%d)",
             osm.tilesNeeded(SCREEN_W, SCREEN_H), TILE_CACHE_SLOTS);
    return true;
}

bool map_fetch(void)
{
    uint32_t t0 = millis();
    bool ok = osm.fetchMap(mapSprite, centerLon, centerLat, ZOOM, 0);
    if (ok)
        ESP_LOGI(TAG, "map drawn (%lu ms)", (unsigned long)(millis() - t0));
    else
        ESP_LOGE(TAG, "fetchMap failed");
    return ok;
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

const char *map_mode_label(void)
{
    switch (s_mode)
    {
    case OpenStreetMap::TILE_SD_ONLY:  return "SD";
    case OpenStreetMap::TILE_NET_ONLY: return "NET";
    default:                           return "AUTO";
    }
}
