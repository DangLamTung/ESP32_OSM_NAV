/**
 * OSM Tile Viewer with Capacitive-Touch Panning
 * Board: 2.8" IPS ESP32-S3 + ILI9341 (ES3C28P / ES3N28P)
 *
 * Modules (src/):
 *   app_config.h   - pins / wifi / geometry / zoom config
 *   display_panel  - ILI9341 + FT6336 init + low-power helpers
 *   map_view       - map state, tile fetch, pan/zoom, tile-source mode
 *   ui_controls    - corner buttons, BLE scan panel, nav overlay
 *   input_touch    - FT6336 gestures (tap / drag / long-press-to-sleep)
 *   power_mgr      - light sleep, wake on touch
 *   ble_nav / ble_scan / sd_card / sd_upload
 */
#include <Arduino.h>
#include <WiFi.h>
#include "app_config.h"
#include "display_panel.h"
#include "map_view.h"
#include "ui_controls.h"
#include "input_touch.h"
#include "power_mgr.h"
#include "ble_nav.h"
#include "ble_scan.h"
#include "sd_card.h"
#include "sd_upload.h"
#include "esp_log.h"
#include "esp_system.h"   /* esp_restart() */

static const char *TAG = "osm_idf";

/* fetch the current view into the sprite, draw the nav overlay, push to LCD */
static void drawMap()
{
    if (map_fetch())
    {
        ui_draw_nav_overlay(mapSprite, centerLon, centerLat, ZOOM);
        map_push();
    }
}

void setup()
{
    ESP_LOGI(TAG, "setup begin");
    Serial.begin(115200);
    ESP_LOGI(TAG, "serial ok");
    delay(200);
    Serial.println("\nOSM touch tile viewer (OpenStreetMap-esp32)");

    /* USB reader mode: if the PC sends the magic within the first few seconds,
     * stream files into /sdcard over COM9, then reboot into nav mode
     * (PC side: scripts/upload_tiles_serial.py). */
    if (sd_upload_listen(3000))
    {
        ESP_LOGI(TAG, "upload session finished - rebooting into nav mode");
        esp_restart();
    }

    bool initOK = display_panel_init();
    ESP_LOGI(TAG, "display.init returned %d", (int)initOK);
    Serial.printf("Display: 320x240\n");

    /* quick visual check: red screen for 1 s (confirms panel + backlight)
     * NOTE: done BEFORE any BLE init to isolate whether BLE corrupts _panel. */
    display.fillScreen(display.color565(255, 0, 0));
    Serial.println("RED fill test done");
    ESP_LOGI(TAG, "red fill test done");
    delay(800);

    bleNavBegin();                 /* BLE GATT server */
    bleScanBegin();                /* BLE scanner */
    bleScanStart();                /* background scan task */
    ESP_LOGI(TAG, "ble begin done");
    display.fillScreen(display.color565(32, 32, 128));   /* OSM background blue */

    /* SD/TF card (SDMMC 4-bit) mounted at /sdcard (FAT32 / exFAT) */
    if (sd_card_init())
        Serial.println("SD card ready at /sdcard");
    else
        Serial.println("SD card not detected (no card? wrong FS?)");

    map_init();

    /* arm wake-on-touch (FT6336 INT) regardless of WiFi outcome */
    power_mgr_init();

    /* connect WiFi */
    display.setTextColor(TFT_WHITE, display.color565(32, 32, 128));
    display.setCursor(20, 110);
    display.print("Connecting WiFi...");
    Serial.println("Connecting WiFi...");
    ESP_LOGI(TAG, "connecting WiFi %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40)
    {
        delay(500);
        tries++;
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        ESP_LOGI(TAG, "WiFi connected IP=%s", WiFi.localIP().toString().c_str());
        Serial.printf("\nWiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
        ESP_LOGW(TAG, "WiFi connect failed");
        display.fillScreen(TFT_BLACK);
        display.setCursor(20, 110);
        display.print("WiFi connect failed!");
        return;                     /* still runs offline from the SD card */
    }

    display.fillScreen(TFT_BLACK);
    display.setCursor(20, 110);
    display.print("Loading tiles...");
    Serial.println("Loading tiles...");
    ESP_LOGI(TAG, "loading tiles...");
    drawMap();
    ui_mark_redraw();
    Serial.println("Drag on the screen to pan the map.");
    ESP_LOGI(TAG, "map drawn, ready");
}

void loop()
{
    static uint32_t hb = 0;
    if ((++hb % 200) == 0) ESP_LOGI(TAG, "loop alive (mapDirty=%d)", (int)s_mapDirty);

    input_touch_poll();

    /* BLE nav updates -> redraw */
    if (navRouteDirty() || navManeuverDirty())
    {
        navRouteClearDirty();
        navManeuverClearDirty();
        s_mapDirty = true;
    }
    if (navPosDirty())
    {
        navPosClearDirty();
        const NavPos *p = navGetPos();
        if (p && p->valid)
        {
            /* auto-follow: recenter when the position drifts far from screen center */
            double wx = lon2wx(p->lon, ZOOM), wy = lat2wy(p->lat, ZOOM);
            double tlX = lon2wx(centerLon, ZOOM) - SCREEN_W / 2.0;
            double tlY = lat2wy(centerLat, ZOOM) - SCREEN_H / 2.0;
            int sx = (int)(wx - tlX), sy = (int)(wy - tlY);
            if (sx - SCREEN_W / 2 > 40 || sx - SCREEN_W / 2 < -40 ||
                sy - SCREEN_H / 2 > 40 || sy - SCREEN_H / 2 < -40)
            {
                centerLon = wx2lon(wx, ZOOM);
                centerLat = wy2lat(wy, ZOOM);
                s_mapDirty = true;
            }
        }
    }

    if (s_mapDirty)
    {
        s_mapDirty = false;
        drawMap();
        ui_mark_redraw();          /* map push covered the corner buttons */
    }

    /* BLE scan list overlay / corner buttons. Only re-draw when something
     * changed - NOT every loop - so the static UI does not continuously
     * rewrite the LCD (no flicker). */
    if (ui_scan_shown()) ui_draw_scan_overlay();
    else if (ui_needs_redraw())
    {
        ui_clear_redraw();
        ui_draw_buttons();
    }

    delay(10);
}
