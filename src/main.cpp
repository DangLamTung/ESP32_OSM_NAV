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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "app_config.h"
#include "display_panel.h"
#include "map_view.h"
#include "ui_controls.h"
#include "input_touch.h"
#include "power_mgr.h"
#include "ble_nav.h"
#include "ble_scan.h"
#include "gps_ublox.h"
#include "sd_card.h"
#include "sd_upload.h"
#include "esp_log.h"
#include "esp_system.h"   /* esp_restart() */
#include "esp_timer.h"    /* esp_timer_get_time() — perf measurement */
#include "esp_heap_caps.h" /* heap_caps_get_free_size() — RAM leak monitoring */

static const char *TAG = "osm_idf";

/* Perf measurement: rolling sums of per-stage draw times + full-frame time.
 * Every PERF_N_FRAMES frames a summary is logged so CPU usage can be read on
 * the serial monitor (e.g. `idf.py monitor`). */
#define PERF_N_FRAMES 30
static uint32_t s_pfFrames = 0;
static uint32_t s_pfFrame = 0, s_pfFetch = 0, s_pfRoute = 0, s_pfRender = 0,
                s_pfHud = 0, s_pfBtns = 0, s_pfPush = 0;

/* Frame pacing: cap redraws to a STEADY cadence so frames arrive evenly (an
 * uneven cadence makes the 1-2px scroll steps land at irregular times, which
 * reads as jitter). A full frame is ~32ms (render ~11 + push ~19 + misc), so
 * 35ms gives an even ~28fps. When a frame is due we keep the loop tight
 * (delay 1ms) and let pacing control the cadence instead of the old delay(10). */
#define FRAME_PERIOD_MS 35
static uint32_t s_lastFrameMS = 0;

static void perfReport(void)
{
    if (s_pfFrames < PERF_N_FRAMES) return;
    ESP_LOGI("perf", "avg %.1fms %.1ffps | fetch %.2f route %.2f render %.2f "
                     "hud %.2f btns %.2f push %.2f | rot %.0f° | stack hw %u",
             s_pfFrame / (PERF_N_FRAMES * 1000.0),
             (PERF_N_FRAMES * 1000000.0) / (double)s_pfFrame,
             s_pfFetch / (PERF_N_FRAMES * 1000.0),
             s_pfRoute / (PERF_N_FRAMES * 1000.0),
             s_pfRender / (PERF_N_FRAMES * 1000.0),
             s_pfHud / (PERF_N_FRAMES * 1000.0),
             s_pfBtns / (PERF_N_FRAMES * 1000.0),
             s_pfPush / (PERF_N_FRAMES * 1000.0),
             (double)map_rotation(),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    s_pfFrames = s_pfFrame = s_pfFetch = s_pfRoute = 0;
    s_pfRender = s_pfHud = s_pfBtns = s_pfPush = 0;
}

/* fetch the current view, draw the overlays, push to LCD.
 * Layering (see map_view.h): map tiles -> mapWorld (north-up), route + position
 * marker drawn on mapWorld, then map_render() rotates mapWorld -> mapSprite;
 * the screen-fixed HUD is drawn on mapSprite afterwards so it stays upright
 * while the map turns.
 *
 * Smooth rotation: after each draw, map_ease_rotation() advances the displayed
 * angle one step toward the target; while it's still gliding we re-arm
 * s_mapDirty so the next loop iteration renders the next eased frame, giving a
 * fluid Google-Maps-style sweep when the car turns. */
static void drawMap()
{
    uint32_t t0 = esp_timer_get_time();

    /* SIMPLE mode = text-only navigation: NO map fetch/render — just the
     * guidance text over a solid background (the route is still cached but
     * not drawn, and no tiles are loaded). */
    if (ui_nav_mode() == UI_MODE_SIMPLE)
    {
        mapSprite.fillScreen(0x2104);
        ui_draw_nav_hud(mapSprite);   /* drawTextOnlyHUD fills + draws text */
        ui_draw_buttons(&mapSprite);
        map_push();
        s_pfFrame += (esp_timer_get_time() - t0);
        if (++s_pfFrames >= PERF_N_FRAMES) perfReport();
        return;
    }

    /* ALWAYS render + push even if map_fetch() failed to recompose (rare):
     * otherwise a failed fetch would leave the previous frame — e.g. the
     * solid SIMPLE screen — frozen on the LCD with no retry until the next
     * dirty event. The world sprite stays valid, so we just redraw it. */
    map_fetch();
    uint32_t t1 = esp_timer_get_time();
    /* redraw the route only when the world was recomposed (it's static on
     * the world between composes; the car marker is drawn screen-fixed) */
    if (map_world_changed())
        ui_draw_nav_route(mapWorld, map_ref_lon(), map_ref_lat(), ZOOM);
    uint32_t t2 = esp_timer_get_time();
    map_render();
    uint32_t t3 = esp_timer_get_time();
    ui_draw_nav_marker(mapSprite);
    ui_draw_nav_hud(mapSprite);
    uint32_t t4 = esp_timer_get_time();
    ui_draw_buttons(&mapSprite);   /* buttons composed into the frame -> no flicker */
    uint32_t t5 = esp_timer_get_time();
    map_push();
    uint32_t t6 = esp_timer_get_time();

    s_pfFetch  += t1 - t0;
    s_pfRoute  += t2 - t1;
    s_pfRender += t3 - t2;
    s_pfHud    += t4 - t3;
    s_pfBtns   += t5 - t4;
    s_pfPush   += t6 - t5;
    s_pfFrame  += t6 - t0;
    if (++s_pfFrames >= PERF_N_FRAMES) perfReport();

    if (map_ease_rotation())
        s_mapDirty = true;   /* keep gliding on the next loop iteration */
}

/* Boot render benchmark: renders a few frames north-up, then rotated with AA,
 * then rotated with nearest-neighbour, and logs the per-stage timings. Used to
 * see exactly where the frame time goes (fetch / route / render / hud / btns).
 * map_push() is SKIPPED for these frames so there is no visible flash at boot;
 * push cost is timed separately. */
static void benchFrames(const char *label, int n)
{
    uint32_t accF = 0, accR = 0, accRd = 0, accH = 0, accB = 0, accT = 0;
    for (int i = 0; i < n; i++)
    {
        uint32_t t0 = esp_timer_get_time();
        if (!map_fetch()) break;
        uint32_t t1 = esp_timer_get_time();
        if (map_world_changed())
            ui_draw_nav_route(mapWorld, map_ref_lon(), map_ref_lat(), ZOOM);
        uint32_t t2 = esp_timer_get_time();
        map_render();
        uint32_t t3 = esp_timer_get_time();
        ui_draw_nav_marker(mapSprite);
        ui_draw_nav_hud(mapSprite);
        uint32_t t4 = esp_timer_get_time();
        ui_draw_buttons(&mapSprite);
        uint32_t t5 = esp_timer_get_time();
        accF += t1 - t0; accR += t2 - t1; accRd += t3 - t2;
        accH += t4 - t3; accB += t5 - t4; accT += t5 - t0;
    }
    ESP_LOGI("bench", "%s: avg %.1fms (%.0ffps no-push) | fetch %.2f route %.2f render %.2f hud %.2f btns %.2f",
             label, accT / (n * 1000.0), n * 1000000.0 / (double)accT,
             accF / (n * 1000.0), accR / (n * 1000.0), accRd / (n * 1000.0),
             accH / (n * 1000.0), accB / (n * 1000.0));
}

void setup()
{
    ESP_LOGI(TAG, "setup begin");
    Serial.begin(115200);
    ESP_LOGI(TAG, "serial ok");
    delay(200);
    Serial.println("\nOSM touch tile viewer (OpenStreetMap-esp32)");

    /* USB reader mode: if the PC sends the magic within the first second,
     * stream files into /sdcard over COM9, then reboot into nav mode
     * (PC side: scripts/upload_tiles_serial.py). 1s window keeps boot fast
     * when no upload is requested; raise it if the upload tool needs more. */
    if (sd_upload_listen(1000))
    {
        ESP_LOGI(TAG, "upload session finished - rebooting into nav mode");
        esp_restart();
    }

    bool initOK = display_panel_init();
    ESP_LOGI(TAG, "display.init returned %d", (int)initOK);
    Serial.printf("Display: 320x240\n");

    /* Mount SD early so the splash can come from the card, then show it
     * (splash.png / cat.png from SD, else a drawn cat). This replaces the old
     * red diagnostic fill — that delay was not needed for normal operation. */
    if (sd_card_init())
    {
        Serial.println("SD card ready at /sdcard");
        ui_load_icon_sd("/sdcard/icon/nav_arrow.png");
    }
    else
        Serial.println("SD card not detected (no card? wrong FS?)");
    ui_show_splash();

    /* NVS: wipe + re-init at boot. Bluedroid SMP was crashing on corrupt bond
     * data in NVS (LoadProhibited in nvs_open_from_partition). Nothing needs
     * persisting here (WiFi removed, web client reconnects fresh), so erase. */
    nvs_flash_erase();
    ESP_LOGI(TAG, "nvs init: %s", esp_err_to_name(nvs_flash_init()));

    bleNavBegin();                 /* BLE GATT server */
    /* Vestigial background BLE scanner removed: the scan overlay/button were
     * dropped from the UI, and the continuous ~99%-duty active scan stressed
     * the NimBLE controller (intermittent Core-0 LoadProhibited crash). */
    ESP_LOGI(TAG, "ble begin done");

    /* U-Blox GPS (NMEA over UART). Optional: if no module is wired, the reader
     * task just idles and the broadcast stays off. */
    gps_init();
    navSetGpsBroadcast(GPS_BROADCAST_DEFAULT);

    map_init();

    /* arm wake-on-touch (FT6336 INT) regardless of WiFi outcome */
    power_mgr_init();

    /* WiFi disabled for test builds — the display runs fully offline (SD tiles + BLE) */
    (void)WIFI_SSID; (void)WIFI_PASS;

    /* Stop the animated splash task, then let the first map draw replace it. */
    ui_splash_stop();
    Serial.println("Loading tiles...");
    ESP_LOGI(TAG, "loading tiles...");
    drawMap();
    ui_mark_redraw();

    /* one-shot boot render benchmark: north-up vs rotated (AA / NN) per-stage
     * timings, logged to serial so the cost of each stage can be read. */
    benchFrames("north-up", 4);
    map_set_rotation(45.0f);
    while (map_ease_rotation()) {}
    map_set_aa(true);
    benchFrames("rot-AA", 4);
    map_set_aa(false);
    benchFrames("rot-NN", 4);
    map_set_aa(true);
    map_set_rotation(0.0f);
    while (map_ease_rotation()) {}
    uint32_t tp0 = esp_timer_get_time();
    map_push();
    ESP_LOGI("bench", "push: %.2f ms", (esp_timer_get_time() - tp0) / 1000.0);
    s_mapDirty = true;

    ESP_LOGI(TAG, "font selftest begin");
    ui_font_selftest();   /* DEBUG (temp): dump FontVN rendering to serial */
    ESP_LOGI(TAG, "font selftest end");

    ESP_LOGI(TAG, "map drawn, ready (loopTask stack hw=%u)",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    Serial.println("Drag on the screen to pan the map.");
}

void loop()
{
    static uint32_t hb = 0;
    if ((++hb % 200) == 0)
        /* heartbeat also reports free internal SRAM + free PSRAM so a long
         * monitor run can confirm there's no memory leak over time */
        ESP_LOGI(TAG, "loop alive (mapDirty=%d, hw=%u, sram=%u, psram=%u)",
                 (int)s_mapDirty,
                 (unsigned)uxTaskGetStackHighWaterMark(NULL),
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    input_touch_poll();

    /* BLE nav updates -> redraw */
    if (navRouteDirty() || navManeuverDirty())
    {
        bool routeNew = navRouteDirty();
        navRouteClearDirty();
        navManeuverClearDirty();
        /* a new route = navigation started: auto-rotate to heading-up so the
         * map follows the road (user can still toggle with the H button) */
        if (routeNew)
        {
            map_set_heading_up(true);
            map_force_recompose();   /* redraw the new route on a fresh world */
        }
        s_mapDirty = true;
    }
    if (navContDirty())
    {
        navContClearDirty();
        map_force_recompose();   /* redraw the continuation on a fresh world */
        s_mapDirty = true;
    }
    if (navEtaDirty() || navClockDirty() || navWeatherDirty() || navCameraDirty())
    {
        navEtaClearDirty();
        navClockClearDirty();
        navWeatherClearDirty();
        navCameraClearDirty();
        s_mapDirty = true;
    }
    if (navPosDirty())
    {
        navPosClearDirty();
        const NavPos *p = navGetPos();
        if (p && p->valid)
        {
            /* record the fix for smooth-follow interpolation */
            map_set_fix(p->lat, p->lon);
            /* heading-up mode: rotate the map so travel direction points up */
            map_apply_heading(p->hdg);
            s_mapDirty = true;
        }
    }

    if (s_mapDirty)
    {
        uint32_t nowMS = millis();
        if (nowMS - s_lastFrameMS >= FRAME_PERIOD_MS)
        {
            s_lastFrameMS = nowMS;
            s_mapDirty = false;
            drawMap();   /* buttons + HUD are composed into mapSprite, pushed once */
        }
    }

    /* Smooth follow: while a GPS fix is active, advance the view center along
     * the interpolated fix path every frame. Keep redrawing while the car is
     * moving so the map scrolls CONTINUOUSLY (no "run then stop" — the fix is
     * extrapolated past the current one instead of settling). */
    const NavPos *fp = navGetPos();
    if (fp && fp->valid && map_follow_interp())
        s_mapDirty = true;

    /* warm the tile cache ahead of the car — skipped in SIMPLE (text-only,
     * no map is loaded) so it doesn't touch the SD card / waste CPU */
    if (ui_nav_mode() != UI_MODE_SIMPLE)
        map_preload();

    /* When a frame is due, keep the loop tight (1ms) so pacing alone controls
     * the draw cadence — adding 10ms here would make the frames uneven again.
     * Idle (nothing to draw) spins at a relaxed rate. (A 100ms delay was
     * linked to the BLE route-receive crash, so stay small when active.) */
    delay(s_mapDirty ? 1 : 10);
}
