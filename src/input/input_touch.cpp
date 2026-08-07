/**
 * input_touch.cpp — FT6336 capacitive-touch input module.
 *
 * This module's touch controller is portrait-native (raw x≈0..239,
 * y≈0..319). Verified empirically: screenY = rawX (up/down correct) and
 * screenX must be MIRRORED: screenX = (W-1) - rawY (left/right).
 *
 * Gestures:
 *   - tap        : press + release with < DRAG_THRESHOLD movement -> buttons
 *   - drag       : move > DRAG_THRESHOLD -> pan the map
 *   - long-press : hold still for LONG_PRESS_MS -> enter sleep
 */
#include "input_touch.h"
#include "app_config.h"
#include "display_panel.h"
#include "map_view.h"
#include "ui_controls.h"
#include "power_mgr.h"
#include <Arduino.h>
#include "esp_log.h"

static const char *TAG = "touch";

static int      s_lastTX = -1, s_lastTY = -1;
static bool     s_touching = false;
static bool     s_dragging = false;
static int      s_downX = 0, s_downY = 0;
static uint32_t s_downMS = 0;
static const int DRAG_THRESHOLD = 6;   /* px; finger must move this far to pan */

/* hit-test buttons at tap position (x, y); returns true if consumed */
static bool handleTap(int x, int y)
{
    /* top-right corner = toggle the BLE scan list ("place" list) - no re-fetch */
    if (x >= SCAN_BTN_X && y <= SCAN_TAP_Y_MAX)
    {
        ui_toggle_scan();
        return true;
    }
    /* tile-source mode button (below SCAN) - cycles + relabels, no refresh */
    if (!ui_scan_shown() &&
        x >= MODE_BTN_X && x < MODE_BTN_X + MODE_BTN_W &&
        y >= MODE_BTN_Y && y < MODE_BTN_Y + MODE_BTN_H)
    {
        map_cycle_tile_mode();
        ui_mark_redraw();
        return true;
    }
    /* zoom-in "+" button (bottom-right) */
    if (!ui_scan_shown() &&
        x >= ZOOM_IN_X && x < ZOOM_IN_X + ZOOM_BTN_W &&
        y >= ZOOM_IN_Y && y < ZOOM_IN_Y + ZOOM_BTN_H)
    {
        map_zoom(+1);
        return true;
    }
    /* zoom-out "-" button (bottom-right) */
    if (!ui_scan_shown() &&
        x >= ZOOM_OUT_X && x < ZOOM_OUT_X + ZOOM_BTN_W &&
        y >= ZOOM_OUT_Y && y < ZOOM_OUT_Y + ZOOM_BTN_H)
    {
        map_zoom(-1);
        return true;
    }
    return false;
}

void input_touch_poll(void)
{
    /* ignore touches for the first moments after boot/wake so a finger still
     * on the panel from waking the device doesn't instantly re-trigger sleep */
    static uint32_t s_bootMS = 0;
    if (s_bootMS == 0) s_bootMS = millis();
    if (millis() - s_bootMS < 500) return;

    lgfx::touch_point_t tp;
    if (display.getTouchRaw(&tp, 1) != 1)
    {
        if (s_touching && !s_dragging)
        {
            /* finger lifted without ever dragging -> tap at the press point */
            handleTap(s_downX, s_downY);
        }
        s_touching = false;
        s_dragging = false;
        s_lastTX = s_lastTY = -1;
        return;
    }

    int x = (display.width() - 1) - tp.y;   /* screenX = 319 - rawY (mirrored) */
    int y = tp.x;                           /* screenY = rawX (0..239) */

    if (!s_touching)
    {
        s_touching = true;
        s_dragging = false;
        s_downX = x;
        s_downY = y;
        s_downMS = millis();
        Serial.printf("touch raw (%d,%d) -> screen (%d,%d)\n", tp.x, tp.y, x, y);
        s_lastTX = x;
        s_lastTY = y;
        return;                     /* don't pan on the very first frame */
    }

    /* long-press (finger held still) -> deep sleep. On touch wake the chip
     * reboots into setup(), so this call never returns. */
    if (!s_dragging && (millis() - s_downMS) >= (uint32_t)LONG_PRESS_MS)
    {
        ESP_LOGI(TAG, "long-press -> deep sleep");
        power_mgr_enter_sleep();
        /* not reached */
        s_touching = false;
        s_dragging = false;
        s_lastTX = s_lastTY = -1;
        return;
    }

    /* Do NOT pan until the finger really moves: jitter / small wobble below
     * the threshold is a still finger -> no map reload (no flicker). */
    int ddx = x - s_downX, ddy = y - s_downY;
    if (!s_dragging)
    {
        if (abs(ddx) < DRAG_THRESHOLD && abs(ddy) < DRAG_THRESHOLD)
        {
            s_lastTX = x;
            s_lastTY = y;
            return;                 /* still within tap tolerance: no pan */
        }
        s_dragging = true;          /* finger moved enough -> real drag */
        s_lastTX = s_downX;         /* anchor the pan where the press started */
        s_lastTY = s_downY;
    }
    map_pan(x - s_lastTX, y - s_lastTY);
    s_lastTX = x;
    s_lastTY = y;
}
