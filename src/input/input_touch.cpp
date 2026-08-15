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
static bool     s_sliderDrag = false;   /* current drag is on the brightness slider */
static const int DRAG_THRESHOLD = 6;   /* px; finger must move this far to pan */

/* hit-test buttons at tap position (x, y); returns true if consumed */
static bool handleTap(int x, int y)
{
    /* settings panel consumes taps inside/around it first */
    if (ui_settings_open())
        return ui_settings_tap(x, y);

    /* rotate button (left edge): turn the map ROTATE_STEP_DEG clockwise.
     * Manual rotation switches off heading-up mode. */
    if (x >= ROTATE_BTN_X && x < ROTATE_BTN_X + ROTATE_BTN_W &&
        y >= ROTATE_BTN_Y && y < ROTATE_BTN_Y + ROTATE_BTN_H)
    {
        map_rotate(ROTATE_STEP_DEG);
        return true;
    }
    /* heading-up toggle (left edge, under rotate): map follows GPS heading */
    if (x >= HDG_BTN_X && x < HDG_BTN_X + HDG_BTN_W &&
        y >= HDG_BTN_Y && y < HDG_BTN_Y + HDG_BTN_H)
    {
        map_set_heading_up(!map_heading_up());
        return true;
    }
    /* center button (left edge, under heading): snap the view onto the car */
    if (x >= CENTER_BTN_X && x < CENTER_BTN_X + CENTER_BTN_W &&
        y >= CENTER_BTN_Y && y < CENTER_BTN_Y + CENTER_BTN_H)
    {
        ui_recenter();
        return true;
    }

    /* top-right corner = settings (gear) */
    if (x >= GEAR_BTN_X && y < GEAR_BTN_Y + GEAR_BTN_H)
    {
        ui_toggle_settings();
        return true;
    }
    /* zoom-in "+" button (bottom-right) — disabled at ZOOM_MAX */
    if (x >= ZOOM_IN_X && x < ZOOM_IN_X + ZOOM_BTN_W &&
        y >= ZOOM_IN_Y && y < ZOOM_IN_Y + ZOOM_BTN_H)
    {
        if (ZOOM < ZOOM_MAX)
            map_zoom(+1);
        return true;
    }
    /* zoom-out "-" button (bottom-right) — disabled at ZOOM_MIN */
    if (x >= ZOOM_OUT_X && x < ZOOM_OUT_X + ZOOM_BTN_W &&
        y >= ZOOM_OUT_Y && y < ZOOM_OUT_Y + ZOOM_BTN_H)
    {
        if (ZOOM > ZOOM_MIN)
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
        s_sliderDrag = false;
        s_lastTX = s_lastTY = -1;
        return;
    }

    int x = (display.width() - 1) - tp.y;   /* screenX = 319 - rawY (mirrored) */
    int y = tp.x;                           /* screenY = rawX (0..239) */

    if (!s_touching)
    {
        s_touching = true;
        s_dragging = false;
        s_sliderDrag = false;
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
        s_sliderDrag = ui_settings_drag_started(s_downX, s_downY);
        s_lastTX = s_downX;         /* anchor the pan where the press started */
        s_lastTY = s_downY;
    }
    if (s_sliderDrag)
        ui_settings_slider_drag(x);      /* live brightness while dragging */
    else
        map_pan(x - s_lastTX, y - s_lastTY);
    s_lastTX = x;
    s_lastTY = y;
}
