/**
 * ui_settings.cpp — settings panel state + touch handling (ui_settings.h).
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#include "ui_controls.h"     /* ui_mark_redraw() (also pulls in ui_settings.h) */
#include "ui_settings.h"
#include "app_config.h"      /* button geometry */
#include "display_panel.h"   /* display.setBrightness() */
#include "wifi_net.h"        /* wifi_net_connect() */
#include "ble_nav.h"         /* navSetGpsBroadcast() */
#include "map_view.h"        /* map_set_aa() */
#include <Arduino.h>

/* ---- settings (gear) panel: WiFi connect + brightness slider ---- */
static bool s_settingsOpen = false;
static int  s_brightness   = BRIGHTNESS_DEFAULT;

/* Bottom-bar style, toggled from the settings panel. The next + next-next
 * guidance is ALWAYS drawn over the full map (no separate 2-step mode). */
static int s_navMode = UI_MODE_FULL;
const char *ui_nav_mode_label(void)
{
    return (s_navMode == UI_MODE_SIMPLE) ? "SIMPLE" : "FULL";
}
int  ui_nav_mode(void) { return s_navMode; }
void ui_cycle_nav_mode(void)
{
    s_navMode = (s_navMode == UI_MODE_SIMPLE) ? UI_MODE_FULL : UI_MODE_SIMPLE;
    Serial.printf("[ui] nav mode %d (%s)\n", s_navMode, ui_nav_mode_label());
    ui_mark_redraw();
}

void ui_toggle_settings(void)
{
    s_settingsOpen = !s_settingsOpen;
    Serial.printf("[ui] settings %s\n", s_settingsOpen ? "open" : "closed");
    ui_mark_redraw();
}

bool ui_settings_open(void) { return s_settingsOpen; }

static void ui_set_brightness(int b)
{
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    s_brightness = b;
    display.setBrightness((uint8_t)b);
    Serial.printf("[ui] brightness %d\n", b);
}

int ui_brightness(void) { return s_brightness; }

bool ui_settings_tap(int x, int y)
{
    if (!s_settingsOpen) return false;
    if (y < SETTINGS_PANEL_Y)            /* tapped above the panel -> close it */
    {
        s_settingsOpen = false;
        ui_mark_redraw();
        return true;
    }
    if (x >= WIFI_BTN_X && x < WIFI_BTN_X + WIFI_BTN_W &&
        y >= WIFI_BTN_Y && y < WIFI_BTN_Y + WIFI_BTN_H)
    {
        wifi_net_connect();
        ui_mark_redraw();
        return true;
    }
    /* GPS broadcast toggle (settings panel) */
    if (x >= GPS_BTN_X && x < GPS_BTN_X + GPS_BTN_W &&
        y >= GPS_BTN_Y && y < GPS_BTN_Y + GPS_BTN_H)
    {
        navSetGpsBroadcast(!navGpsBroadcast());
        ui_mark_redraw();
        return true;
    }
    /* nav HUD mode cycle (settings panel) */
    if (x >= MODE_BTN_X && x < MODE_BTN_X + MODE_BTN_W &&
        y >= MODE_BTN_Y && y < MODE_BTN_Y + MODE_BTN_H)
    {
        ui_cycle_nav_mode();
        return true;
    }
    /* rotation quality: crisp (AA) vs fast (nearest-neighbour) */
    if (x >= AA_BTN_X && x < AA_BTN_X + AA_BTN_W &&
        y >= AA_BTN_Y && y < AA_BTN_Y + AA_BTN_H)
    {
        map_set_aa(!map_aa_enabled());
        ui_mark_redraw();
        return true;
    }
    if (y >= SLIDER_Y && y < SLIDER_Y + SLIDER_H)
    {
        ui_set_brightness((x - SLIDER_X) * 255 / SLIDER_W);
        ui_mark_redraw();
        return true;
    }
    return true;                        /* inside the panel but not on a control */
}

bool ui_settings_drag_started(int downX, int downY)
{
    return s_settingsOpen && downY >= SLIDER_Y && downY < SLIDER_Y + SLIDER_H;
}

void ui_settings_slider_drag(int x)
{
    ui_set_brightness((x - SLIDER_X) * 255 / SLIDER_W);
    ui_mark_redraw();
}
