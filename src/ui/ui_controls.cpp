/**
 * ui_controls.cpp — UI shared state + lifecycle (redraw flag, init/deinit).
 *
 * The heavier drawing lives in the sibling modules split out on 2026-08-15
 * (behavior-preserving refactor):
 *   ui_icon_cache / ui_settings / ui_hud / ui_buttons / ui_splash
 * Callers keep including ui_controls.h (the umbrella header).
 */
#include "ui_controls.h"
#include "ui_icon_cache.h"   /* ui_icons_free_all() */
#include "map_view.h"        /* s_mapDirty */
#include "esp_log.h"

static bool s_uiDirty  = true;

bool ui_needs_redraw(void) { return s_uiDirty; }
void ui_clear_redraw(void) { s_uiDirty = false; }
/* Buttons/settings are composed into the map frame (drawMap -> ui_draw_buttons
 * on mapSprite), so any UI-state change must re-render the map frame. */
void ui_mark_redraw(void)  { s_uiDirty = true; s_mapDirty = true; }

void ui_init(void)
{
    s_uiDirty  = true;
}

void ui_deinit(void)
{
    ui_icons_free_all();
}
