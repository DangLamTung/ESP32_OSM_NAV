/**
 * ui_icon_cache.h — icon cache: Material-style white icons + the vehicle arrow.
 * Each icon is loaded ONCE (/sdcard/icon/<name>.png if present, else the
 * embedded UI_ICONS[] C-array) and cached for the app lifetime; the vehicle
 * arrow uses /sdcard/icon/nav_arrow.png with an embedded NAV_ARROW_PX fallback.
 * Pushing uses an explicit destination so the same sprite works for the
 * overlay sprite (mapSprite) and the LCD (display).
 */
#ifndef UI_ICON_CACHE_H_
#define UI_ICON_CACHE_H_

#include <LovyanGFX.hpp>

/* Push an icon sprite centered at (x,y), scaled by `scale`, to `dst`. */
void ui_icon_push(const char *name, LovyanGFX *dst, int x, int y, float scale);

/* Load the vehicle arrow PNG from SD (default /sdcard/icon/nav_arrow.png).
 * Falls back to the embedded NAV_ARROW_PX array. Returns true on success. */
bool ui_load_icon_sd(const char *path = "/sdcard/icon/nav_arrow.png");

/* Draw the vehicle arrow centered at (x,y), rotated by angleDeg, scaled.
 * Called by ui_draw_nav_marker (ui_hud) with the eased heading + map rotation. */
void ui_draw_arrow_icon(LGFX_Sprite &dst, int x, int y, float angleDeg, float scale);

/* Release all cached icon sprites (arrow + generic icons). Called by ui_deinit. */
void ui_icons_free_all(void);

#endif // UI_ICON_CACHE_H_
