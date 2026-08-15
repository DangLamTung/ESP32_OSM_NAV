/**
 * ui_hud.h — navigation HUD drawing: maneuver banner, weather, bottom bar,
 * route polyline, car marker, and the SIMPLE text-only screen.
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#ifndef UI_HUD_H_
#define UI_HUD_H_

#include <LovyanGFX.hpp>

/* Draw the route polylines onto the (north-up) world sprite, before
 * map_render() rotates it. `spr` is centered on (refLon,refLat) — the point
 * the world was last composed at (see map_ref_lon/lat). Called only when the
 * world is recomposed so the route can't ghost. */
void ui_draw_nav_route(LGFX_Sprite &spr, double refLon, double refLat, int zoom);

/* Draw the car marker on the visible sprite (screen-fixed) at its center,
 * eased heading + map rotation — drawn AFTER map_render() so it never ghosts. */
void ui_draw_nav_marker(LGFX_Sprite &spr);

/* Draw the screen-fixed HUD onto the visible sprite AFTER map_render():
 * FULL = guidance strip + weather + bottom bar; SIMPLE = text-only screen. */
void ui_draw_nav_hud(LGFX_Sprite &spr);

/* DEBUG (temp): dump FontVN glyph rendering to serial (ESP_LOGI 'font'). */
void ui_font_selftest(void);

#endif // UI_HUD_H_
