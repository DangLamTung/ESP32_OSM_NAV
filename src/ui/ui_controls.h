/**
 * ui_controls.h — on-screen controls + overlays module.
 * Corner buttons (SCAN / mode / zoom), the BLE scan list panel, and the nav
 * overlay (route polyline, position marker, maneuver HUD).
 */
#ifndef UI_CONTROLS_H_
#define UI_CONTROLS_H_

#include <LovyanGFX.hpp>

void ui_init(void);                                      /* reset state */
void ui_deinit(void);                                    /* release resources */
void ui_font_selftest(void);                             /* DEBUG (temp): dump FontVN rendering to serial */
void ui_show_splash(void);                               /* boot splash: splash.ani (GIF), else SD image, else fill */
void ui_splash_stop(void);                               /* stop the animated splash task before map draw */
bool ui_load_icon_sd(const char *path = "/sdcard/icon/nav_arrow.png"); /* load vehicle PNG from SD */
/* Compose rotate/heading/center/gear/zoom/settings onto a sprite. Called from
 * drawMap() with mapSprite so buttons live INSIDE the frame -> no LCD flicker. */
void ui_draw_buttons(LovyanGFX *dst);
/* Snap the view back onto the car (center button). */
void ui_recenter(void);
/* Draw the route polylines onto the (north-up) world sprite, before
 * map_render() rotates it. `spr` is centered on (refLon,refLat) — the point
 * the world was last composed at (see map_ref_lon/lat). Called only when the
 * world is recomposed so the route can't ghost. */
void ui_draw_nav_route(LGFX_Sprite &spr, double refLon, double refLat, int zoom);
/* Draw the car marker on the visible sprite (screen-fixed) at its center,
 * eased heading + map rotation — drawn AFTER map_render() so it never ghosts. */
void ui_draw_nav_marker(LGFX_Sprite &spr);
/* Draw the screen-fixed HUD (maneuver banner, weather, bottom bar) onto the
 * visible sprite AFTER map_render(), so it stays upright while the map turns. */
void ui_draw_nav_hud(LGFX_Sprite &spr);
/* Bottom-bar style (cycled from the settings panel): FULL (icons+clock+ETA) /
 * SIMPLE (one row of speed/time/ETA). The next + next-next guidance is always
 * shown over the full map — there is NO separate 2-step mode. */
enum { UI_MODE_FULL = 0, UI_MODE_SIMPLE = 1 };
int  ui_nav_mode(void);       /* current bottom-bar style */
void ui_cycle_nav_mode(void); /* toggle the bottom-bar style (settings button) */
void ui_toggle_settings(void);                           /* gear: open/close panel */
bool ui_settings_open(void);                             /* panel visible? */
bool ui_settings_tap(int x, int y);                      /* tap handled by panel? */
bool ui_settings_drag_started(int downX, int downY);     /* drag on brightness slider? */
void ui_settings_slider_drag(int x);                     /* live brightness from drag */
bool ui_needs_redraw(void);                              /* buttons need (re)draw */
void ui_clear_redraw(void);
void ui_mark_redraw(void);

#endif // UI_CONTROLS_H_
