/**
 * ui_controls.h — on-screen controls + overlays module.
 * Corner buttons (SCAN / mode / zoom), the BLE scan list panel, and the nav
 * overlay (route polyline, position marker, maneuver HUD).
 */
#ifndef UI_CONTROLS_H_
#define UI_CONTROLS_H_

#include <LovyanGFX.hpp>

void ui_init(void);                                      /* reset state */
void ui_draw_buttons(void);                              /* SCAN+mode+zoom on LCD */
void ui_draw_scan_overlay(void);                         /* BLE list panel */
void ui_draw_nav_overlay(LGFX_Sprite &spr, double clon, double clat, int zoom);
void ui_toggle_scan(void);                               /* show/hide BLE list */
bool ui_scan_shown(void);
bool ui_needs_redraw(void);                              /* buttons need (re)draw */
void ui_clear_redraw(void);
void ui_mark_redraw(void);

#endif // UI_CONTROLS_H_
