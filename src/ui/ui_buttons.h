/**
 * ui_buttons.h — corner buttons + settings panel composition.
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#ifndef UI_BUTTONS_H_
#define UI_BUTTONS_H_

#include <LovyanGFX.hpp>

/* Compose rotate/heading/center/gear/zoom/settings onto a sprite. Called from
 * drawMap() with mapSprite so buttons live INSIDE the frame -> no LCD flicker. */
void ui_draw_buttons(LovyanGFX *dst);

/* Snap the view back onto the car (center button). */
void ui_recenter(void);

#endif // UI_BUTTONS_H_
