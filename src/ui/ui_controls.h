/**
 * ui_controls.h — UI umbrella header.
 * Re-exports the split UI modules (icon cache, settings, HUD, buttons, splash)
 * plus the shared UI lifecycle/redraw flag, so existing callers that include
 * just this header keep working unchanged.
 *
 * Modules (split out of ui_controls.cpp, 2026-08-15 refactor):
 *   ui_icon_cache.h  - icon cache (Material icons + vehicle arrow)
 *   ui_settings.h    - settings panel state + touch handling (nav mode, brightness)
 *   ui_hud.h         - navigation HUD drawing (banner/weather/bottom bar/route/marker)
 *   ui_buttons.h     - corner buttons + settings panel composition
 *   ui_splash.h      - boot splash animation/static image
 */
#ifndef UI_CONTROLS_H_
#define UI_CONTROLS_H_

#include <LovyanGFX.hpp>
#include "ui_icon_cache.h"
#include "ui_settings.h"
#include "ui_hud.h"
#include "ui_buttons.h"
#include "ui_splash.h"

/* Shared UI lifecycle + redraw flag. */
void ui_init(void);               /* reset state */
void ui_deinit(void);             /* release resources (icons etc.) */
bool ui_needs_redraw(void);       /* buttons need (re)draw */
void ui_clear_redraw(void);
void ui_mark_redraw(void);

#endif // UI_CONTROLS_H_
