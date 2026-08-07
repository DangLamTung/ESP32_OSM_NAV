/**
 * display_panel.h — ILI9341 display + FT6336 touch panel module.
 * Owns the global `LGFX display` object and the low-power helpers.
 */
#ifndef DISPLAY_PANEL_H_
#define DISPLAY_PANEL_H_

#include <stdbool.h>
#include "LGFX_ILI9341.h"

extern LGFX display;              /* global display + touch object */

bool display_panel_init(void);    /* init panel, landscape rotation, backlight */
void display_panel_sleep(void);   /* backlight off + panel SLEEP IN (low power) */
void display_panel_wake(void);    /* panel wake + backlight on */
void ft6336_enable_level_int(void); /* FT6336 INT -> level mode (reliable wake) */

#endif // DISPLAY_PANEL_H_
