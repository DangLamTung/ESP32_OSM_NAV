/**
 * ui_splash.h — boot splash animation/static image.
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#ifndef UI_SPLASH_H_
#define UI_SPLASH_H_

void ui_show_splash(void);   /* boot splash: splash.ani, else SD image, else fill */
void ui_splash_stop(void);   /* stop the animated splash task before map draw */

#endif // UI_SPLASH_H_
