/**
 * ui_settings.h — settings panel state + touch handling.
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 * Holds the NAV mode (FULL/SIMPLE), the panel-open flag, and brightness.
 */
#ifndef UI_SETTINGS_H_
#define UI_SETTINGS_H_

/* Bottom-bar style, toggled from the settings panel. The next + next-next
 * guidance is ALWAYS drawn over the full map (no separate 2-step mode). */
enum { UI_MODE_FULL = 0, UI_MODE_SIMPLE = 1 };

int  ui_nav_mode(void);            /* current style: FULL or SIMPLE */
const char *ui_nav_mode_label(void); /* "FULL" / "SIMPLE" (tiny mode tag) */
void ui_cycle_nav_mode(void);      /* toggle style (settings panel NAV button) */
void ui_toggle_settings(void);     /* gear: open/close the panel */
bool ui_settings_open(void);       /* panel visible? */
bool ui_settings_tap(int x, int y);/* tap handled by the panel? */
bool ui_settings_drag_started(int downX, int downY); /* drag on brightness slider? */
void ui_settings_slider_drag(int x);                 /* live brightness from drag */
int  ui_brightness(void);          /* current brightness (settings panel thumb) */

/* offline (real-road) routing toggle, persisted to /sdcard/config.txt. When
 * OFF the big graph is not loaded at boot (faster boot, less PSRAM). */
void ui_settings_init(void);       /* read /sdcard/config.txt once at boot */
bool ui_offline_route_enabled(void);
void ui_toggle_offline_route(void);/* flip + persist + load/unload the graph */

#endif // UI_SETTINGS_H_
