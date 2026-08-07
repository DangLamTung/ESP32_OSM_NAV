/**
 * power_mgr.h — deep-sleep power management, wake on touch.
 * The FT6336 interrupt pin (TOUCH_INT_GPIO, RTC-capable) wakes the chip from
 * deep sleep whenever the panel is touched. Wake = full reboot into setup().
 */
#ifndef POWER_MGR_H_
#define POWER_MGR_H_

void power_mgr_init(void);        /* one-time init/log (wake source is armed
                                     per-sleep, just before esp_deep_sleep) */
void power_mgr_enter_sleep(void); /* backlight+panel off, ext0 wake, deep sleep
                                     (never returns; reboots on touch wake) */

#endif // POWER_MGR_H_
