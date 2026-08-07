/**
 * power_mgr.cpp — deep-sleep power management, wake on touch.
 *
 * Deep sleep: RAM is lost and the chip reboots into setup() on wake, so the
 * whole app re-initializes (display, SD, WiFi, BLE) — ~1-2 s to a live map.
 * Wake source: FT6336 INT (active-low, open-drain) on TOUCH_INT_GPIO (17),
 * which is RTC-capable (RTC_GPIO17) -> esp_sleep_enable_ext0_wakeup.
 */
#include "power_mgr.h"
#include "app_config.h"
#include "display_panel.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

static const char *TAG = "power";

void power_mgr_init(void)
{
    /* Wake source is armed per-sleep (just before esp_deep_sleep_start), so
     * nothing to do here except log. Kept so the app calls an explicit API. */
    ESP_LOGI(TAG, "power manager ready (deep sleep, wake on touch GPIO%d)",
             TOUCH_INT_GPIO);
}

void power_mgr_enter_sleep(void)
{
    ESP_LOGI(TAG, "deep sleep: backlight off, wake on touch (GPIO%d low)",
             TOUCH_INT_GPIO);

    /* display off first: backlight 0 + ILI9341 SLEEP IN */
    display_panel_sleep();

    /* FT6336 INT must HOLD LOW while touched (level mode) so the RTC sees it. */
    ft6336_enable_level_int();

    /* Wake when the FT6336 INT (active-low) goes low. */
    gpio_pullup_en((gpio_num_t)TOUCH_INT_GPIO);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT_GPIO, 0);

    /* Never returns. On touch, the chip reboots and runs setup() again. */
    esp_deep_sleep_start();
}
