/**
 * display_panel.cpp — ILI9341 display + FT6336 touch panel module.
 */
#include "display_panel.h"
#include "app_config.h"
#include <Arduino.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

static const char *TAG = "display";

LGFX display;                     /* global display + touch object */

bool display_panel_init(void)
{
    if (!display.init())
    {
        ESP_LOGE(TAG, "display.init failed");
        return false;
    }
    display.setRotation(1);          /* landscape 320x240 */
    display.setBrightness(BRIGHTNESS_DEFAULT);   /* default backlight = half */
    ESP_LOGI(TAG, "display ready 320x240");
    return true;
}

void display_panel_sleep(void)
{
    display.setBrightness(0);        /* backlight off */
    display.sleep();                 /* ILI9341 SLEEP IN (0x10) - lowest power */
}

void display_panel_wake(void)
{
    display.wakeup();                /* 0x11 */
    display.setBrightness(BRIGHTNESS_DEFAULT);
}

/* ---- software I2C master to the FT6336 ----
 * LovyanGFX owns the shared I2C bus (port 0), so we bit-bang a single write on
 * the same pins. Only used briefly before entering sleep, so no bus conflict. */
#define FT_I2C_ADDR  0x38
#define FT_SDA_PIN   GPIO_NUM_16
#define FT_SCL_PIN   GPIO_NUM_15

static void sw_i2c_delay(void) { esp_rom_delay_us(6); }   /* ~90 kHz */

static void sw_scl(bool hi)    { gpio_set_level(FT_SCL_PIN, hi ? 1 : 0); sw_i2c_delay(); }
static void sw_sda_out(bool hi){ gpio_set_direction(FT_SDA_PIN, GPIO_MODE_OUTPUT_OD);
                                 gpio_set_level(FT_SDA_PIN, hi ? 1 : 0); sw_i2c_delay(); }
static bool sw_sda_in(void)    { gpio_set_direction(FT_SDA_PIN, GPIO_MODE_INPUT);
                                 sw_i2c_delay(); return gpio_get_level(FT_SDA_PIN) != 0; }

static void sw_i2c_start(void) { sw_sda_out(true); sw_scl(true); sw_sda_out(false); sw_scl(false); }
static void sw_i2c_stop(void)  { sw_sda_out(false); sw_scl(true); sw_sda_out(true); }

static bool sw_i2c_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--)
    {
        sw_scl(false);
        sw_sda_out((b >> i) & 1);
        sw_scl(true);
    }
    sw_scl(false);
    bool ack = !sw_sda_in();       /* slave pulls SDA low to ack */
    sw_scl(true);
    sw_scl(false);
    return ack;
}

static void sw_i2c_write_reg(uint8_t reg, uint8_t val)
{
    gpio_set_direction(FT_SCL_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(FT_SDA_PIN, GPIO_MODE_OUTPUT_OD);
    sw_i2c_start();
    sw_i2c_byte(FT_I2C_ADDR << 1); /* write */
    sw_i2c_byte(reg);
    sw_i2c_byte(val);
    sw_i2c_stop();
    /* leave both lines high (released) */
    sw_scl(true);
    sw_sda_out(true);
}

void ft6336_enable_level_int(void)
{
    /* G_MODE (0x86) = 2 -> interrupt mode: INT is held LOW for the whole time a
     * touch is present. Reliable wake source vs the default short pulse. */
    sw_i2c_write_reg(0x86, 0x02);
    ESP_LOGI(TAG, "FT6336 INT -> level mode (low while touched)");
}
