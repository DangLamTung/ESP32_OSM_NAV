/**
 * ft6336.c — FT6336 touch over ESP-IDF I2C master (new driver API).
 *
 * Registers follow the FocalTech FT6x06 family (same as the vendor
 * FT6336-arduino lib): TD_STATUS 0x02, touch data at 0x03 (6 bytes/point).
 */
#include "ft6336.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define FT6336_ADDR     0x38
#define FT6336_TD_STATUS 0x02
#define FT6336_TOUCH_1   0x03

#define PIN_SDA 16
#define PIN_SCL 15
#define PIN_RST 18

static const char *TAG = "ft6336";
static i2c_master_dev_handle_t s_dev = NULL;

esp_err_t ft6336_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = FT6336_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(err));
        return err;
    }

    /* hardware reset (matches vendor FT6336 begin()) */
    gpio_reset_pin(PIN_RST);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 1);
    esp_rom_delay_us(20 * 1000);
    gpio_set_level(PIN_RST, 0);
    esp_rom_delay_us(20 * 1000);
    gpio_set_level(PIN_RST, 1);
    esp_rom_delay_us(500 * 1000);

    ESP_LOGI(TAG, "FT6336 ready on I2C0 (SDA=%d SCL=%d)", PIN_SDA, PIN_SCL);
    return ESP_OK;
}

static uint8_t ft_read_reg(uint8_t reg)
{
    uint8_t v = 0;
    if (s_dev) {
        i2c_master_transmit_receive(s_dev, &reg, 1, &v, 1, 100 / portTICK_PERIOD_MS);
    }
    return v;
}

void ft6336_read(int *raw_x, int *raw_y, bool *touched)
{
    *touched = false;
    if (!s_dev) return;

    uint8_t status = ft_read_reg(FT6336_TD_STATUS);
    uint8_t n = status & 0x0F;
    if (n == 0 || n > 2) return;

    uint8_t data[4];
    uint8_t reg = FT6336_TOUCH_1;
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, data, 4,
                                                100 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return;

    *raw_x = ((data[0] & 0x0F) << 8) | data[1];
    *raw_y = ((data[2] & 0x0F) << 8) | data[3];
    *touched = true;
}

void ft6336_to_screen(int raw_x, int raw_y, int *sx, int *sy)
{
    /* Verified on this module: portrait-native raw -> landscape screen. */
    *sx = 319 - raw_y;
    *sy = raw_x;
}
