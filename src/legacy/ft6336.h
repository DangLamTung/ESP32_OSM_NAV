/**
 * ft6336.h — FT6336 capacitive touch driver (ESP-IDF, I2C master).
 *
 * Pins: SDA=16, SCL=15, INT=17, RST=18. Raw touch is portrait-native
 * (rawX ~0..239, rawY ~0..319). Use ft6336_to_screen() to map to the
 * landscape 320x240 display (verified empirically on this module).
 */
#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ft6336_init(void);
void ft6336_read(int *raw_x, int *raw_y, bool *touched);

/** Map raw touch to landscape display coords:
 *  screenX = 319 - rawY, screenY = rawX (mirrored swap). */
void ft6336_to_screen(int raw_x, int raw_y, int *sx, int *sy);

#ifdef __cplusplus
}
#endif
