/**
 * sd_card.h — SD/TF card (SDMMC 4-bit) + FAT/exFAT mount for the
 * 2.8" IPS ESP32-S3 + ILI9341 board (ES3C28P/ES3N28P).
 *
 * TF slot (vendor Example_05_show_SD_jpg_picture):
 *   SD_CLK=38  SD_CMD=40  SD_D0=39  SD_D1=41  SD_D2=48  SD_D3=47
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the SDMMC host, mount the card at /sdcard (FAT/exFAT).
 * @return true on success (card mounted), false otherwise.
 */
bool sd_card_init(void);

/**
 * @brief Unmount the card and release the host.
 */
void sd_card_deinit(void);

#ifdef __cplusplus
}
#endif
