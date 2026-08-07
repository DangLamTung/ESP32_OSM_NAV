/**
 * sd_card.c — SD/TF card (SDMMC 4-bit) + FAT/exFAT mount.
 *
 * TF slot pins (vendor Example_05_show_SD_jpg_picture):
 *   SD_CLK=38  SD_CMD=40  SD_D0=39  SD_D1=41  SD_D2=48  SD_D3=47
 *   -> native SDMMC host, slot 1, 4-bit bus, 3.3V.
 *
 * Mounts the card at /sdcard using ESP-IDF's esp_vfs_fat_sdmmc_mount().
 * exFAT support requires FF_FS_EXFAT=1 in components/fatfs/src/ffconf.h.
 */
#include "sd_card.h"

#include <stdio.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static const char *TAG = "sd_card";

/* ES3C28P/ES3N28P TF card slot (SDMMC slot 1, 4-bit) */
#define SD_CLK GPIO_NUM_38
#define SD_CMD GPIO_NUM_40
#define SD_D0  GPIO_NUM_39
#define SD_D1  GPIO_NUM_41
#define SD_D2  GPIO_NUM_48
#define SD_D3  GPIO_NUM_47

static sdmmc_card_t *s_card = NULL;

bool sd_card_init(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; /* 40 MHz (vendor demo speed) */

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_CLK;
    slot.cmd   = SD_CMD;
    slot.d0    = SD_D0;
    slot.d1    = SD_D1;
    slot.d2    = SD_D2;
    slot.d3    = SD_D3;
    slot.width = 4; /* 4-bit bus */
    slot.flags = 0; /* no CD/WP pins on this board */

    esp_vfs_fat_mount_config_t mount = {
        .format_if_mount_failed = false, /* never format the card */
        .max_files              = 5,
        .allocation_unit_size   = 0,     /* default cluster size */
        .disk_status_check_enable = false,
        .use_one_fat            = false,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mount /sdcard failed: %s", esp_err_to_name(ret));
        s_card = NULL;
        return false;
    }

    /* Card info (name, type, size) is printed by sdmmc_card_print_info too. */
    sdmmc_card_print_info(stdout, s_card);
    uint64_t bytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
    ESP_LOGI(TAG, "SD mounted at /sdcard: %s, %" PRIu64 " MB",
             s_card->is_mmc ? "MMC" : "SD", bytes / (1024 * 1024));
    return true;
}

void sd_card_deinit(void)
{
    if (s_card) {
        esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        s_card = NULL;
    }
}
