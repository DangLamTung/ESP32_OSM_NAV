/**
 * sd_upload.c — USB "reader mode" implementation (see sd_upload.h).
 *
 * Uses the USB-Serial/JTAG driver directly (usb_serial_jtag_read_bytes /
 * usb_serial_jtag_write_bytes) so it reads COM9 regardless of which console
 * owns stdin. The IDF console may mirror logs to the same port; the PC side
 * skips log lines when looking for protocol acknowledgements.
 */
#include "sd_upload.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"

#include "sd_card.h"

static const char *TAG = "sd_up";

#define UPLOAD_MAGIC "OSMUP1"
#define MAX_FILE_SIZE (4 * 1024 * 1024) /* 4 MB per file is plenty for a tile */

/* ---- line reader over the USB-Serial/JTAG port ---- */
static char s_line[128];

/* Read one byte; returns -1 on timeout. */
static int read_byte(uint32_t timeout_ms)
{
    uint8_t b;
    int n = usb_serial_jtag_read_bytes(&b, 1, pdMS_TO_TICKS(timeout_ms));
    return (n == 1) ? (int)b : -1;
}

/* Collect bytes until '\n'. Returns line length, 0 if empty, -1 on timeout. */
static int read_line(uint32_t timeout_ms)
{
    size_t len = 0;
    const uint32_t start = (uint32_t)(esp_timer_get_time() / 1000);
    while (len < sizeof(s_line) - 1) {
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now - start) > timeout_ms)
            return -1;
        int b = read_byte(20);
        if (b < 0)
            continue;
        if (b == '\n') {
            s_line[len] = '\0';
            return (int)len;
        }
        if (b != '\r')
            s_line[len++] = (char)b;
    }
    s_line[len] = '\0';
    return (int)len;
}

static void send_str(const char *s)
{
    usb_serial_jtag_write_bytes(s, strlen(s), pdMS_TO_TICKS(200));
}

/* mkdir -p for a relative dir like "16/52130" under /sdcard. */
static void mkdir_p(const char *rel_dir)
{
    char path[128];
    snprintf(path, sizeof(path), "/sdcard/%s", rel_dir);
    for (char *p = path + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            mkdir(path, 0755);
            *p = '/';
        }
    }
    mkdir(path, 0755);
}

/* Drain anything already waiting on the port (boot logs are TX-only, but be
 * safe so a stray keystroke can't look like the magic). */
static void drain_rx(void)
{
    uint8_t tmp[64];
    while (usb_serial_jtag_read_bytes(tmp, sizeof(tmp), 0) > 0) {
    }
}

bool sd_upload_listen(int timeout_ms)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 1024;
    cfg.tx_buffer_size = 1024;
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "usb_serial_jtag install: %s", esp_err_to_name(err));
    /* ESP_ERR_INVALID_STATE = already installed (e.g. by the console) — fine. */

    drain_rx();

    ESP_LOGI(TAG, "USB reader mode: waiting %d ms for upload magic...", timeout_ms);
    const int rc = read_line((uint32_t)timeout_ms);
    if (rc <= 0 || strcmp(s_line, UPLOAD_MAGIC) != 0) {
        ESP_LOGI(TAG, "no upload requested, booting normally");
        return false;
    }

    ESP_LOGI(TAG, "upload requested, mounting SD...");
    if (!sd_card_init()) {
        send_str("ERR no_sd\n");
        return true;
    }
    send_str("READY\n");

    while (1) {
        const int lr = read_line(60000);
        if (lr <= 0) {
            send_str("ERR timeout\n");
            continue;
        }
        if (strcmp(s_line, "DONE") == 0) {
            send_str("OK\n");
            vTaskDelay(pdMS_TO_TICKS(150));
            ESP_LOGI(TAG, "upload complete");
            return true;
        }
        if (strcmp(s_line, "ABRT") == 0) {
            send_str("OK\n");
            return true;
        }
        if (strncmp(s_line, "FILE ", 5) != 0) {
            send_str("ERR cmd\n");
            continue;
        }

        unsigned int size = 0;
        char rel[96] = {0};
        if (sscanf(s_line + 5, "%u %95s", &size, rel) != 2 || size == 0 ||
            size > MAX_FILE_SIZE) {
            send_str("ERR badfile\n");
            continue;
        }

        char *slash = strrchr(rel, '/');
        if (slash) {
            *slash = '\0';
            mkdir_p(rel);
            *slash = '/';
        }

        char full[128];
        snprintf(full, sizeof(full), "/sdcard/%s", rel);
        FILE *f = fopen(full, "wb");
        if (!f) {
            send_str("ERR open\n");
            continue;
        }

        uint8_t buf[512];
        uint32_t left = size;
        bool ok = true;
        while (left > 0) {
            const uint32_t want = (left > sizeof(buf)) ? (uint32_t)sizeof(buf) : left;
            const int got = usb_serial_jtag_read_bytes(buf, want, pdMS_TO_TICKS(15000));
            if (got <= 0) {
                ok = false;
                break;
            }
            if (fwrite(buf, 1, (size_t)got, f) != (size_t)got) {
                ok = false;
                break;
            }
            left -= (uint32_t)got;
        }
        fclose(f);

        if (ok && left == 0) {
            ESP_LOGI(TAG, "wrote %s (%u B)", rel, size);
            send_str("OK\n");
        } else {
            unlink(full);
            send_str("ERR write\n");
        }
    }
}
