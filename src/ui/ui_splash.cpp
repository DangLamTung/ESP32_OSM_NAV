/**
 * ui_splash.cpp — boot splash implementation (ui_splash.h).
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 *
 * Priority: 1) /sdcard/splash.ani (a GIF preprocessed by scripts/gif_to_splash
 *              -> played by a small task while the rest of setup() runs),
 *           2) a static SD image (splash.png / cat.png / splash.jpg),
 *           3) a plain deep-blue fill.
 * The .ani task is stopped via ui_splash_stop() before the first map draw.
 */
#include "ui_splash.h"
#include "display_panel.h"   /* display */
#include "app_config.h"      /* SCREEN_W/H */
#include <Arduino.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static TaskHandle_t  s_splashTask = NULL;
static volatile bool s_splashStop = false;
static uint32_t      s_splashStartMS = 0;
#define MIN_SPLASH_MS 2500   /* guaranteed splash visibility before the map */

static void splashAniTask(void *arg)
{
  const long kDataOff = 6 + 2 + 2 + 2;   /* magic + w + h + nframes = 12 */
  FILE *f = fopen("/sdcard/splash.ani", "rb");
  if (f)
  {
    char magic[6];
    uint16_t w = 0, h = 0, nframes = 0;
    if (fread(magic, 1, 6, f) == 6 && memcmp(magic, "ANIM01", 6) == 0 &&
        fread(&w, 2, 1, f) == 1 && fread(&h, 2, 1, f) == 1 &&
        fread(&nframes, 2, 1, f) == 1 &&
        w > 0 && h > 0 && nframes > 0 && w <= SCREEN_W && h <= SCREEN_H)
    {
      size_t fbsz = (size_t)w * h * 2;
      uint16_t *fb = (uint16_t *)heap_caps_malloc(fbsz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!fb) fb = (uint16_t *)malloc(fbsz);
      if (fb)
      {
        uint32_t fno = 0;
        while (!s_splashStop)
        {
          uint16_t delay = 100;
          if (fread(&delay, 2, 1, f) != 1 ||
              fread(fb, 2, (size_t)w * h, f) != (size_t)w * h)
          {
            ESP_LOGW("ui", "splash: frame read fail, looping back");
            fseek(f, kDataOff, SEEK_SET);
            fread(&delay, 2, 1, f);
            fread(fb, 2, (size_t)w * h, f);
          }
          ESP_LOGI("ui", "splash: frame %u", (unsigned)fno);
          display.pushImage(0, 0, w, h, fb);
          /* sleep in small chunks so ui_splash_stop() returns quickly */
          uint32_t remain = delay;
          while (remain > 0 && !s_splashStop)
          {
            uint32_t chunk = remain > 20 ? 20 : remain;
            vTaskDelay(pdMS_TO_TICKS(chunk));
            remain -= chunk;
          }
          if (++fno >= nframes) { fno = 0; fseek(f, kDataOff, SEEK_SET); }
        }
        free(fb);
      }
    }
    fclose(f);
  }
  s_splashTask = NULL;
  vTaskDelete(NULL);
}

void ui_splash_stop(void)
{
  /* Keep animating until the splash has been visible for MIN_SPLASH_MS, so
   * the animation is actually seen even though boot is fast. */
  uint32_t elapsed = millis() - s_splashStartMS;
  if (elapsed < MIN_SPLASH_MS)
  {
    uint32_t remain = MIN_SPLASH_MS - elapsed;
    while (remain > 0 && s_splashTask)
    {
      uint32_t chunk = remain > 20 ? 20 : remain;
      vTaskDelay(pdMS_TO_TICKS(chunk));
      remain -= chunk;
    }
  }
  s_splashStop = true;
  while (s_splashTask) vTaskDelay(pdMS_TO_TICKS(5));
}

void ui_show_splash(void)
{
  /* 1) animated splash.ani (preprocessed GIF) */
  FILE *probe = fopen("/sdcard/splash.ani", "rb");
  if (probe)
  {
    fclose(probe);
    s_splashStop = false;
    s_splashStartMS = millis();
    xTaskCreatePinnedToCore(splashAniTask, "splash", 4096, NULL, 5, &s_splashTask, 1);
    ESP_LOGI("ui", "splash: animating /sdcard/splash.ani");
    return;
  }

  /* 2) static splash image from SD */
  static const char *kSplash[] = {
    "/sdcard/splash.png",
    "/sdcard/cat.png",
    "/sdcard/splash.jpg",
    "/sdcard/cat.jpg",
  };
  for (int i = 0; i < (int)(sizeof(kSplash) / sizeof(kSplash[0])); ++i)
  {
    const char *path = kSplash[i];
    FILE *f = fopen(path, "rb");
    if (!f) continue;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    bool ok = false;
    if (len > 0 && len <= 1024 * 1024)
    {
      uint8_t *buf = (uint8_t *)malloc((size_t)len);
      if (buf)
      {
        if (fread(buf, 1, (size_t)len, f) == (size_t)len)
        {
          if (strstr(path, ".jpg") || strstr(path, ".jpeg"))
            ok = display.drawJpg(buf, (uint32_t)len, 0, 0);
          else
            ok = display.drawPng(buf, (uint32_t)len, 0, 0);
        }
        free(buf);
      }
    }
    fclose(f);
    if (ok)
    {
      ESP_LOGI("ui", "splash from %s", path);
      return;
    }
  }

  /* 3) plain deep-blue fill until the map covers it. */
  ESP_LOGI("ui", "splash: no SD image, plain fill");
  display.fillScreen(display.color565(24, 34, 60));
}
