/**
 * ui_icon_cache.cpp — icon cache implementation (ui_icon_cache.h).
 * Split out of ui_controls.cpp (2026-08-15 refactor, behavior-preserving).
 */
#include "ui_icon_cache.h"
#include "ui_controls.h"     /* ui_mark_redraw() */
#include "map_view.h"        /* mapSprite (sprite parent) */
#include "nav_arrow.h"       /* NAV_ARROW_* embedded fallback */
#include "ui_icons.h"        /* UI_ICONS[] embedded white Material icons */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static LGFX_Sprite *g_arrowIcon = nullptr;   /* vehicle marker (nav_arrow.h / SD) */

/* generic icon cache (ui_icons.h): SD PNG first, embedded C-array fallback */
#define UI_ICON_CACHE_MAX 10
static struct { const char *name; LGFX_Sprite *spr; } s_iconCache[UI_ICON_CACHE_MAX];
static int s_iconCacheCount = 0;

static bool loadPngIconFromBuffer(LGFX_Sprite &dst, const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return false;

    if (g_arrowIcon) {
        g_arrowIcon->deleteSprite();
    } else {
        g_arrowIcon = new LGFX_Sprite(&dst);
    }
    g_arrowIcon->setColorDepth(16);
    g_arrowIcon->createSprite(NAV_ARROW_W, NAV_ARROW_H);
    g_arrowIcon->fillSprite(NAV_ARROW_TRANSP);

    bool ok = g_arrowIcon->drawPng(data, len, 0, 0);
    if (!ok) {
        ESP_LOGE("ui", "drawPng failed to decode PNG stream");
        g_arrowIcon->deleteSprite();
        delete g_arrowIcon;
        g_arrowIcon = nullptr;
        return false;
    }
    return true;
}

bool ui_load_icon_sd(const char *path)
{
    const char *filePath = (path && path[0]) ? path : "/sdcard/icon/nav_arrow.png";
    FILE *f = fopen(filePath, "rb");
    if (!f) {
        ESP_LOGW("ui", "Icon file not found on SD card: %s", filePath);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (len <= 0 || len > 256 * 1024) {
        ESP_LOGE("ui", "Invalid PNG file size on SD card (%ld bytes)", len);
        fclose(f);
        return false;
    }

    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) {
        ESP_LOGE("ui", "Failed to allocate memory for SD PNG buffer (%ld bytes)", len);
        fclose(f);
        return false;
    }

    size_t nread = fread(buf, 1, len, f);
    fclose(f);

    if (nread != (size_t)len) {
        ESP_LOGE("ui", "Read mismatch for SD PNG file: read %zu / %ld", nread, len);
        free(buf);
        return false;
    }

    bool ok = loadPngIconFromBuffer(mapSprite, buf, (uint32_t)len);
    free(buf);

    if (ok) {
        ESP_LOGI("ui", "Successfully loaded vehicle PNG icon from SD: %s", filePath);
        ui_mark_redraw();
    }
    return ok;
}

static void ensureArrowIcon(LGFX_Sprite &dst) {
    if (g_arrowIcon) return;

    /* Attempt loading custom PNG from SD card first */
    if (ui_load_icon_sd("/sdcard/icon/nav_arrow.png")) {
        return;
    }

    /* Fallback: pre-rendered embedded C-array icon */
    ESP_LOGI("ui", "Using embedded C-array vehicle icon fallback");
    g_arrowIcon = new LGFX_Sprite(&dst);
    g_arrowIcon->setColorDepth(16);
    g_arrowIcon->createSprite(NAV_ARROW_W, NAV_ARROW_H);
    for (int y = 0; y < NAV_ARROW_H; y++)
        for (int x = 0; x < NAV_ARROW_W; x++)
            g_arrowIcon->drawPixel(x, y, NAV_ARROW_PX[y * NAV_ARROW_W + x]);
}

void ui_draw_arrow_icon(LGFX_Sprite &dst, int x, int y, float angleDeg, float scale) {
  ensureArrowIcon(dst);
  /* Push to the EXPLICIT dst (mapWorld), not the icon's parent: when the arrow
   * PNG is loaded from SD its parent is mapSprite, and map_render() would
   * overwrite that sprite and erase the arrow. Same pattern as ui_icon_push. */
  g_arrowIcon->pushRotateZoomWithAA(&dst, x, y, angleDeg, scale, scale, NAV_ARROW_TRANSP);
}

/* ================= generic icon cache (ui_icons.h: white Material icons) === */
static LGFX_Sprite *ui_icon_get(const char *name) {
  for (int i = 0; i < s_iconCacheCount; i++)
    if (!strcmp(s_iconCache[i].name, name)) return s_iconCache[i].spr;

  LGFX_Sprite *spr = nullptr;

  /* 1) try the SD card PNG first (/sdcard/icon/<name>.png) */
  char path[64];
  snprintf(path, sizeof path, "/sdcard/icon/%s.png", name);
  FILE *f = fopen(path, "rb");
  if (f) {
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len > 0 && len <= 256 * 1024) {
      uint8_t *buf = (uint8_t *)malloc(len);
      if (buf) {
        if (fread(buf, 1, len, f) == (size_t)len) {
          spr = new LGFX_Sprite(&mapSprite);
          spr->setColorDepth(16);
          spr->createSprite(UI_ICON_SIZE, UI_ICON_SIZE);
          spr->fillSprite(UI_ICON_TRANSP);
          if (!spr->drawPng(buf, len, 0, 0)) {
            spr->deleteSprite(); delete spr; spr = nullptr;
          } else {
            ESP_LOGI("ui", "icon %s from SD", name);
          }
        }
        free(buf);
      }
    }
    fclose(f);
  }

  /* 2) fallback: embedded RGB565 C-array */
  if (!spr) {
    for (int i = 0; UI_ICONS[i].name; i++) {
      if (!strcmp(UI_ICONS[i].name, name)) {
        spr = new LGFX_Sprite(&mapSprite);
        spr->setColorDepth(16);
        spr->createSprite(UI_ICONS[i].w, UI_ICONS[i].h);
        for (int y = 0; y < UI_ICONS[i].h; y++)
          for (int x = 0; x < UI_ICONS[i].w; x++)
            spr->drawPixel(x, y, UI_ICONS[i].px[y * UI_ICONS[i].w + x]);
        ESP_LOGI("ui", "icon %s from C-array", name);
        break;
      }
    }
  }

  if (spr && s_iconCacheCount < UI_ICON_CACHE_MAX) {
    s_iconCache[s_iconCacheCount].name = name;
    s_iconCache[s_iconCacheCount].spr  = spr;
    s_iconCacheCount++;
  }
  return spr;
}

void ui_icon_push(const char *name, LovyanGFX *dst, int x, int y, float scale) {
  LGFX_Sprite *spr = ui_icon_get(name);
  if (!spr) return;
  spr->pushRotateZoomWithAA(dst, x, y, 0.0f, scale, scale, UI_ICON_TRANSP);
}

void ui_icons_free_all(void)
{
    if (g_arrowIcon) {
        g_arrowIcon->deleteSprite();
        delete g_arrowIcon;
        g_arrowIcon = nullptr;
    }
    /* release cached PNG/C-array icons */
    for (int i = 0; i < s_iconCacheCount; i++) {
        if (s_iconCache[i].spr) {
            s_iconCache[i].spr->deleteSprite();
            delete s_iconCache[i].spr;
            s_iconCache[i].spr = nullptr;
        }
    }
    s_iconCacheCount = 0;
}
