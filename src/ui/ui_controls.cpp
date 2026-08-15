/**
 * ui_controls.cpp — on-screen controls + overlays module.
 */
#include "ui_controls.h"
#include "app_config.h"
#include "display_panel.h"
#include "mercator.h"
#include "ble_nav.h"
#include "ble_scan.h"
#include "map_view.h"
#include "wifi_net.h"
#include <Arduino.h>
#include <string.h>
#include "esp_log.h"
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include "nav_arrow.h"
#include "nav_font_vn.h"
#include "ui_icons.h"

static bool s_uiDirty  = true;
static LGFX_Sprite *g_arrowIcon = nullptr;   /* vehicle marker (nav_arrow.h / SD) */

/* generic icon cache (ui_icons.h): SD PNG first, embedded C-array fallback */
#define UI_ICON_CACHE_MAX 10
static struct { const char *name; LGFX_Sprite *spr; } s_iconCache[UI_ICON_CACHE_MAX];
static int s_iconCacheCount = 0;

/* Smooth arrow heading: the GPS POS frame arrives ~1 Hz, so drawing the arrow
 * at the raw heading each time makes it JUMP between fixes. Instead we keep the
 * currently-drawn heading and ease it toward the target every redraw (the loop
 * redraws far faster than GPS), so the arrow glides smoothly like a real car.
 * Angles are kept in [0,360); s_arrowValid guards the very first fix. */
static float  s_arrowHdg   = 0.0f;
static bool   s_arrowValid = false;
#define ARROW_EASE_FACTOR  0.22f   /* per-redraw fraction toward the target */
#define ARROW_DEADBAND_DEG 3.0f    /* ignore heading wobble < 3deg (no arrow jitter while the map holds) */

bool ui_needs_redraw(void) { return s_uiDirty; }
void ui_clear_redraw(void) { s_uiDirty = false; }
/* Buttons/settings are composed into the map frame (drawMap -> ui_draw_buttons
 * on mapSprite), so any UI-state change must re-render the map frame. */
void ui_mark_redraw(void)  { s_uiDirty = true; s_mapDirty = true; }

/* ---- settings (gear) panel: WiFi connect + brightness slider ---- */
static bool s_settingsOpen = false;
static int  s_brightness   = BRIGHTNESS_DEFAULT;

/* Bottom-bar style, toggled from the settings panel. The next + next-next
 * guidance is ALWAYS drawn over the full map (no separate 2-step mode). */
static int s_navMode = UI_MODE_FULL;
static const char *ui_nav_mode_label(void)
{
    return (s_navMode == UI_MODE_SIMPLE) ? "SIMPLE" : "FULL";
}
int  ui_nav_mode(void) { return s_navMode; }
void ui_cycle_nav_mode(void)
{
    s_navMode = (s_navMode == UI_MODE_SIMPLE) ? UI_MODE_FULL : UI_MODE_SIMPLE;
    Serial.printf("[ui] nav mode %d (%s)\n", s_navMode, ui_nav_mode_label());
    ui_mark_redraw();
}

void ui_toggle_settings(void)
{
    s_settingsOpen = !s_settingsOpen;
    Serial.printf("[ui] settings %s\n", s_settingsOpen ? "open" : "closed");
    ui_mark_redraw();
}

bool ui_settings_open(void) { return s_settingsOpen; }

static void ui_set_brightness(int b)
{
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    s_brightness = b;
    display.setBrightness((uint8_t)b);
    Serial.printf("[ui] brightness %d\n", b);
}

bool ui_settings_tap(int x, int y)
{
    if (!s_settingsOpen) return false;
    if (y < SETTINGS_PANEL_Y)            /* tapped above the panel -> close it */
    {
        s_settingsOpen = false;
        ui_mark_redraw();
        return true;
    }
    if (x >= WIFI_BTN_X && x < WIFI_BTN_X + WIFI_BTN_W &&
        y >= WIFI_BTN_Y && y < WIFI_BTN_Y + WIFI_BTN_H)
    {
        wifi_net_connect();
        ui_mark_redraw();
        return true;
    }
    /* GPS broadcast toggle (settings panel) */
    if (x >= GPS_BTN_X && x < GPS_BTN_X + GPS_BTN_W &&
        y >= GPS_BTN_Y && y < GPS_BTN_Y + GPS_BTN_H)
    {
        navSetGpsBroadcast(!navGpsBroadcast());
        ui_mark_redraw();
        return true;
    }
    /* nav HUD mode cycle (settings panel) */
    if (x >= MODE_BTN_X && x < MODE_BTN_X + MODE_BTN_W &&
        y >= MODE_BTN_Y && y < MODE_BTN_Y + MODE_BTN_H)
    {
        ui_cycle_nav_mode();
        return true;
    }
    /* rotation quality: crisp (AA) vs fast (nearest-neighbour) */
    if (x >= AA_BTN_X && x < AA_BTN_X + AA_BTN_W &&
        y >= AA_BTN_Y && y < AA_BTN_Y + AA_BTN_H)
    {
        map_set_aa(!map_aa_enabled());
        ui_mark_redraw();
        return true;
    }
    if (y >= SLIDER_Y && y < SLIDER_Y + SLIDER_H)
    {
        ui_set_brightness((x - SLIDER_X) * 255 / SLIDER_W);
        ui_mark_redraw();
        return true;
    }
    return true;                        /* inside the panel but not on a control */
}

bool ui_settings_drag_started(int downX, int downY)
{
    return s_settingsOpen && downY >= SLIDER_Y && downY < SLIDER_Y + SLIDER_H;
}

void ui_settings_slider_drag(int x)
{
    ui_set_brightness((x - SLIDER_X) * 255 / SLIDER_W);
    ui_mark_redraw();
}

void ui_init(void)
{
    s_uiDirty  = true;
}

/* ===== DEBUG (temp): verify FontVN glyph coverage. Renders each Vietnamese
 * char + the em-dash separator individually and reports ink + bounding box:
 * ink≈0 => glyph missing from the font. NOTE: uses ESP_LOGI (Serial.* does not
 * reach the host on this USB-Serial/JTAG board). ===== */
void ui_font_selftest(void)
{
    static const char *chars = "ệộạồừĐễợưươớáéíóúýàèìòùâêôơậặằẵăń–—·Ag";
    LGFX_Sprite spr(&display);
    spr.setPsram(true);
    spr.setColorDepth(16);
    spr.createSprite(48, 32);   /* taller + wider: unifont is 16px, don't clip */
    spr.setFont(&FontVN);
    spr.setTextSize(1);
    spr.setTextColor(0xFFFF, 0x0000);
    ESP_LOGI("font", "SELFTEST start (48x32, cursor 2,26)");
    for (const char *p = chars; *p; ) {
        /* decode one UTF-8 char */
        uint8_t c = (uint8_t)*p;
        int len = 1;
        if      (c >= 0xF0) { len = 4; }
        else if (c >= 0xE0) { len = 3; }
        else if (c >= 0xC0) { len = 2; }
        char one[8] = {0};
        memcpy(one, p, len);
        p += len;

        spr.fillSprite(0x0000);
        spr.setCursor(2, 26);
        spr.print(one);
        uint32_t ink = 0; int minx = 99, miny = 99, maxx = -1, maxy = -1;
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 48; x++)
                if (spr.readPixelValue(x, y) != 0x0000) {
                    ink++;
                    if (x < minx) { minx = x; }
                    if (x > maxx) { maxx = x; }
                    if (y < miny) { miny = y; }
                    if (y > maxy) { maxy = y; }
                }
        ESP_LOGI("font", "SELFTEST %s ink=%u box=(%d,%d)-(%d,%d)",
                 one, (unsigned)ink, minx, miny, maxx, maxy);
        /* ASCII-art shape dump for a few key chars so we can SEE the glyph */
        if (!strcmp(one, "ệ") || !strcmp(one, "ạ") || !strcmp(one, "A")) {
            for (int y = 0; y < 32; y++) {
                char row[49];
                for (int x = 0; x < 48; x++)
                    row[x] = (spr.readPixelValue(x, y) != 0x0000) ? '#' : '.';
                row[48] = 0;
                ESP_LOGI("font", "SHAPE %s |%s|", one, row);
            }
        }
    }
    spr.deleteSprite();
    ESP_LOGI("font", "SELFTEST end");
}

void ui_deinit(void)
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

/* ================= Google arrow icon (nav_arrow.h / SD card PNG) =================
 * Renders live car marker on map. Uses /sdcard/icon/nav_arrow.png if available on SD,
 * falling back to the embedded NAV_ARROW_PX RGB565 C-array.
 */

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

static void drawIconRotated(LGFX_Sprite &dst, int x, int y, float angleDeg, float scale) {
  ensureArrowIcon(dst);
  /* Push to the EXPLICIT dst (mapWorld), not the icon's parent: when the arrow
   * PNG is loaded from SD its parent is mapSprite, and map_render() would
   * overwrite that sprite and erase the arrow. Same pattern as ui_icon_push. */
  g_arrowIcon->pushRotateZoomWithAA(&dst, x, y, angleDeg, scale, scale, NAV_ARROW_TRANSP);
}

/* ================= generic icon cache (ui_icons.h: white Material icons) =
 * Each icon is loaded ONCE: /sdcard/icon/<name>.png if present, else the
 * embedded UI_ICONS[] C-array. Sprites are cached for the app lifetime;
 * ui_deinit() frees them. Pushing uses an explicit destination so the same
 * sprite works for the overlay sprite (mapSprite) and the LCD (display).
 */

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

/* Push an icon sprite centered at (x,y), scaled by `scale`, to `dst`. */
static void ui_icon_push(const char *name, LovyanGFX *dst, int x, int y, float scale) {
  LGFX_Sprite *spr = ui_icon_get(name);
  if (!spr) return;
  spr->pushRotateZoomWithAA(dst, x, y, 0.0f, scale, scale, UI_ICON_TRANSP);
}

/* ================= next-movement banner arrow =================
 * Green rounded badge with a white turn arrow (like the navbridge back-arrow):
 * straight up for straight/arrive, bent left/right for turns, left for u-turn.
 */
static void drawBannerArrow(LGFX_Sprite &dst, int cx, int cy, const char *man, float scale = 0.7f) {
  /* White Material turn arrow (banner uses 0.7f; the 2-step HUD passes bigger). */
  const char *icon = "straight";
  if      (!strcmp(man, "left"))         icon = "turn_left";
  else if (!strcmp(man, "slight-left"))  icon = "turn_slight_left";
  else if (!strcmp(man, "right"))        icon = "turn_right";
  else if (!strcmp(man, "slight-right")) icon = "turn_slight_right";
  else if (!strcmp(man, "u-turn"))       icon = "u_turn";
  ui_icon_push(icon, &dst, cx, cy, scale);
}

/* Build the guidance line: show the FULL street name. The turn arrow badge
 * already shows the maneuver, so the verbose "Turn left onto" / "Stay on"
 * prefixes are dropped — they were eating the banner width and forcing the
 * street name to be truncated. */
static void buildInstruction(const NavManeuver *man, char *out, size_t n) {
  const char *street = man->street; /* raw UTF-8 — rendered by FontVN */
  const char *m = man->maneuver;
  if (street[0]) {
    snprintf(out, n, "%s", street);
    return;
  }
  /* no street info — fall back to a short maneuver label */
  const char *lbl = "Straight";
  if      (!strcmp(m, "left"))         lbl = "Turn left";
  else if (!strcmp(m, "right"))        lbl = "Turn right";
  else if (!strcmp(m, "slight-left"))  lbl = "Slight left";
  else if (!strcmp(m, "slight-right")) lbl = "Slight right";
  else if (!strcmp(m, "u-turn"))       lbl = "U-turn";
  else if (!strcmp(m, "roundabout"))   lbl = "Roundabout";
  else if (!strcmp(m, "arrive"))       lbl = "Arrive";
  snprintf(out, n, "%s", lbl);
}

/* ---- weather: procedural icon + compact widget in its own box (top-right,
 *      below the 100px guidance panel, so it never overlaps the guidance) ---- */
static void drawWeatherIcon(LGFX_Sprite &spr, int cx, int cy, int code)
{
  const uint16_t SUN   = spr.color565(255, 214, 70);
  const uint16_t CLOUD = spr.color565(214, 218, 224);
  const uint16_t RAIN  = spr.color565(120, 180, 255);
  const uint16_t BOLT  = spr.color565(255, 200, 40);

  int type;   /* classify WMO weather code -> icon type */
  if (code == 0) type = 0;                                      /* clear */
  else if (code <= 2) type = 1;                                 /* partly cloudy */
  else if (code <= 3) type = 2;                                 /* overcast */
  else if (code <= 48) type = 3;                                /* fog */
  else if (code <= 67 || (code >= 80 && code <= 82)) type = 4;  /* rain */
  else if (code <= 77) type = 5;                                /* snow */
  else type = 6;                                                /* thunderstorm */

  if (type == 0 || type == 1) {                                 /* sun */
    spr.fillCircle(cx, cy, 7, SUN);
    for (int a = 0; a < 8; a++) {
      float r = a * M_PI / 4.0f;
      int x0 = cx + (int)roundf(11.0f * cosf(r));
      int y0 = cy + (int)roundf(11.0f * sinf(r));
      int x1 = cx + (int)roundf(16.0f * cosf(r));
      int y1 = cy + (int)roundf(16.0f * sinf(r));
      spr.drawLine(x0, y0, x1, y1, SUN);
    }
  }
  if (type >= 1) {                                              /* cloud */
    spr.fillCircle(cx - 1, cy + 1, 8, CLOUD);
    spr.fillCircle(cx - 6, cy + 3, 6, CLOUD);
    spr.fillCircle(cx + 5, cy + 3, 6, CLOUD);
    spr.fillRect(cx - 10, cy + 2, 21, 6, CLOUD);
    if (type == 1) spr.fillCircle(cx - 1, cy + 1, 6, SUN);      /* sun peeking */
  }
  if (type == 4) {                                              /* rain */
    for (int i = 0; i < 3; i++) {
      int dx = cx - 5 + i * 5;
      spr.drawLine(dx, cy + 9, dx - 2, cy + 14, RAIN);
    }
  } else if (type == 5) {                                       /* snow */
    for (int i = 0; i < 3; i++) spr.fillCircle(cx - 5 + i * 5, cy + 11, 2, TFT_WHITE);
  } else if (type == 6) {                                       /* thunderstorm */
    spr.fillTriangle(cx, cy + 8, cx + 7, cy + 8, cx + 2, cy + 14, BOLT);
    spr.fillTriangle(cx + 3, cy + 10, cx + 9, cy + 10, cx + 6, cy + 16, BOLT);
  } else if (type == 3) {                                       /* fog */
    spr.drawLine(cx - 8, cy + 9, cx + 8, cy + 9, CLOUD);
    spr.drawLine(cx - 6, cy + 12, cx + 6, cy + 12, CLOUD);
  }
}

static void drawWeatherWidget(LGFX_Sprite &spr)
{
  const NavWeather *wx = navGetWeather();
  if (!wx || !wx->valid) return;

  const uint16_t BG = 0x2104;                  /* dark graphite (matches banner) */
  const int w = 98, h = 40;
  const int bx = SCREEN_W - w - 6;
  const int by = 50;                           /* below the 46px guidance strip */
  spr.fillRoundRect(bx, by, w, h, 10, BG);
  spr.drawRoundRect(bx, by, w, h, 10, 0x39E7); /* subtle border */

  drawWeatherIcon(spr, bx + 18, by + 18, wx->code);

  spr.setTextColor(TFT_WHITE, BG);
  spr.setTextFont(2);
  spr.setCursor(bx + 36, by + 7);
  spr.print(wx->tempC);
  spr.print("C");
  spr.setTextFont(1);
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(bx + 36, by + 27);
  spr.print(wx->humidity);
  spr.print("%");
}

/* ---- bottom bar: speed / time / ETA strip ---- */
static void drawBottomBar(LGFX_Sprite &spr) {
  const NavPos   *pos  = navGetPos();
  const NavEta   *eta  = navGetEta();
  const NavClock *clk  = navGetClock();

  bool hasPos = (pos && pos->valid);
  bool hasClk = (clk && clk->valid);
  bool hasEta = (eta && eta->valid);
  if (!hasPos && !hasClk && !hasEta) return;   /* only show when there is data */

  const uint16_t BG = 0x2104;                /* dark graphite (black-ish) */
  /* Keep the pill clear of the zoom-button column (ZOOM_*_X = SCREEN_W - 50 =
   * 270). bw=216 centers it at x52..268, so it never covers the "-" button
   * (was bw=248 -> x36..284, which overlapped the zoom-out button). */
  int bw = 216, bh = 32;
  int bx = (SCREEN_W - bw) / 2;              /* centered pill */
  int by = SCREEN_H - bh - 8;
  spr.fillRoundRect(bx, by, bw, bh, 16, BG);
  spr.setTextColor(TFT_WHITE, BG);

  if (ui_nav_mode() == UI_MODE_SIMPLE) {
    /* simple: one consistent Font2 row — speed | time | ETA */
    spr.setTextFont(2);
    if (hasPos) { spr.setCursor(bx + 16, by + 8); spr.print(pos->spd); spr.print("k"); }
    if (hasClk) {
      spr.setCursor(bx + 84, by + 8);
      if (clk->hour < 10) spr.print("0");
      spr.print(clk->hour); spr.print(":");
      if (clk->minute < 10) spr.print("0");
      spr.print(clk->minute);
    }
    if (hasEta) {
      spr.setCursor(bx + 140, by + 8);
      spr.print("E ");
      if (eta->hour < 10) spr.print("0");
      spr.print(eta->hour); spr.print(":");
      if (eta->minute < 10) spr.print("0");
      spr.print(eta->minute);
    }
    spr.setTextFont(1);
    return;
  }

  /* full mode: speed-limit & speedometer icons, then clock + ETA BOTH Font2
   * (same size, so they line up and read consistently). */
  spr.setTextFont(1);
  if (hasPos) {
    bool hasSign = (pos->limit > 0);
    if (hasSign) {
      int lx = bx + 24, ly = by + 16;
      /* red-ring road-sign PNG (Material-style); number overlaid in the centre */
      ui_icon_push("speed_limit", &spr, lx, ly, 0.65f);
      int v = pos->limit;
      spr.setTextColor(TFT_BLACK);
      if (v >= 100) { spr.setTextFont(1); spr.setCursor(lx - 12, ly - 4); }
      else          { spr.setTextFont(2); spr.setCursor(lx - (v >= 10 ? 10 : 5), ly - 8); }
      spr.print(v);
      spr.setTextFont(1);
      spr.setTextColor(TFT_WHITE, BG);
    }
    /* speedometer icon + live speed (km/h) */
    ui_icon_push("speed", &spr, hasSign ? bx + 46 : bx + 18, by + 16, 0.4f);
    spr.setCursor(hasSign ? bx + 56 : bx + 30, by + 12);
    spr.print(pos->spd);                       /* km/h implied (Vietnam) */
  }

  /* current time — Font2 */
  if (hasClk) {
    spr.setTextFont(2);
    spr.setCursor(bx + 92, by + 8);
    if (clk->hour < 10) spr.print("0");
    spr.print(clk->hour); spr.print(":");
    if (clk->minute < 10) spr.print("0");
    spr.print(clk->minute);
  }
  /* ETA — SAME Font2 as the clock (previously Font1 → inconsistent size) */
  if (hasEta) {
    spr.setTextFont(2);
    spr.setCursor(bx + 140, by + 8);
    spr.print("ETA ");
    if (eta->hour < 10) spr.print("0");
    spr.print(eta->hour); spr.print(":");
    if (eta->minute < 10) spr.print("0");
    spr.print(eta->minute);
  }
  spr.setTextFont(1);
  spr.setTextColor(TFT_WHITE, BG);
}

/* Draw the full route the client sends (matches firmware NAV_MAX_ROUTE_POINTS
 * = 256). A plain loop of bounded-per-call wide-line segments is fine on the
 * 8KB loopTask stack; keep an eye on the stack-high-water log if routes get
 * much longer. */
#define MAX_ROUTE_DRAW_PTS 256

/* ---- nav route: drawn on the NORTH-UP world sprite so it rotates with the
 *      map. `spr` (mapWorld) is centered on (refLon,refLat) — the point the
 *      world was last composed at (map_ref_lon/lat) — so the route aligns with
 *      the tiles even while the view scrolls via the blit offset. Called only
 *      when the world is recomposed so the route can't ghost. The car marker
 *      is drawn separately, screen-fixed (ui_draw_nav_marker). ---- */
void ui_draw_nav_route(LGFX_Sprite &spr, double refLon, double refLat, int zoom)
{
  const NavRoute *rt   = navGetRoute();

  double tlX = lon2wx(refLon, zoom) - spr.width() / 2.0;
  double tlY = lat2wy(refLat, zoom) - spr.height() / 2.0;

  /* helper: draw a route polyline as a thick lane. */
  auto drawPoly = [&](const NavRoute *r, float casing, float coreW,
                      uint16_t coreCol, uint16_t casingCol) {
    if (!r || r->count < 2) return;
    int px = 0, py = 0;
    int n = r->count;
    if (n > MAX_ROUTE_DRAW_PTS) n = MAX_ROUTE_DRAW_PTS;
    for (int i = 0; i < n; i++) {
      int sx = (int)lround(lon2wx(r->pts[i].lon, zoom) - tlX);
      int sy = (int)lround(lat2wy(r->pts[i].lat, zoom) - tlY);
      if (i > 0) {
        spr.drawWideLine(px, py, sx, sy, casing, casingCol);
        spr.drawWideLine(px, py, sx, sy, coreW, coreCol);
      }
      px = sx; py = sy;
    }
  };

  /* 0) route CONTINUATION drawn first (faint grey) — where the road goes next,
   *    beyond the near path-ahead. The bright near route draws over it. */
  const NavRoute *rtc = navGetRouteCont();
  if (rtc && rtc->count >= 2) {
    ESP_LOGI("ui", "route-cont draw: %d pts", rtc->count);
    drawPoly(rtc, 2.0f, 1.2f, spr.color565(0xB0, 0xB8, 0xC4),
             spr.color565(0x80, 0x88, 0x94));
  }

  /* 1) route as a thick "lane" (navy casing + bright blue core), ~street width */
  if (rt && rt->count >= 2) {
    uint16_t core = spr.color565(0x1A, 0x73, 0xE8);   /* bright Google blue */
    int px = 0, py = 0;
    int n = rt->count;
    if (n > MAX_ROUTE_DRAW_PTS) n = MAX_ROUTE_DRAW_PTS;   /* small path ahead only */
    ESP_LOGI("ui", "route draw: %d pts, first screen=(%d,%d) zoom=%d", n,
             (int)lround(lon2wx(rt->pts[0].lon, zoom) - tlX),
             (int)lround(lat2wy(rt->pts[0].lat, zoom) - tlY), zoom);
    for (int i = 0; i < n; i++) {
      int sx = (int)lround(lon2wx(rt->pts[i].lon, zoom) - tlX);
      int sy = (int)lround(lat2wy(rt->pts[i].lat, zoom) - tlY);
      if (i > 0) {
        /* Thick lane via wide lines: casing (5 px) then core (3 px). One call
         * per segment instead of the old 18 drawLine offset-loop -> far less
         * PSRAM traffic during the redraw burst, so core 0's BLE controller
         * ISR isn't starved (was tripping "Interrupt wdt timeout on CPU0"). */
        spr.drawWideLine(px, py, sx, sy, 2.5f, TFT_NAVY);
        spr.drawWideLine(px, py, sx, sy, 1.5f, core);
      }
      px = sx; py = sy;
    }
  }

}

/* ---- car marker: screen-fixed, drawn AFTER map_render() ----
 * The view is always centered on the car, so the arrow goes at the sprite
 * center. Its screen rotation = eased GPS heading + the map's own rotation
 * (up in heading-up mode, along the road in north-up — same as drawing it on
 * the world then rotating). map_render() fully overwrites mapSprite each
 * frame, so this never leaves a ghost. */
void ui_draw_nav_marker(LGFX_Sprite &spr)
{
  const NavPos *pos = navGetPos();
  if (!pos || !pos->valid) return;

  int sx = spr.width() / 2, sy = spr.height() / 2;
  spr.drawCircle(sx, sy, 18, TFT_WHITE);   /* contrast ring */
  /* Ease the drawn heading toward the target GPS heading (shortest-path
   * wrap) so the arrow rotates smoothly instead of jumping every fix. */
  float target = (float)pos->hdg;
  if (!s_arrowValid) {
    s_arrowHdg   = target;
    s_arrowValid = true;
  } else {
    float d = (float)fmod((double)(target - s_arrowHdg + 540.0f), 360.0f) - 180.0f;
    /* deadband: hold the arrow still for small heading wobble so it doesn't
     * shimmer while the map is otherwise static */
    if (fabsf(d) > ARROW_DEADBAND_DEG)
      s_arrowHdg = fmod((double)(s_arrowHdg + d * ARROW_EASE_FACTOR + 360.0f), 360.0f);
  }
  drawIconRotated(spr, sx, sy, s_arrowHdg + map_rotation(), 1.0f);
}

/* compact text label for a maneuver — used in SIMPLE (text-only) mode so the
 * turn is clear without the arrow icon */
static const char *maneuverText(const char *m) {
  if      (!strcmp(m, "left"))         return "Turn left";
  else if (!strcmp(m, "right"))        return "Turn right";
  else if (!strcmp(m, "slight-left"))  return "Slight left";
  else if (!strcmp(m, "slight-right")) return "Slight right";
  else if (!strcmp(m, "u-turn"))       return "U-turn";
  else if (!strcmp(m, "roundabout"))   return "Roundabout";
  else if (!strcmp(m, "arrive"))       return "Arrive";
  return "Go straight";
}

/* ---- next + next-next guidance, drawn over the FULL map ----
 * A THIN top strip (like a classic nav banner) so it barely covers the map:
 * the next-maneuver arrow badge on the LEFT, the street on the top line and
 * "dist · then <next-next>" on the bottom line. Both lines use the Vietnamese
 * font so accents render correctly (Font1 is ASCII-only). Uses
 * navGetManeuver() / navGetManeuver2() (BLE 0x03/0x08). */
static void drawTwoStepHUD(LGFX_Sprite &spr)
{
  const NavManeuver *n1 = navGetManeuver();
  if (!n1 || !n1->valid) return;
  const NavManeuver *n2 = navGetManeuver2();

  const uint16_t BG = 0x2104;                 /* strip background */
  const int ph = 46;                           /* thin guidance strip */
  spr.fillRect(0, 0, SCREEN_W, ph, BG);
  spr.drawRect(0, 0, SCREEN_W - 1, ph, 0x39E7);

  /* 1) next-maneuver arrow badge on the LEFT (FULL mode's guidance strip) */
  drawBannerArrow(spr, 32, ph / 2, n1->maneuver, 0.8f);

  /* 2) next street name (full, Vietnamese) — top line */
  char instr[NAV_MAX_STREET + 24];
  buildInstruction(n1, instr, sizeof instr);
  spr.setTextColor(TFT_WHITE, BG);
  spr.setFont(&FontVN);
  spr.setTextSize(1);
  spr.setCursor(56, 3);
  spr.print(instr);

  /* 3) distance · then next-next street — bottom line, SAME Vietnamese font */
  char line[NAV_MAX_STREET * 2 + 40];
  int used = 0;
  if (n1->dist > 0) used = snprintf(line, sizeof line, "%d m", n1->dist);
  else              line[0] = 0;
  if (n2 && n2->valid) {
    const char *s2 = n2->street[0] ? n2->street : n2->maneuver;
    used = snprintf(line + used, sizeof line - used,
                    "%s%s", used ? " - then " : "then ", s2);
    if (n2->dist > 0)
      snprintf(line + used, sizeof line - used, " (%d m)", n2->dist);
  }
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(56, 27);
  spr.print(line);

  /* tiny mode tag (top-right of the strip) so toggling NAV mode is clearly
   * visible — "FULL" vs "SIMPLE" flips right here when the button is tapped */
  spr.setTextFont(1);
  spr.setTextColor(0xCED8, BG);
  spr.setCursor(SCREEN_W - 52, 4);
  spr.print(ui_nav_mode_label());
  spr.setTextSize(1);
}

/* ---- screen-fixed HUD: maneuver banner + weather + bottom bar.
 *      Drawn onto the visible sprite AFTER map_render() so these stay upright
 *      while the map turns beneath them. ---- */
/* ---- SIMPLE mode: text-only navigation screen (NO map) ----
 * Fills the whole screen: time/ETA up top, a BIG turn arrow in the middle
 * (same Material icons as the FULL banner — "arrow in the middle" the user
 * asked for), the maneuver + street, distance, next-next, and speed. EVERY
 * line uses FontVN so Vietnamese accents render (Font1/Font2 are ASCII-only).
 * drawMap() does NOT fetch or render the map in this mode — the route is
 * still cached. */
static void drawTextOnlyHUD(LGFX_Sprite &spr)
{
  const uint16_t BG = 0x2104;
  spr.fillScreen(BG);

  const NavManeuver *n1 = navGetManeuver();
  const NavPos   *pos = navGetPos();
  const NavClock *clk = navGetClock();
  const NavEta   *eta = navGetEta();

  spr.setFont(&FontVN);
  spr.setTextSize(1);

  /* top line: time - ETA (ASCII separators — some non-Latin-1 glyphs like
   * the middle-dot/em-dash render as boxes in the unifont subset) */
  char top[64] = "";
  if (clk && clk->valid) snprintf(top, sizeof top, "%02d:%02d", clk->hour, clk->minute);
  if (eta && eta->valid) { int n = (int)strlen(top); snprintf(top + n, sizeof top - n, "  -  ETA %02d:%02d", eta->hour, eta->minute); }
  spr.setTextColor(0xCED8, BG);
  if (top[0]) { int w = spr.textWidth(top); spr.setCursor((SCREEN_W - w) / 2, 6); spr.print(top); }

  /* weather widget, top-right (below the time/ETA line, clear of the arrow) */
  drawWeatherWidget(spr);

  if (n1 && n1->valid) {
    /* BIG next-maneuver arrow, dead centre of the screen (40px icon * 1.6) */
    drawBannerArrow(spr, SCREEN_W / 2, 74, n1->maneuver, 1.6f);

    /* maneuver + street, big text under the arrow */
    char big[NAV_MAX_STREET + 40];
    const char *mv = maneuverText(n1->maneuver);
    bool showMv = (strcmp(n1->maneuver, "straight") != 0) && (strcmp(n1->maneuver, "arrive") != 0);
    if (n1->street[0])
      snprintf(big, sizeof big, "%s%s%s", showMv ? mv : "", showMv ? " - " : "", n1->street);
    else
      snprintf(big, sizeof big, "%s", mv);
    spr.setTextColor(TFT_WHITE, BG);
    int w = spr.textWidth(big);
    spr.setCursor((SCREEN_W - w) / 2, 124);
    spr.print(big);

    /* distance */
    if (n1->dist > 0) {
      char d[16]; snprintf(d, sizeof d, "%d m", n1->dist);
      spr.setTextColor(0xCED8, BG);
      int w2 = spr.textWidth(d);
      spr.setCursor((SCREEN_W - w2) / 2, 146);
      spr.print(d);
    }

    /* next-next: small arrow + "then <street> · dist" on ONE centred line */
    const NavManeuver *n2 = navGetManeuver2();
    if (n2 && n2->valid) {
      char nn[NAV_MAX_STREET + 40];
      const char *s2 = n2->street[0] ? n2->street : maneuverText(n2->maneuver);
      snprintf(nn, sizeof nn, "then %s", s2);
      if (n2->dist > 0) { char d2[24]; snprintf(d2, sizeof d2, "  -  %d m", n2->dist); strncat(nn, d2, sizeof nn - strlen(nn) - 1); }
      spr.setTextColor(0xCED8, BG);
      int w3 = spr.textWidth(nn);
      const int arrowW = 20, gap = 6;   /* 40px icon * 0.5 + spacer */
      int groupW = w3 + arrowW + gap;
      int gx = (SCREEN_W - groupW) / 2;
      drawBannerArrow(spr, gx + arrowW / 2, 178, n2->maneuver, 0.5f);
      spr.setCursor(gx + arrowW + gap, 170);
      spr.print(nn);
    }
  }

  /* bottom: red speed-limit sign + speedometer + live speed (centred group) */
  if (pos && pos->valid) {
    const int by = 204;
    bool hasSign = (pos->limit > 0);
    int scx  = SCREEN_W / 2 - 26;                  /* red-ring sign centre */
    int spdX = SCREEN_W / 2 + (hasSign ? 10 : 0);  /* speedometer centre   */
    if (hasSign) {
      ui_icon_push("speed_limit", &spr, scx, by + 8, 0.65f);
      int v = pos->limit;
      spr.setTextColor(TFT_BLACK);
      if (v >= 100) { spr.setTextFont(1); spr.setCursor(scx - 12, by + 4); }
      else          { spr.setTextFont(2); spr.setCursor(scx - (v >= 10 ? 10 : 5), by); }
      spr.print(v);
      spr.setTextFont(1);
      spr.setTextColor(TFT_WHITE, BG);
    }
    ui_icon_push("speed", &spr, spdX, by + 8, 0.4f);
    spr.setTextFont(2);
    spr.setTextColor(TFT_WHITE, BG);
    spr.setCursor(spdX + 10, by + 4);
    spr.print(pos->spd);
    spr.setTextFont(1);
  }
  spr.setTextFont(1);
}

void ui_draw_nav_hud(LGFX_Sprite &spr)
{
  /* SIMPLE = text-only navigation screen (no map). */
  if (ui_nav_mode() == UI_MODE_SIMPLE) {
    drawTextOnlyHUD(spr);
    return;
  }

  /* FULL: guidance strip + weather + bottom bar over the map */
  const NavManeuver *man = navGetManeuver();
  if (man && man->valid) drawTwoStepHUD(spr);
  drawWeatherWidget(spr);
  drawBottomBar(spr);
}

/* ---- corner settings (gear) button ---- */
static void drawGearButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(GEAR_BTN_X, GEAR_BTN_Y, GEAR_BTN_W, GEAR_BTN_H, bg);
  dst->drawRect(GEAR_BTN_X, GEAR_BTN_Y, GEAR_BTN_W - 1, GEAR_BTN_H - 1, TFT_BLACK);
  /* white Material settings gear, scaled to the 42x22 button */
  int cx = GEAR_BTN_X + GEAR_BTN_W / 2, cy = GEAR_BTN_Y + GEAR_BTN_H / 2;
  ui_icon_push("gear", dst, cx, cy, 0.5f);
}

/* ---- rotate button (left edge): tap to turn the map ROTATE_STEP_DEG clockwise ---- */
static void drawRotateButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(ROTATE_BTN_X, ROTATE_BTN_Y, ROTATE_BTN_W, ROTATE_BTN_H, bg);
  dst->drawRect(ROTATE_BTN_X, ROTATE_BTN_Y, ROTATE_BTN_W - 1, ROTATE_BTN_H - 1, TFT_BLACK);
  int cx = ROTATE_BTN_X + ROTATE_BTN_W / 2;
  int cy = ROTATE_BTN_Y + ROTATE_BTN_H / 2;
  /* circular "rotate" glyph: ring + clockwise arrowhead at the top */
  const int R = 6;
  dst->drawCircle(cx, cy, R, TFT_BLACK);
  dst->fillTriangle(cx + 6, cy - R, cx, cy - R - 3, cx, cy - R + 3, TFT_BLACK);
  dst->fillCircle(cx, cy, 1, TFT_BLACK);
}

/* ---- heading-up toggle (left edge, under rotate): green + "H" when the map
 *      auto-rotates with GPS heading, grey + "N" when fixed north-up ---- */
static void drawHeadingButton(LovyanGFX *dst) {
  bool on = map_heading_up();
  uint16_t bg = on ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
  dst->fillRect(HDG_BTN_X, HDG_BTN_Y, HDG_BTN_W, HDG_BTN_H, bg);
  dst->drawRect(HDG_BTN_X, HDG_BTN_Y, HDG_BTN_W - 1, HDG_BTN_H - 1, TFT_BLACK);
  int cx = HDG_BTN_X + HDG_BTN_W / 2;
  int cy = HDG_BTN_Y + HDG_BTN_H / 2;
  /* up arrow (travel direction) + mode letter */
  dst->fillTriangle(cx, cy - 5, cx - 4, cy, cx + 4, cy, TFT_WHITE);
  dst->setTextColor(TFT_WHITE, bg);
  dst->setTextFont(1);
  dst->setCursor(cx - 3, cy + 1);
  dst->print(on ? "H" : "N");
}

/* ---- center button (left edge, under heading): snap the view back onto the
 *      car (ui_recenter) — crosshair glyph ---- */
static void drawCenterButton(LovyanGFX *dst) {
  uint16_t bg = dst->color565(70, 74, 82);
  dst->fillRect(CENTER_BTN_X, CENTER_BTN_Y, CENTER_BTN_W, CENTER_BTN_H, bg);
  dst->drawRect(CENTER_BTN_X, CENTER_BTN_Y, CENTER_BTN_W - 1, CENTER_BTN_H - 1, TFT_BLACK);
  int cx = CENTER_BTN_X + CENTER_BTN_W / 2;
  int cy = CENTER_BTN_Y + CENTER_BTN_H / 2;
  dst->drawCircle(cx, cy, 5, TFT_BLACK);
  dst->drawLine(cx - 9, cy, cx - 6, cy, TFT_BLACK);
  dst->drawLine(cx + 6, cy, cx + 9, cy, TFT_BLACK);
  dst->drawLine(cx, cy - 9, cx, cy - 6, TFT_BLACK);
  dst->drawLine(cx, cy + 6, cx, cy + 9, TFT_BLACK);
  dst->fillCircle(cx, cy, 1, TFT_BLACK);
}

/* ---- zoom in/out buttons (bottom-right) ---- */
static void drawZoomButtons(LovyanGFX *dst) {
  uint16_t grey  = dst->color565(80, 80, 80);   /* enabled */
  uint16_t dim   = dst->color565(38, 38, 38);   /* disabled bg */
  uint16_t glyph = TFT_BLACK;                       /* enabled glyph */
  uint16_t dimG  = dst->color565(90, 90, 90);    /* disabled glyph */
  bool canIn  = ZOOM < ZOOM_MAX;                    /* + allowed? */
  bool canOut = ZOOM > ZOOM_MIN;                    /* - allowed? */

  /* "+" */
  dst->fillRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W, ZOOM_BTN_H, canIn ? grey : dim);
  dst->drawRect(ZOOM_IN_X, ZOOM_IN_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  dst->fillRect(ZOOM_IN_X + 8,  ZOOM_IN_Y + 17, ZOOM_BTN_W - 16, 6, canIn ? glyph : dimG);
  dst->fillRect(ZOOM_IN_X + 18, ZOOM_IN_Y + 7,  6, ZOOM_BTN_H - 14, canIn ? glyph : dimG);

  /* "-" */
  dst->fillRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W, ZOOM_BTN_H, canOut ? grey : dim);
  dst->drawRect(ZOOM_OUT_X, ZOOM_OUT_Y, ZOOM_BTN_W - 1, ZOOM_BTN_H - 1, TFT_BLACK);
  dst->fillRect(ZOOM_OUT_X + 8, ZOOM_OUT_Y + 17, ZOOM_BTN_W - 16, 6, canOut ? glyph : dimG);
}

/* ---- boot splash -----------------------------------------------------------
 * Priority: 1) /sdcard/splash.ani (a GIF preprocessed by scripts/gif_to_splash
 *              -> played by a small task while the rest of setup() runs),
 *           2) a static SD image (splash.png / cat.png / splash.jpg),
 *           3) a plain deep-blue fill.
 * The .ani task is stopped via ui_splash_stop() before the first map draw.
 * -------------------------------------------------------------------------- */
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

static void ui_draw_settings_panel(LovyanGFX *dst);   /* forward decl (defined below) */

void ui_draw_buttons(LovyanGFX *dst)
{
    drawRotateButton(dst);
    drawHeadingButton(dst);
    drawCenterButton(dst);
    drawGearButton(dst);
    if (s_settingsOpen)
        ui_draw_settings_panel(dst);   /* bottom overlay covers the zoom column */
    else
        drawZoomButtons(dst);
}

/* Snap the view back onto the car (center button). */
void ui_recenter(void)
{
    const NavPos *p = navGetPos();
    if (p && p->valid)
    {
        map_center_on(p->lat, p->lon);
        Serial.println("[ui] recenter on car");
    }
}

/* ---- settings panel: WiFi + brightness (drawn over the bottom strip) ---- */
static void ui_draw_settings_panel(LovyanGFX *dst)
{
    const uint16_t panelBG = dst->color565(22, 24, 30);
    dst->fillRect(0, SETTINGS_PANEL_Y, SCREEN_W, SETTINGS_PANEL_H, panelBG);
    dst->drawRect(0, SETTINGS_PANEL_Y, SCREEN_W - 1, SETTINGS_PANEL_H - 1, TFT_WHITE);

    /* WiFi button: green=on, amber=connecting, grey=off */
    bool on      = wifi_net_connected();
    bool pending = wifi_net_pending();
    uint16_t wbg = on      ? dst->color565(40, 150, 70)
                 : pending ? dst->color565(200, 140, 20)
                           : dst->color565(70, 74, 82);
    dst->fillRect(WIFI_BTN_X, WIFI_BTN_Y, WIFI_BTN_W, WIFI_BTN_H, wbg);
    dst->drawRect(WIFI_BTN_X, WIFI_BTN_Y, WIFI_BTN_W - 1, WIFI_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, wbg);
    dst->setTextFont(1);
    dst->setCursor(WIFI_BTN_X + 6, WIFI_BTN_Y + 6);
    dst->print(wifi_net_label());

    /* GPS broadcast toggle (green when broadcasting) */
    bool gpsOn = navGpsBroadcast();
    uint16_t gbg = gpsOn ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
    dst->fillRect(GPS_BTN_X, GPS_BTN_Y, GPS_BTN_W, GPS_BTN_H, gbg);
    dst->drawRect(GPS_BTN_X, GPS_BTN_Y, GPS_BTN_W - 1, GPS_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, gbg);
    dst->setTextFont(1);
    dst->setCursor(GPS_BTN_X + 6, GPS_BTN_Y + 6);
    dst->print(gpsOn ? "GPS BCAST" : "GPS off");

    /* nav HUD mode cycle (2-STEP / FULL / SIMPLE) */
    uint16_t mbg = dst->color565(70, 74, 82);
    dst->fillRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W, MODE_BTN_H, mbg);
    dst->drawRect(MODE_BTN_X, MODE_BTN_Y, MODE_BTN_W - 1, MODE_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, mbg);
    dst->setTextFont(1);
    dst->setCursor(MODE_BTN_X + 4, MODE_BTN_Y + 6);
    dst->print("NAV ");
    dst->print(ui_nav_mode_label());

    /* rotation quality: AA (crisp) vs fast (nearest-neighbour) */
    bool aa = map_aa_enabled();
    uint16_t abg = aa ? dst->color565(40, 150, 70) : dst->color565(70, 74, 82);
    dst->fillRect(AA_BTN_X, AA_BTN_Y, AA_BTN_W, AA_BTN_H, abg);
    dst->drawRect(AA_BTN_X, AA_BTN_Y, AA_BTN_W - 1, AA_BTN_H - 1, TFT_BLACK);
    dst->setTextColor(TFT_WHITE, abg);
    dst->setTextFont(1);
    dst->setCursor(AA_BTN_X + 6, AA_BTN_Y + 6);
    dst->print(aa ? "ROT: CRISP" : "ROT: FAST");

    /* brightness slider */
    dst->setTextColor(TFT_WHITE, panelBG);
    dst->setCursor(SLIDER_X, SLIDER_Y - 12);
    dst->print("Brightness");
    dst->fillRect(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H,
                  dst->color565(60, 62, 70));
    int thumbX = SLIDER_X + (SLIDER_W - 6) * s_brightness / 255;
    dst->fillRect(thumbX, SLIDER_Y - 2, 6, SLIDER_H + 4, TFT_WHITE);
}

