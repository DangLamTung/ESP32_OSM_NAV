/**
 * nav_font_vn.h — Vietnamese-capable font for the navigation HUD.
 *
 * The built-in LovyanGFX bitmap fonts (GLCD / BMPfont) only cover ASCII, so
 * accented Vietnamese street names were previously ASCII-stripped. This header
 * provides a full Unicode font through LovyanGFX's U8g2font decoder:
 *
 *   Font name : u8g2_font_unifont_t_vietnamese2 (Unifont, 16px)
 *   Coverage  : 928 glyphs — ASCII + Latin-1 + complete Vietnamese block
 *               (precomposed à á ả ã ạ, ă ắ ằ ẳ ẵ ặ, â ấ ầ ẩ ẫ ậ, ê ế ề ể ễ ệ,
 *                ô ố ồ ổ ỗ ộ, ơ ớ ờ ở ỡ ợ, ư ứ ừ ử ữ ự, ...)
 *   License   : SIL OFL 1.1 / GPLv2+ with font embedding exception
 *               (Unifont (c) 1998-2024 Roman Czyborra, Paul Hardy, et al.)
 *
 * Data lives in u8g2_font_unifont_t_vietnamese2.c (from the u8g2 project).
 * Usage:
 *   #include "nav_font_vn.h"
 *   ...
 *   spr.setFont(&FontVN);
 *   spr.setCursor(x, y);
 *   spr.print("Lê Lợi");   // raw UTF-8 — decoded automatically
 */
#pragma once

#include <stdint.h>

/* u8g2 font files reference a U8G2_FONT_SECTION macro; not needed here. */
#ifndef U8G2_FONT_SECTION
#define U8G2_FONT_SECTION(name)
#endif

#include "u8g2_font_unifont_t_vietnamese2.c"

#include "lgfx/v1/lgfx_fonts.hpp"

/* FontVN: 16px Unifont with full Vietnamese support. Pass to setFont(&FontVN). */
static const lgfx::U8g2font FontVN((const uint8_t *)u8g2_font_unifont_t_vietnamese2);
