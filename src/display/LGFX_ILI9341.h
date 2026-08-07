/**
 * LovyanGFX board config for the 2.8" IPS ESP32-S3 + ILI9341 (ES3C28P/ES3N28P)
 *
 * SPI2: SCLK=12 MOSI=11 MISO=13 CS=10 DC=46 RST=(none, software reset)
 * Backlight: GPIO45, active HIGH
 * Matches the vendor TFT_eSPI User_Setup (CS=10 DC=46 RST=-1 MOSI=11 SCLK=12
 * MISO=13 BL=45, 40MHz, no color inversion).
 */
#ifndef LGFX_ILI9341_H_
#define LGFX_ILI9341_H_

#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
  lgfx::Touch_FT5x06  _touch;   // FT6336 (FocalTech) on I2C

public:
  LGFX(void)
  {
    { // ---- SPI bus ----
      auto cfg = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = 12;
      cfg.pin_mosi    = 11;
      cfg.pin_miso    = 13;
      cfg.pin_dc      = 46;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // ---- Panel ----
      auto cfg = _panel.config();
      cfg.pin_cs         = 10;
      cfg.pin_rst        = -1;      // no RST wire -> software reset via cmd 0x01
      cfg.panel_width    = 240;
      cfg.panel_height   = 320;
      cfg.offset_x       = 0;
      cfg.offset_y       = 0;
      cfg.offset_rotation = 0;
      cfg.readable       = false;
      cfg.invert         = true;    // IPS panel needs INVON (colours were inverted)
      cfg.rgb_order      = false;   // BGR (matches vendor MADCTL 0x28)
      cfg.dlen_16bit     = false;
      cfg.bus_shared     = false;   // only this display on the SPI bus
      _panel.config(cfg);
    }
    { // ---- Backlight ----
      auto cfg = _light.config();
      cfg.pin_bl      = 45;
      cfg.invert      = false;      // active HIGH
      cfg.freq        = 12000;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    { // ---- Touch (FT6336, I2C: SDA=16 SCL=15 INT=17 RST=18) ----
      auto cfg = _touch.config();
      cfg.x_min    = 0;
      cfg.x_max    = 320;   // native touch range
      cfg.y_min    = 0;
      cfg.y_max    = 240;
      cfg.pin_int  = 17;
      cfg.pin_rst  = 18;
      cfg.pin_sda  = 16;
      cfg.pin_scl  = 15;
      cfg.i2c_port = 0;
      cfg.freq     = 400000;
      _touch.config(cfg);
    }
    setPanel(&_panel);
    _panel.setTouch(&_touch);
  }
};

#endif // LGFX_ILI9341_H_
