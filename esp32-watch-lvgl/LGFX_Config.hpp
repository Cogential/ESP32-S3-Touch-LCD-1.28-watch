// Display driver setup for the Waveshare ESP32-S3-Touch-LCD-1.28
// GC9A01 240x240 round panel over SPI. Touch (CST816S) and the IMU
// (QMI8658) are handled separately over plain Wire I2C — see main .ino —
// so LovyanGFX here only owns the SPI bus / panel / backlight.
#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Light_PWM     _light_instance;

public:
  LGFX(void)
  {
    { // SPI bus
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      // 80MHz: the GC9A01 is rated well past this and Waveshare's own
      // driver for this board runs it here. Raised from 40MHz when the melt
      // transition turned out to be SPI-bound — a full 240x240 RGB565 frame
      // is 115KB, which is 23ms at 40MHz and ~11ms here, and the melt pushes
      // one of those per frame. Every other redraw got cheaper too.
      // Verified on hardware: if the panel ever shows tearing, speckle or
      // shifted pixels, this is the first thing to put back to 40000000.
      cfg.freq_write = 80000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 10;
      cfg.pin_mosi = 11;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 8;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    { // panel
      auto cfg = _panel_instance.config();
      cfg.pin_cs   = 9;
      cfg.pin_rst  = 14;
      cfg.pin_busy = -1;
      cfg.panel_width  = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable   = false;
      cfg.invert     = true;   // GC9A01 needs color inversion on
      cfg.rgb_order  = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }
    { // backlight (PWM dimmable)
      auto cfg = _light_instance.config();
      cfg.pin_bl = 2;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};
