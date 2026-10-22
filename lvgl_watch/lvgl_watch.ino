// lvgl_watch — proof-of-concept: LVGL v9 running on top of the same
// already-verified LovyanGFX panel driver as esp32-watch-original, to see what a
// real widget-toolkit UI looks like on this exact board. Deliberately NOT
// a second full watch app: no WiFi/BLE/steps/sleep here — just enough to
// prove the render + touch pipeline works, and show what LVGL's stock
// widgets (arcs, labels, animation) look like next to esp32-watch-original's
// hand-drawn screens. See the repo README's "Exploring a richer UI"
// section for the reasoning behind this approach (why LovyanGFX stays the
// low-level driver instead of switching to TFT_eSPI, why v9 over v8.3).
//
// Panel is left at its native, unrotated orientation on purpose: the touch
// controller (Touch_CST816S.h, copied unchanged from esp32-watch-original/) reports
// raw, unrotated panel coordinates, and LVGL needs those to line up exactly
// with what's drawn for hit-testing to work at all. esp32-watch-original never had
// to solve that: it only reads swipe deltas and tap/hold duration, never an
// absolute tap position, so its display.setRotation(1) never needed a
// matching touch remap. Getting rotation working here (to match
// esp32-watch-original's upright orientation) needs that remap worked out — not
// attempted in this proof-of-concept.

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>

#include "LGFX_Config.hpp"
#include "Touch_CST816S.h"

static const uint8_t PIN_I2C_SDA = 6;
static const uint8_t PIN_I2C_SCL = 7;
static const uint8_t PIN_TOUCH_RST = 13;
static const uint8_t PIN_TOUCH_INT = 5;

LGFX display;
TouchCST816S touch;
bool touchOk = false;

lv_display_t *lvDisplay = nullptr;
lv_obj_t *clockLabel = nullptr;
lv_obj_t *ring = nullptr;

// Push whatever LVGL rendered for `area` straight to the panel — synchronous,
// no DMA hand-off. Simplest thing that works; esp32-watch-original's own reasoning
// for drawing straight to the panel (easier to trust, easier to debug)
// applies doubly for a first proof-of-concept.
void lvglFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  display.startWrite();
  display.setAddrWindow(area->x1, area->y1, w, h);
  display.writePixels(reinterpret_cast<lgfx::rgb565_t *>(px_map), w * h);
  display.endWrite();
  lv_display_flush_ready(disp);
}

void lvglTouchRead(lv_indev_t *indev, lv_indev_data_t *data) {
  int16_t x, y;
  if (touchOk && touch.read(x, y)) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// Tap toggles the ring's colour — proves a touch is actually hitting the
// widget under your finger, not just registering *a* touch somewhere on
// the panel (which a coordinate mismatch could still do and look "working"
// at a glance).
void onRingTapped(lv_event_t *e) {
  static bool alt = false;
  alt = !alt;
  lv_obj_set_style_arc_color(ring, alt ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_CYAN),
                              LV_PART_INDICATOR);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[lvgl_watch] boot");

  display.init();
  display.setBrightness(180);
  // Deliberately NOT display.setRotation(1) — see the file header comment.

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  for (int attempt = 1; attempt <= 4 && !touchOk; attempt++) {
    touchOk = touch.begin(Wire, PIN_TOUCH_RST, PIN_TOUCH_INT);
    if (!touchOk) delay(150);
  }
  Serial.printf("[lvgl_watch] touch init -> %d\n", touchOk);

  lv_init();
  lv_tick_set_cb(millis);

  lvDisplay = lv_display_create(240, 240);
  lv_display_set_color_format(lvDisplay, LV_COLOR_FORMAT_RGB565);
  // 40 rows at a time (240 * 40 * 2 bytes = ~19KB per buffer) — plenty of
  // headroom below the ESP32-S3's internal RAM without reaching for PSRAM.
  static uint8_t buf1[240 * 40 * 2];
  static uint8_t buf2[240 * 40 * 2];
  lv_display_set_buffers(lvDisplay, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(lvDisplay, lvglFlush);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvglTouchRead);

  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

  // Decorative progress ring — demonstrates LVGL's anti-aliased arc
  // rendering, which esp32-watch-original's hand-drawn circles (see
  // drawWeatherIcon()/drawBigTempAt() in esp32-watch-original.ino) can't match
  // without a lot more hand-written math.
  ring = lv_arc_create(screen);
  lv_obj_set_size(ring, 220, 220);
  lv_obj_center(ring);
  lv_arc_set_bg_angles(ring, 0, 360);
  lv_arc_set_value(ring, 70);
  lv_obj_set_style_arc_color(ring, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(ring, 8, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(ring, 8, LV_PART_MAIN);
  lv_obj_add_event_cb(ring, onRingTapped, LV_EVENT_CLICKED, NULL);

  // Big clock label — Montserrat 48, proportional and anti-aliased, next to
  // esp32-watch-original's 7-segment Font7 digital-clock look. Not wired to real
  // time (no WiFi/NTP here) — just ticks up from millis() to prove live
  // text updates render cleanly.
  clockLabel = lv_label_create(screen);
  lv_obj_set_style_text_font(clockLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(clockLabel, lv_color_white(), 0);
  lv_label_set_text(clockLabel, "00:00");
  lv_obj_center(clockLabel);

  lv_obj_t *hint = lv_label_create(screen);
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_label_set_text(hint, "tap the ring");
  lv_obj_align(hint, LV_ALIGN_CENTER, 0, 60);

  Serial.println("[lvgl_watch] LVGL up.");
}

void loop() {
  lv_timer_handler();

  // Fake running clock, just to prove label updates render live — this
  // sketch has no WiFi/NTP/BLE time source of its own. A real integration
  // would reuse esp32-watch-original's own time-sync code rather than reinvent it.
  static uint32_t lastClockMs = 0;
  uint32_t now = millis();
  if (now - lastClockMs >= 1000) {
    lastClockMs = now;
    uint32_t s = now / 1000;
    char buf[8];
    snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
    lv_label_set_text(clockLabel, buf);
  }

  delay(5);
}
