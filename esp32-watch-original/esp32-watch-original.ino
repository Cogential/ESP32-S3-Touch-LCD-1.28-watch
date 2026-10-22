// ESP32-WATCH watch face for the Waveshare ESP32-S3-Touch-LCD-1.28
//
// - Round GC9A01 display, green-phosphor CRT look, four screens (tap to
//   cycle): STAT (clock + weather one-liner), DATA (steps), WEATHER (full
//   temp/hi-lo/condition), NOTIF (latest phone notification).
// - Step counting from the onboard QMI8658 accelerometer. The gyro
//   (GYRO_ENABLED) is on too, feeding raise-to-wake: turning the wrist to
//   look at the watch un-dims the screen without a touch, the same way a
//   real smartwatch does. See updateRaiseToWake() — its thresholds are
//   reasoned guesses pending calibration against real hardware, the same
//   situation the swipe-axis knobs below started in.
// - No physical power switch on this board — long-press-and-hold (3s,
//   finger still) on the DATA screen puts it into a real ESP32 deep sleep
//   instead, waking back up on the next touch. See enterDeepSleep() and the
//   wake-from-sleep fast path near the top of setup().
// - Time comes from WiFi + NTP on boot (this board has no RTC chip), then
//   WiFi is switched off to save power. Re-syncs periodically in case the
//   software clock drifts. No SSID/password is hardcoded: WiFiManager opens
//   an "esp32-watch-setup" WiFi hotspot + captive portal to pick a network the
//   first time (or whenever) it can't connect — long-press-and-hold on the
//   STAT screen reopens it on demand, e.g. to switch onto a phone's hotspot.
// - BLE peripheral "ESP32-WATCH": bondable/pairable from Android's
//   Bluetooth settings, and speaks the open "Bangle.js" Nordic-UART JSON
//   protocol that Gadgetbridge (free, open-source Android app) already
//   knows how to talk — add this device in Gadgetbridge as a Bangle.js
//   watch to get notification + weather forwarding with no custom app.
//
// Board pinout reference (Waveshare ESP32-S3-Touch-LCD-1.28):
//   LCD  SPI : SCLK10 MOSI11 MISO12 CS9 DC8 RST14  BL2
//   I2C shared (touch + IMU): SDA6 SCL7
//   Touch CST816S : INT5 RST13  addr 0x15
//   IMU   QMI8658  : INT1 4 / INT2 3   addr 0x6A/0x6B
//   Battery ADC: GPIO1 (200k/100k divider -> multiply by 3)
//
// Drawing goes straight to the panel (no offscreen sprite) so there's
// nothing PSRAM-allocation-dependent between "display.init() succeeded"
// and "something is on screen" — easier to trust, easier to debug.

#include <WiFi.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <QMI8658.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLESecurity.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_sleep.h>
#include <esp_pm.h>

#include "LGFX_Config.hpp"
#include "Touch_CST816S.h"
#include "StepCounter.h"
#include "vaultboy_bitmap.h"

// ---------- WiFi / time ----------
// No hardcoded SSID/password anymore — WiFiManager owns WiFi credentials
// (stored in its own NVS blob, separate from Preferences). First boot with
// nothing saved (or a failed connect) pops an "esp32-watch-setup" WiFi AP + captive
// portal to pick a network, same flow as most consumer IoT gadgets. A
// long-press on the STAT screen re-opens that portal on demand, e.g. to
// switch onto your phone's hotspot. See wifiConnectOrSetup() /
// forceWifiSetupPortal() below.
static const char *WIFI_SETUP_AP_NAME = "esp32-watch-setup";

// ---------- long-press gestures ----------
// This board has no physical power switch, and the long-press-and-hold
// gesture is shared by two different actions depending which screen it's
// done on — see the long-press handling in loop().
// Which action fires is chosen by how long the hold lasted, not by which
// screen you were on — screen-dependent actions proved unpredictable, since
// waking from deep sleep reboots and lands back on STAT.
static const uint32_t TAP_MAX_MS = 600;          // a press shorter than this, held still, counts as a tap
static const uint32_t STEPS_RESET_ARM_MS = 4000; // gap allowed between taps before the count lapses
static const uint8_t STEPS_RESET_TAPS = 4;       // deliberate taps needed to wipe the day's steps
static const uint32_t LONG_PRESS_MS = 3000;       // hold at least this long, finger still, then release
static const uint32_t SLEEP_PRESS_MAX_MS = 6000;  // 3-6s puts it to sleep; beyond that is the portal
static const uint32_t LONG_PRESS_MAX_MS = 12000;  // ...and past this nothing fires, so something resting on
                                                  // the screen and later removed doesn't count as a press
// POSIX TZ string for America/Toronto — handles EST/EDT DST automatically.
static const char *POSIX_TZ = "EST5EDT,M3.2.0,M11.1.0/2";
// Tracked in seconds of real wall-clock time (not millis()) because millis()
// resets to 0 on every deep-sleep wake — see rtcLastSyncAttemptEpoch below.
static const time_t WIFI_RESYNC_INTERVAL_SEC = 4 * 60 * 60; // 4h
// Deep sleep tracks elapsed time using the RTC's internal RC oscillator
// (esp_timer/the main crystal aren't powered), which is meaningfully less
// accurate than the crystal-derived clock used while continuously awake —
// drift on the order of a few percent of however long the nap lasted. So a
// sleep-wake earns its own, much shorter, resync threshold rather than
// sharing WIFI_RESYNC_INTERVAL_SEC with the continuously-awake case: a wake
// after only 30-60 minutes asleep can already be visibly off, well before
// 4h of *awake* drift would be.
static const time_t WAKE_RESYNC_INTERVAL_SEC = 30 * 60; // 30m

// ---------- power saving ----------
static const uint8_t BRIGHT_ACTIVE = 180;
static const uint8_t BRIGHT_DIM = 12;
static const uint32_t IDLE_DIM_TIMEOUT_MS = 8000; // dim backlight after 8s of no touch
// Deep sleep after this long untouched. Note what this costs: deep sleep drops
// BLE, so notifications stop arriving until the watch is woken. That is the
// point of the trade — an idle watch that is genuinely asleep rather than
// merely dim — but it is a real consequence, not a free one. Setting it to 0
// disables the automatic sleep and leaves the long-press as the only route in.
static const uint32_t AUTO_SLEEP_TIMEOUT_MS = 10UL * 60 * 1000;
// Gyros draw meaningfully more current than accel-only mode on this class of
// IMU — not worth paying for while nothing reads it. On (costs the extra
// current) because raise-to-wake, below, is the thing that reads it now.
static const bool GYRO_ENABLED = true;
// Periodic accel/gyro logging, used to calibrate raise-to-wake against the
// board's real axis convention rather than an assumed one. Off now that the
// thresholds below are calibrated; flip on to re-measure.
static const bool IMU_TRACE = false;

// ---------- raise-to-wake (gyro) ----------
// Calibrated from logged wrist raises on the real board, not from
// assumptions — two earlier attempts got this wrong. The first guessed the
// screen normal was +Z and never fired at all. The second read a resting
// sample of (-1.6, 9.8, 1.9) and concluded +Y, but the board was propped on
// edge for that, not face up. Resting genuinely face up it reads
// a = (-1.1, 2.5, -9.5) m/s^2, so the screen normal is -Z.
static const float RAISE_GYRO_THRESHOLD_DPS = 80.0f; // rotation speed that counts as an active wrist turn, not ambient jitter
static const uint32_t RAISE_GYRO_MIN_MS = 50;        // must sustain that rotation this long — filters brief knocks
static const uint32_t RAISE_GYRO_MAX_MS = 1500;      // ...but a raise is a flick, not a slow drift — cap it too
// Gravity vector (m/s^2) in the viewing position, taken from samples logged
// while the watch was actually being held up and looked at:
// a ~ (0, -7.2, -5.5), scaled to magnitude ~9.8 to match the normalisation
// in updateRaiseToWake().
//
// Note the sign on Y. An earlier pass targeted (0, +4.9, -8.5) and got this
// exactly backwards — that is the orientation the watch rests in on a desk,
// so it woke on being *put down* and stayed dark while raised. The two are
// only distinguishable given the report of which motion produced the wake;
// the log alone cannot say which samples were which.
static const float RAISE_ACCEL_TARGET_X = 0.0f;
static const float RAISE_ACCEL_TARGET_Y = -7.8f;
static const float RAISE_ACCEL_TARGET_Z = -6.0f;
// Cosine similarity to that target. Measured: raised-and-viewed samples
// score ~0.99-1.00 against it, while resting-on-a-desk samples score
// 0.00-0.38. 0.80 sits well clear of both.
static const float RAISE_ACCEL_DOT_MIN = 0.80f;
static const uint32_t RAISE_COOLDOWN_MS = 1500; // minimum gap between two raise-to-wake triggers

// ---------- swipe navigation ----------
// The display is rotated 90° in software (see setRotation(1) in setup()),
// but the touch chip reports raw, unrotated panel coordinates — so the
// raw axis that corresponds to "visual left/right" is the *other* raw
// axis from what you'd naively expect, and possibly flipped in sign.
// These two knobs are the one-line fix if a swipe feels backwards or
// diagonal once tested on the real hardware:
static const bool SWIPE_AXIS_IS_RAW_Y = true;  // true: raw Y delta = visual horizontal swipe
static const bool SWIPE_INVERT = false;        // flip if left/right come out reversed
// Direction of the perpendicular (notification-scrolling) swipe. Same kind
// of one-line knob as SWIPE_INVERT above: flip if scrolling feels backwards.
static const bool VSWIPE_INVERT = false;
static const int16_t SWIPE_MIN_DIST = 35;      // px of raw movement to count as a swipe, not a tap

// ---------- pins ----------
static const uint8_t PIN_I2C_SDA   = 6;
static const uint8_t PIN_I2C_SCL   = 7;
// Touch reset is GPIO13 and the IMU's INT1 is GPIO4 — these were swapped
// here until now, meaning every "touch reset" actually toggled the IMU's
// interrupt line and the touch controller was never reset at all. It went
// unnoticed because the CST816S comes up fine on its own from a cold power
// cycle; the cost only shows up when it needs recovering from a bad state,
// which is exactly when the reset matters. Confirmed against both the
// Waveshare schematic netlist (TP_RST -> GPIO13) and Waveshare's published
// pinout.
static const uint8_t PIN_TOUCH_RST = 13;
static const uint8_t PIN_TOUCH_INT = 5;
static const uint8_t PIN_IMU_INT   = 4;
static const uint8_t PIN_BATT_ADC  = 1;

// ---------- esp32-watch palette ----------
static const uint16_t COL_BG        = lgfx::color565(0, 0, 0);
static const uint16_t COL_GREEN     = lgfx::color565(85, 255, 110);   // bright phosphor
// Reserved for icon outlines and state (an unconnected Bluetooth rune, the
// battery shell) and for the boot log's scrollback fade. Text no longer
// uses it: at this size, on this panel, dim secondary text was markedly
// harder to read than the clock sitting next to it.
static const uint16_t COL_GREEN_DIM = lgfx::color565(25, 95, 40);
static const uint16_t COL_AMBER     = lgfx::color565(255, 170, 60);   // fault/status accent

// Per-screen accents. Each section owns a colour so you can tell at a glance
// which one you're on. All are kept at high luminance against the black
// background — on this panel a dark colour reads as unlit rather than as
// "darker", which is the same legibility problem the dim green had.
static const uint16_t COL_YELLOW    = lgfx::color565(255, 220, 70);   // weather
static const uint16_t COL_BLUE      = lgfx::color565(90, 175, 255);   // notifications
static const uint16_t COL_RED       = lgfx::color565(255, 85, 75);    // stopwatch
static const uint16_t COL_PURPLE    = lgfx::color565(175, 110, 255);  // timer
static const uint16_t COL_ORANGE    = lgfx::color565(255, 150, 50);   // steps (and step history)
static const uint16_t COL_SCANLINE  = lgfx::color565(0, 30, 10);      // faint CRT texture
// Flashlight only. Deliberately off-theme: every other screen stays
// monochrome green-phosphor, but the point of a flashlight is raw lumens to
// see by, and white puts more light out of this panel than green alone.
static const uint16_t COL_WHITE     = lgfx::color565(255, 255, 255);

// ---------- globals ----------
LGFX display;
TouchCST816S touch;
QMI8658 imu;
StepCounter stepCounter;
Preferences prefs;

bool touchOk = false;
bool imuOk = false;

volatile bool bleConnected = false;
volatile bool timeSynced = false;
uint8_t currentScreen = 0; // see the SCREEN_* slots below
const uint8_t SCREEN_STAT = 0;       // clock, date, weather line, notification banner
const uint8_t SCREEN_DATA = 1;       // step count
const uint8_t SCREEN_HISTORY = 2;    // 7-day step history bar chart — grouped next to DATA
const uint8_t SCREEN_WEATHER = 3;    // full weather
const uint8_t SCREEN_NOTIF = 4;      // notification list
const uint8_t SCREEN_MUSIC = 5;      // now-playing, from Gadgetbridge — grouped next to NOTIF
const uint8_t SCREEN_STOPWATCH = 6;
const uint8_t SCREEN_TIMER = 7;
const uint8_t SCREEN_FLASHLIGHT = 8; // max-brightness solid fill; last in the cycle, out of the way
const uint8_t NUM_SCREENS = 9;
// Nothing persists currentScreen across a boot (it always starts back at
// SCREEN_STAT — see wokeFromSleep in setup()), so renumbering these between
// firmware versions is safe: no stored value anywhere depends on the
// specific numbers.

// ---------- stopwatch ----------
// Both the stopwatch and the timer keep running while you're on other
// screens; they're only *displayed* by their own screens. Elapsed time is
// derived from millis() rather than counted up in the loop, so it can't
// drift with however often the loop happens to run.
bool swRunning = false;
uint32_t swStartMs = 0;   // millis() at the last start
uint32_t swAccumMs = 0;   // time banked by previous start/stop cycles

// ---------- countdown timer ----------
// Presets rather than a digit-entry UI — there's one finger and a 240px
// circle to work with, and picking from a short list is far quicker than
// setting a duration a digit at a time.
const uint32_t TIMER_PRESETS_SEC[] = {60, 180, 300, 600, 900, 1800, 3600};
const uint8_t TIMER_PRESET_COUNT = sizeof(TIMER_PRESETS_SEC) / sizeof(TIMER_PRESETS_SEC[0]);
uint8_t timerPresetIdx = 2; // 5 minutes
bool timerRunning = false;
uint32_t timerEndMs = 0;      // millis() the countdown reaches zero at
uint32_t timerRemainMs = 0;   // frozen remainder while paused
bool timerFired = false;      // expired and still asking to be acknowledged
uint32_t timerFiredMs = 0;
bool prevTouchDown = false;   // debounced/stable state used for edge detection
uint8_t touchDownStreak = 0;  // consecutive raw "down" reads in a row
bool wakeTouchPending = false; // a touch woke the dimmed screen; suppress navigation until it lifts
uint32_t lastTouchPollMs = 0;
int16_t touchStartX = 0, touchStartY = 0; // where the current gesture began
int16_t touchLastX = 0, touchLastY = 0;   // most recent sample of the current gesture
bool touchWasDimWake = false; // this gesture's first touch woke a dimmed screen — don't also navigate
uint32_t touchDownStartMs = 0; // when the current gesture's stable "down" began, for long-press timing
bool forceRedraw = true;
uint32_t notifAutoRevertMs = 0; // when a popped-up notification should hide itself
bool notifAutoShown = false;

// ---------- notification + weather state (from Gadgetbridge) ----------
// Notifications are kept as a list, newest first, and only leave it when the
// phone says they were dismissed — matching what you see in the phone's own
// shade rather than the watch quietly forgetting them. The NOTIF screen
// scrolls through the list; the STAT banner always shows slot 0.
struct Notif {
  long id;        // Gadgetbridge's notification id, used to match dismissals
  String src;     // app name, e.g. "Signal"
  String title;   // usually the sender
  String body;    // message text
};
static const uint8_t MAX_NOTIFS = 8; // older ones fall off the end
Notif notifs[MAX_NOTIFS];
uint8_t notifCount = 0;
uint8_t notifIndex = 0; // which one the NOTIF screen is currently showing

bool hasWeather = false;
int weatherTempC = 0;
int weatherHiC = 0;
int weatherLoC = 0;
String weatherTxt = "";
// Declared up here rather than beside the icon drawing code: the Arduino
// build auto-generates function prototypes at the top of the sketch, so a
// type used in any signature must exist before them.
enum WeatherIcon { WI_SUN, WI_PARTLY, WI_CLOUD, WI_RAIN, WI_SNOW, WI_STORM, WI_FOG };

// OpenWeatherMap condition code, carried through by Gadgetbridge (2xx
// thunderstorm, 3xx drizzle, 5xx rain, 6xx snow, 7xx fog/haze, 800 clear,
// 801-802 partly cloudy, 803-804 cloudy). 0 = not supplied, in which case
// the icon falls back to keyword-matching weatherTxt.
int weatherCode = 0;
// Wall-clock epoch the current reading arrived at, or 0 if it arrived while
// the clock was still unset. Persisted with the reading so a reboot can tell
// how old the restored weather is instead of presenting it as current —
// Gadgetbridge only pushes on its own schedule, so without this the watch
// shows "NO DATA" after every boot until the phone next feels like sending.
time_t weatherEpoch = 0;
// Past this, a restored reading is too old to be worth showing at all.
static const time_t WEATHER_MAX_AGE_SEC = 12 * 60 * 60;
// Below this, it's current enough not to bother captioning with an age.
static const time_t WEATHER_FRESH_SEC = 15 * 60;

// ---------- step history ----------
// The past 6 *completed* days' final step totals, oldest first — index 5 is
// the most recently finished day. "Today" is deliberately not stored here;
// it's read live from stepCounter so the HISTORY screen's rightmost bar
// updates as you walk instead of only appearing at day-rollover. Shifted
// and persisted once/day — see the day-rollover block in loop() and
// saveState().
uint32_t stepHist[6] = {0, 0, 0, 0, 0, 0};

// ---------- now playing (from Gadgetbridge) ----------
// Populated from the Bangle.js "musicinfo"/"musicstate" messages Gadgetbridge
// already sends over the same NUS channel as notify/weather — see
// handleGadgetbridgeLine(). Not persisted: this is "what's playing right
// now", stale the moment the phone disconnects, so there's nothing worth
// restoring after a reboot.
bool hasMusic = false;
String musicArtist = "";
String musicTrack = "";
bool musicPlaying = false;

// Latest gyro reading (deg/sec), updated every IMU poll alongside the accel
// feed into StepCounter. Not consumed by anything yet — available for
// whatever reads it next.
float gyroX = 0, gyroY = 0, gyroZ = 0;

uint32_t lastImuPollMs = 0;
uint32_t lastImuTraceMs = 0;
uint32_t lastFastDrawMs = 0;
uint32_t stepsResetArmedMs = 0; // time of the most recent reset tap, 0 when not counting
uint8_t stepsResetTaps = 0;     // taps counted so far toward STEPS_RESET_TAPS
uint32_t lastSaveMs = 0;
// RTC_DATA_ATTR survives deep sleep (backed by RTC slow memory, unlike
// regular globals which reset every wake) but not a full power cycle — that
// initializes it back to 0, which is fine, since a fresh boot always makes
// its own sync attempt regardless. Tracks the last *attempt*, not last
// *success*, so a stretch with no WiFi in range retries once per interval
// rather than either spamming every wake or never retrying again. See
// wifiResyncDue().
RTC_DATA_ATTR time_t rtcLastSyncAttemptEpoch = 0;
uint32_t lastActivityMs = 0;
bool backlightDimmed = false;
uint32_t lastDayMarker = 0; // epoch-day steps were last reset on
uint32_t nextFlickerMs = 0; // when the next CRT power-flicker blip is due — see loop()
bool wasFlashlightScreen = false; // tracks entering/leaving SCREEN_FLASHLIGHT — see loop()
int lastRenderedSecond = -1;

// ---------- raise-to-wake state ----------
bool raiseGyroActive = false;   // currently inside a rotation burst above RAISE_GYRO_THRESHOLD_DPS
uint32_t raiseGyroStartMs = 0;  // when the current burst began
float raiseGyroPeak = 0;        // fastest rotation seen during it, for calibration logging
uint32_t lastRaiseWakeMs = 0;   // last time raise-to-wake actually fired, for RAISE_COOLDOWN_MS

// ---------- BLE ----------
// Custom "esp32-watch Time Service". The companion web page writes 4 bytes
// (little-endian uint32) of "local wall-clock epoch seconds" — i.e. it
// already folds in the phone's timezone offset, so the watch can just
// treat the value as the time to display, no timezone math needed here.
static const char *SERVICE_UUID     = "7a2b0001-6b1d-4a6f-9c2e-6f2f6f1b0a01";
static const char *TIME_CHAR_UUID   = "7a2b0002-6b1d-4a6f-9c2e-6f2f6f1b0a01";
static const char *STEPS_CHAR_UUID  = "7a2b0003-6b1d-4a6f-9c2e-6f2f6f1b0a01";

// Nordic UART Service — the standard BLE "serial port" profile. Gadgetbridge
// talks this to any device it treats as a "Bangle.js" watch: it writes
// newline-terminated `GB({...})` JSON lines to RX, we could reply on TX
// (unused for now — we only need phone -> watch).
static const char *NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLEServer *bleServer = nullptr;
BLECharacteristic *stepsChar = nullptr;
BLEAdvertising *bleAdvertising = nullptr;
String nusRxBuffer = "";
// TX is now also used for music control (see sendMusicControl()), so unlike
// the other characteristics it needs to outlive setupBLE()'s local scope.
BLECharacteristic *nusTxChar = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    forceRedraw = true;
    Serial.println("[esp32-watch] BLE client connected.");
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    forceRedraw = true;
    Serial.println("[esp32-watch] BLE client disconnected, re-advertising.");
    bleAdvertising->start(); // keep advertising so the phone can reconnect
  }
};

class TimeWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    if (v.length() != 4) return;
    uint32_t epoch = (uint8_t)v[0] | ((uint8_t)v[1] << 8) | ((uint8_t)v[2] << 16) | ((uint32_t)(uint8_t)v[3] << 24);
    struct timeval tv = { (time_t)epoch, 0 };
    settimeofday(&tv, nullptr);
    timeSynced = true;
    forceRedraw = true;
  }
};

// ---------- notification list ----------
// Adds a notification at slot 0 (newest first). An id already in the list
// updates in place rather than duplicating — Gadgetbridge re-sends the same
// id when a notification is updated on the phone, e.g. a chat thread
// gaining another message.
void notifAdd(long id, const String &src, const String &title, const String &body) {
  for (uint8_t i = 0; i < notifCount; i++) {
    if (id != 0 && notifs[i].id == id) {
      notifs[i].src = src;
      notifs[i].title = title;
      notifs[i].body = body;
      // Move it to the front so "newest first" still holds after an update.
      for (uint8_t j = i; j > 0; j--) {
        Notif tmp = notifs[j - 1];
        notifs[j - 1] = notifs[j];
        notifs[j] = tmp;
      }
      return;
    }
  }

  if (notifCount < MAX_NOTIFS) notifCount++;
  for (uint8_t j = notifCount - 1; j > 0; j--) notifs[j] = notifs[j - 1]; // shift down, dropping the oldest
  notifs[0].id = id;
  notifs[0].src = src;
  notifs[0].title = title;
  notifs[0].body = body;
}

// Removes by id. A dismissal without a usable id falls back to dropping the
// newest, which is the best guess available and still better than ignoring
// the message and letting the list grow stale.
void notifRemove(long id) {
  if (notifCount == 0) return;
  uint8_t target = 0;
  bool found = (id == 0);
  if (id != 0) {
    for (uint8_t i = 0; i < notifCount; i++) {
      if (notifs[i].id == id) { target = i; found = true; break; }
    }
  }
  if (!found) return; // an id we never held — nothing to do

  for (uint8_t i = target; i + 1 < notifCount; i++) notifs[i] = notifs[i + 1];
  notifCount--;
  notifs[notifCount] = Notif(); // release the String memory of the vacated slot
  if (notifIndex >= notifCount) notifIndex = notifCount ? notifCount - 1 : 0;
}

// Handles one complete line already extracted from the RX stream. Most
// messages are `GB({...})` JSON, but time sync is a special case:
// Gadgetbridge sends it as a raw Espruino JS command instead —
//   \x10setTime(1735000000);E.setTimeZone(-5);...
// — since real Bangle.js devices run a JS REPL. The number is real UTC
// epoch seconds; we ignore Gadgetbridge's crude whole-hour timezone
// offset and use our own DST-aware POSIX TZ rule (set at boot) instead.
// Gadgetbridge pushes this automatically on every BLE connect, which is
// exactly what gets the clock right when away from WiFi.
void handleGadgetbridgeLine(const String &line) {
  // Log every line that arrives, so "the phone isn't sending anything" is
  // distinguishable from "it's sending something we fail to parse".
  Serial.printf("[esp32-watch] GB rx: %.120s\n", line.c_str());

  int stIdx = line.indexOf("setTime(");
  if (stIdx >= 0) {
    int numStart = stIdx + 8; // strlen("setTime(")
    int numEnd = line.indexOf(')', numStart);
    if (numEnd > numStart) {
      time_t epoch = (time_t)line.substring(numStart, numEnd).toInt();
      if (epoch > 1700000000) { // sanity check: looks like a real date
        struct timeval tv = { epoch, 0 };
        settimeofday(&tv, nullptr);
        timeSynced = true;
        forceRedraw = true;
        Serial.printf("[esp32-watch] time synced via BLE (Gadgetbridge): %ld\n", (long)epoch);
      }
    }
    return;
  }

  int start = line.indexOf('{');
  int end = line.lastIndexOf('}');
  if (start < 0 || end <= start) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line.substring(start, end + 1));
  if (err) {
    Serial.printf("[esp32-watch] GB JSON parse failed: %s\n", err.c_str());
    return;
  }

  const char *type = doc["t"] | "";

  if (strcmp(type, "notify") == 0) {
    long id = doc["id"] | 0L;
    notifAdd(id,
             String((const char *)(doc["src"] | "")),
             String((const char *)(doc["title"] | "")),
             String((const char *)(doc["body"] | "")));
    Serial.printf("[esp32-watch] notify id=%ld: %s / %s (%u held)\n",
                  id, notifs[0].src.c_str(), notifs[0].title.c_str(), notifCount);

    // Pop the notification screen up for a few seconds, like a real watch,
    // showing the one that just arrived rather than wherever the list was
    // last scrolled to.
    notifIndex = 0;
    currentScreen = SCREEN_NOTIF;
    notifAutoShown = true;
    notifAutoRevertMs = millis() + 8000;
    wakeScreen(millis());

  } else if (strcmp(type, "notify-") == 0) {
    // Dismissed on the phone. Matching on id is what keeps the watch's list
    // in step with the phone's shade — dismissing one message must not take
    // the others with it.
    long id = doc["id"] | 0L;
    notifRemove(id);
    forceRedraw = true;
    Serial.printf("[esp32-watch] notification dismissed on phone: id=%ld (%u left)\n", id, notifCount);

  } else if (strcmp(type, "weather") == 0) {
    // Gadgetbridge's Bangle.js weather payload is
    //   {t:"weather", temp, hi, lo, hum, rain, uv, code, txt, wind, wdir, loc}
    // with temperatures in Kelvin (it carries OpenWeatherMap-style values
    // through unchanged). toC() tolerates a provider that sends Celsius
    // instead: no real ambient temperature reaches 200, so a value above
    // that is unambiguously Kelvin. Cheap insurance against a screen full
    // of numbers like "-273C" if some future provider differs.
    auto toC = [](float v) { return (int)roundf(v > 200.0f ? v - 273.15f : v); };
    float rawTemp = doc["temp"] | 0.0f;
    weatherTempC = toC(rawTemp);
    weatherHiC = toC(doc["hi"] | rawTemp);
    weatherLoC = toC(doc["lo"] | rawTemp);
    weatherTxt = String((const char *)(doc["txt"] | ""));
    weatherCode = doc["code"] | 0;
    hasWeather = true;
    forceRedraw = true;
    // Stamp with wall-clock time when it's trustworthy; 0 means "arrived,
    // but we can't date it", which the age display treats as unknown
    // rather than as brand new.
    time_t nowEpoch = time(nullptr);
    weatherEpoch = (nowEpoch > 1700000000) ? nowEpoch : 0;
    saveWeather();
    Serial.printf("[esp32-watch] weather: %dC (%s)\n", weatherTempC, weatherTxt.c_str());

  } else if (strcmp(type, "musicinfo") == 0) {
    // Bangle.js "musicinfo": {t:"musicinfo", artist, album, track, dur, c, n}
    // — sent once whenever the track changes, not on every poll, so this is
    // deliberately sticky rather than re-derived each render.
    musicArtist = String((const char *)(doc["artist"] | ""));
    musicTrack = String((const char *)(doc["track"] | ""));
    hasMusic = true;
    forceRedraw = true;
    Serial.printf("[esp32-watch] musicinfo: %s - %s\n", musicArtist.c_str(), musicTrack.c_str());

  } else if (strcmp(type, "musicstate") == 0) {
    // Bangle.js "musicstate": {t:"musicstate", state, position, shuffle, repeat}
    // — state is "play" or "pause". Sent on its own, separately from
    // musicinfo, so a track can arrive before we're told it's actually
    // playing (or vice versa) — hasMusic is set by either, independently.
    const char *state = doc["state"] | "";
    musicPlaying = (strcmp(state, "play") == 0);
    hasMusic = true;
    forceRedraw = true;
    Serial.printf("[esp32-watch] musicstate: %s\n", state);
  }
  // Other message types (notify-, findPhone, etc.) are ignored for now.
}

class NUSRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String chunk = c->getValue();
    nusRxBuffer += chunk;

    int nl;
    while ((nl = nusRxBuffer.indexOf('\n')) >= 0) {
      String line = nusRxBuffer.substring(0, nl);
      nusRxBuffer = nusRxBuffer.substring(nl + 1);
      handleGadgetbridgeLine(line);
    }
    // Guard against a runaway buffer if we somehow never see a newline.
    if (nusRxBuffer.length() > 2048) nusRxBuffer = "";
  }
};

void setupBLE() {
  BLEDevice::init("Bangle.js esp32-watch"); // name must match Gadgetbridge's device-name allowlist

  // Default BLE TX power is +3dBm — plenty of range for a device that's
  // worn on the wrist next to a phone in the same pocket/room. 0dBm still
  // covers well beyond that with margin, for meaningfully less radio
  // current on every advertising/connection event.
  BLEDevice::setPower(ESP_PWR_LVL_N0);

  // Bondable/pairable with no PIN prompt ("Just Works") — lets the watch
  // show up and pair normally from Android's Bluetooth settings.
  BLESecurity *security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);
  security->setCapability(ESP_IO_CAP_NONE);
  security->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  security->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = bleServer->createService(SERVICE_UUID);

  BLECharacteristic *timeChar = svc->createCharacteristic(
      TIME_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  timeChar->setCallbacks(new TimeWriteCallbacks());

  stepsChar = svc->createCharacteristic(
      STEPS_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  stepsChar->setValue("0");

  svc->start();

  // Nordic UART Service — this is what Gadgetbridge actually talks to.
  BLEService *nus = bleServer->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rxChar = nus->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new NUSRxCallbacks());
  // Kept as a global now (nusTxChar) instead of a local — sendMusicControl(),
  // called from loop() on a MUSIC-screen tap, needs to reach it too.
  nusTxChar = nus->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  nusTxChar->addDescriptor(new BLE2902());
  nus->start();

  bleAdvertising = BLEDevice::getAdvertising();
  bleAdvertising->addServiceUUID(SERVICE_UUID);
  bleAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  bleAdvertising->setScanResponse(true);
  // Default advertising interval is a fast 20-40ms — the radio keys up
  // 25-50x/second the whole time the watch is unconnected. Slowed to
  // 100-200ms: still reconnects well under half a second (imperceptible
  // for a background BLE reconnect), for a fraction of the radio-on time
  // during any stretch spent disconnected (phone out of range, Bluetooth
  // off, etc.).
  bleAdvertising->setMinInterval(160); // 160 * 0.625ms = 100ms
  bleAdvertising->setMaxInterval(320); // 320 * 0.625ms = 200ms
  bleAdvertising->start();
}

// Sends a media-control command back to the phone, for the MUSIC screen's
// tap-to-play/pause. cmd is one of "play", "pause", "next", "previous".
//
// UNVERIFIED against a real Gadgetbridge session — everything else this
// file sends or parses (setTime, notify/notify-, weather) was confirmed
// against an actual phone; this one line wasn't, because doing so needs a
// phone with Gadgetbridge actually paired and playing music, not just this
// board on a bench. It matches the call real Bangle.js firmware makes when
// its own music-control widget is tapped, which is what Gadgetbridge's
// Bangle.js support is documented to watch for on this same TX
// characteristic — but "documented" isn't "tested here". If tapping the
// MUSIC screen does nothing to the phone's playback, this line is the first
// thing to check with a serial log and a packet capture.
void sendMusicControl(const char *cmd) {
  if (!bleConnected || !nusTxChar) return;
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "Bangle.musicControl(\"%s\")\n", cmd);
  nusTxChar->setValue((uint8_t *)buf, n);
  nusTxChar->notify();
  Serial.printf("[esp32-watch] sent music control: %s\n", cmd);
}

// ---------- WiFi / NTP time sync ----------
// True if it's been over intervalSec of real wall-clock time since the last
// resync *attempt* (successful or not — see rtcLastSyncAttemptEpoch), or if
// the clock isn't valid yet at all. Callers should stamp
// rtcLastSyncAttemptEpoch = time(nullptr) right before they actually attempt
// a sync, win or lose, so a stretch with no WiFi in range retries at a
// steady interval instead of every single wake. Two call sites use two
// different intervals on purpose — see WAKE_RESYNC_INTERVAL_SEC above.
bool wifiResyncDue(time_t intervalSec = WIFI_RESYNC_INTERVAL_SEC) {
  if (!timeSynced) return true; // no valid clock yet — always worth trying
  return (time(nullptr) - rtcLastSyncAttemptEpoch) > intervalSec;
}

// Shared by every path below once WiFi is already associated. Blocking on
// purpose — same reasoning as the functions that call it.
bool fetchNtpTime(uint32_t timeoutMs) {
  configTzTime(POSIX_TZ, "pool.ntp.org", "time.google.com");
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    time_t now = time(nullptr);
    if (now > 1700000000) return true; // sane "this is a real date" sanity check
    delay(200);
  }
  return false;
}

// Quick reconnect using whatever credentials are already saved to flash
// (from a prior WiFiManager portal run). No portal, no AP — this is what
// boot (after the first one) and the periodic 4h resync use, so the watch
// never surprises you with a setup hotspot mid-day just because it's
// temporarily out of range of home WiFi.
bool syncTimeViaWifi(uint32_t timeoutMs) {
  Serial.println("[esp32-watch] connecting to WiFi (saved credentials)...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(); // ESP-IDF replays the last SSID/pass it persisted to NVS

  // NOTE on every WiFi.disconnect(wifioff, eraseap) call in this file:
  // the second argument MUST stay false. eraseap=true zeroes the stored
  // STA config in NVS — i.e. it deletes the very credentials WiFiManager
  // just saved. That was harmless back when the SSID/pass were hardcoded
  // constants, but with WiFiManager owning them it means "forget my WiFi
  // on every successful sync", which sends the watch back to the setup
  // portal on the next boot, every time.

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[esp32-watch] WiFi connect failed/timed out.");
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println("[esp32-watch] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);

  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF); // save power — we don't need WiFi between syncs

  if (gotTime) {
    Serial.println("[esp32-watch] NTP sync OK.");
    timeSynced = true;
    forceRedraw = true;
  } else {
    Serial.println("[esp32-watch] NTP sync timed out.");
  }
  return gotTime;
}

// Draws the "join the setup hotspot" prompt shown for as long as the
// captive portal AP is up. WiFiManager's blocking portal loop doesn't give
// us a redraw callback beyond "AP just started", so this is a one-shot
// draw, not a live screen — fine, since nothing on it needs to change.
void drawWifiSetupScreen() {
  display.startWrite();
  display.fillScreen(COL_BG);
  display.setTextDatum(middle_center);
  useSharpSmallFont();
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString("WIFI SETUP", 120, 82);
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString("join hotspot:", 120, 112);
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString(WIFI_SETUP_AP_NAME, 120, 134);
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString("on your phone, then", 120, 160);
  display.drawString("follow the prompt to", 120, 180);
  display.drawString("pick your WiFi", 120, 200);
  display.endWrite();
}

// First-boot (or nothing valid saved) path: tries saved credentials first —
// fast, no AP — and only pops the setup hotspot if that fails. Call once,
// at boot.
bool wifiConnectOrSetup() {
  // BLE and WiFi share one 2.4GHz radio on the S3 — advertising still
  // running while the setup portal's AP/DNS/web server are also active
  // means the two contend for airtime, which was making the portal itself
  // (network scan, saving credentials) slow or unresponsive enough to lose
  // the race against a phone's own "this network has no internet, give up"
  // timeout (~30s on Android). Paused for the duration, resumed after.
  if (bleAdvertising) bleAdvertising->stop();

  WiFiManager wm;
  wm.setConnectTimeout(15);       // seconds to try saved creds before giving up
  wm.setConfigPortalTimeout(180); // don't let the portal hang boot forever if no one responds
  // Respond to the phone the instant credentials are saved, instead of
  // first spending several more seconds actually testing that the new
  // connection works — that verification step was exactly the kind of
  // delay a phone's own patience runs out on first.
  wm.setBreakAfterConfig(true);
  wm.setAPCallback([](WiFiManager *) { drawWifiSetupScreen(); });

  bool connected = wm.autoConnect(WIFI_SETUP_AP_NAME);
  if (bleAdvertising) bleAdvertising->start();
  if (!connected) {
    Serial.println("[esp32-watch] WiFi setup portal timed out with no connection.");
    forceRedraw = true;
    return false;
  }

  Serial.println("[esp32-watch] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  if (gotTime) {
    timeSynced = true;
    Serial.println("[esp32-watch] NTP sync OK.");
  } else {
    Serial.println("[esp32-watch] NTP sync timed out.");
  }
  forceRedraw = true;
  return gotTime;
}

// Manually triggered (long-press on the STAT screen) — always reopens the
// portal, even with credentials already saved, so you can point the watch
// at a different network (e.g. your phone's hotspot) without first wiping
// the old one.
bool forceWifiSetupPortal() {
  drawWifiSetupScreen();

  // Same reasoning as wifiConnectOrSetup() — see its comment.
  if (bleAdvertising) bleAdvertising->stop();

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.setBreakAfterConfig(true);
  bool connected = wm.startConfigPortal(WIFI_SETUP_AP_NAME);
  if (bleAdvertising) bleAdvertising->start();

  bool gotTime = false;
  if (connected) {
    Serial.println("[esp32-watch] WiFi connected, fetching NTP time...");
    gotTime = fetchNtpTime(8000);
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    if (gotTime) {
      timeSynced = true;
      Serial.println("[esp32-watch] NTP sync OK.");
    }
  } else {
    Serial.println("[esp32-watch] WiFi setup portal cancelled/timed out.");
  }
  forceRedraw = true;
  return gotTime;
}

// ---------- battery ----------
// Called from every render() — up to once/second while a screen's active —
// but a battery percentage doesn't need better than ~30s resolution, and
// each ADC read has its own small power/time cost. Cached and throttled
// internally so every call site benefits without needing to know about it.
// Returns BATT_UNKNOWN when the reading can't plausibly be a LiPo cell —
// most importantly when no battery is connected at all, where the divider
// reads whatever the charger leaves on the floating BAT node. Reporting a
// confident "0%" or "100%" for that is worse than admitting we don't know,
// so callers render "--" instead.
//
// Note this can't distinguish "no battery, USB powered" from "full battery"
// by voltage alone if the empty node happens to sit near 4.2V; the ETA6098's
// STAT pin (which would tell us) isn't wired to a GPIO on this board. The
// raw millivolts are logged on each fresh sample so the thresholds can be
// checked against what this board actually reads.
static const int BATT_UNKNOWN = -1;
static const int BATT_MIN_PLAUSIBLE_MV = 2800; // below a LiPo's protection cutoff
static const int BATT_MAX_PLAUSIBLE_MV = 4350; // above a full cell plus charge headroom
// Forces an early sleep (see enterCriticalBatterySleep(), called from
// setup()) at or below this reading. Set well above BATT_MIN_PLAUSIBLE_MV
// and above a typical LiPo protection IC's own cutoff (~2.9-3.0V) on
// purpose — the goal is real margin before the cell gets anywhere near
// whatever this board's one observed "won't power back on without a
// physical battery reconnect" episode needed (see
// esp32-watch-battery-reconnect-fix in project memory), not to shave the
// cutoff as close as the hardware allows. 3350mV sits just above the
// lipoPercent() curve's own 0% floor (3300mV).
static const int BATT_CRITICAL_MV = 3350;

// A LiPo's voltage/charge relationship is markedly non-linear: it falls
// quickly off the top, sits on a long plateau through the middle, then
// drops away at the bottom. The straight line this replaced under-read the
// plateau badly — most of a cell's usable life is spent between 3.7V and
// 4.0V, which a linear 3.3-4.2V map compresses into a third of the scale.
//
// Note these are *resting* voltages. The watch never stops drawing current,
// so every reading is taken under load and sits a little below the true
// state of charge. No attempt is made to compensate for that: the offset
// depends on the load at that instant, and inventing a correction factor
// would be guessing dressed up as precision.
int lipoPercent(int mv) {
  static const struct { int mv; int pct; } curve[] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3950, 70}, {3900, 60}, {3850, 50},
    {3800, 40}, {3750, 30}, {3700, 20}, {3600, 10}, {3300, 0},
  };
  const uint8_t n = sizeof(curve) / sizeof(curve[0]);
  if (mv >= curve[0].mv) return 100;
  if (mv <= curve[n - 1].mv) return 0;
  for (uint8_t i = 0; i + 1 < n; i++) {
    if (mv <= curve[i].mv && mv > curve[i + 1].mv) {
      int span = curve[i].mv - curve[i + 1].mv;
      int into = mv - curve[i + 1].mv;
      return curve[i + 1].pct + (curve[i].pct - curve[i + 1].pct) * into / span;
    }
  }
  return 0;
}

// Shared by readBatteryPercent() and the critical-battery check in setup()
// — same 8-sample average and 200k/100k divider scaling either way, so the
// two can't ever disagree about what the cell's actual voltage is.
// Averaged because a single ESP32 ADC read carries several tens of mV of
// noise, which on a LiPo curve is worth a percent or two of jitter —
// enough to make the displayed number wander while nothing is happening.
int readBatteryMilliVolts() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 8; i++) acc += analogReadMilliVolts(PIN_BATT_ADC);
  return (int)(acc / 8) * 3; // 200k/100k divider
}

int readBatteryPercent() {
  static int cachedPct = BATT_UNKNOWN;
  static uint32_t lastReadMs = 0;
  static bool everRead = false;
  uint32_t now = millis();
  if (!everRead || now - lastReadMs > 30000) {
    int mv = readBatteryMilliVolts();
    everRead = true;
    lastReadMs = now;
    if (mv < BATT_MIN_PLAUSIBLE_MV || mv > BATT_MAX_PLAUSIBLE_MV) {
      Serial.printf("[esp32-watch] battery: %dmV -> no plausible cell, reporting unknown\n", mv);
      cachedPct = BATT_UNKNOWN;
    } else {
      cachedPct = lipoPercent(mv);
      Serial.printf("[esp32-watch] battery: %dmV -> %d%%\n", mv, cachedPct);
    }
  }
  return cachedPct;
}

// ---------- persistence ----------
void loadState() {
  prefs.begin("esp32-watch", false);
  stepCounter.begin(prefs.getUInt("steps", 0));
  lastDayMarker = prefs.getUInt("dayMarker", 0);

  // Past 6 completed days' totals for the HISTORY screen. getBytes() returns
  // 0 (not sizeof(stepHist)) on first-ever boot, when the key doesn't exist
  // yet — treated the same as "no history", not as corruption.
  size_t histLen = prefs.getBytes("stepHist", stepHist, sizeof(stepHist));
  if (histLen != sizeof(stepHist)) memset(stepHist, 0, sizeof(stepHist));

  // Restore the last weather so a reboot isn't stuck on "NO DATA" until the
  // phone next pushes. Age is judged at draw time, not here: this runs
  // before NTP/BLE time sync, so the clock usually isn't trustworthy yet.
  weatherEpoch = (time_t)prefs.getULong("wEpoch", 0);
  if (weatherEpoch > 0) {
    weatherTempC = prefs.getInt("wTemp", 0);
    weatherHiC = prefs.getInt("wHi", 0);
    weatherLoC = prefs.getInt("wLo", 0);
    weatherTxt = prefs.getString("wTxt", "");
    weatherCode = prefs.getInt("wCode", 0);
    hasWeather = true;
  }
  prefs.end();
}

void saveWeather() {
  prefs.begin("esp32-watch", false);
  prefs.putULong("wEpoch", (uint32_t)weatherEpoch);
  prefs.putInt("wTemp", weatherTempC);
  prefs.putInt("wHi", weatherHiC);
  prefs.putInt("wLo", weatherLoC);
  prefs.putString("wTxt", weatherTxt);
  prefs.putInt("wCode", weatherCode);
  prefs.end();
}

// Seconds since the current reading arrived, or -1 when that can't be known
// (no reading, or it arrived before the clock was set).
long weatherAgeSec() {
  if (!hasWeather || weatherEpoch == 0) return -1;
  time_t now = time(nullptr);
  if (now < 1700000000) return -1; // clock not trustworthy yet
  long age = (long)(now - weatherEpoch);
  return age < 0 ? -1 : age; // negative means the clock moved backwards; treat as unknown
}

// True when a restored reading is old enough that showing it would mislead.
bool weatherTooStale() {
  long age = weatherAgeSec();
  return age >= 0 && age > WEATHER_MAX_AGE_SEC;
}

void saveState() {
  // Flash writes cost real current (and wear) — skip the NVS commit
  // entirely on the common case of "nothing changed since last save"
  // (e.g. sitting still between steps). Doesn't change the save
  // guarantee at all: still checked/written every 30s the instant
  // something *does* change, same window as before.
  static uint32_t lastSavedSteps = UINT32_MAX;
  static uint32_t lastSavedDayMarker = UINT32_MAX;
  uint32_t steps = stepCounter.count();
  if (steps == lastSavedSteps && lastDayMarker == lastSavedDayMarker) return;

  prefs.begin("esp32-watch", false);
  prefs.putUInt("steps", steps);
  prefs.putUInt("dayMarker", lastDayMarker);
  // stepHist only actually changes at day-rollover, which is exactly the
  // moment lastDayMarker changes too — so this write is never wasted, and
  // never needs its own separate change-tracking.
  prefs.putBytes("stepHist", stepHist, sizeof(stepHist));
  prefs.end();
  lastSavedSteps = steps;
  lastSavedDayMarker = lastDayMarker;
}

// ---------- power ----------
// Lets the CPU automatically drop clock speed (down to min_freq_mhz) and
// grab brief automatic light-sleep windows during any idle time in loop()
// (e.g. every delay(), every blocking wait) — completely transparent to
// application timing, so nothing about polling rates, redraw cadence, or
// reconnect behavior changes even when it's working.
//
// IMPORTANT: this requires the esp32 Arduino core itself to have been
// *built* with CONFIG_PM_ENABLE (FreeRTOS tickless idle) turned on. As of
// core 3.3.11, the official prebuilt board package does NOT have that set
// (confirmed via its sdkconfig) — esp_pm_configure() still links and runs,
// but is compiled down to a no-op that returns ESP_ERR_NOT_SUPPORTED,
// nothing more. Called anyway, and logged honestly either way, rather than
// assumed: if a future core release (or a custom-built one via
// esp32-arduino-lib-builder) ships with it on, this starts working with no
// code changes needed here — but don't take "no crash" as "it's working"
// without checking this exact log line on real hardware.
void setupPowerManagement() {
  esp_pm_config_t pmConfig = {
    .max_freq_mhz = 80,  // matches the fixed setCpuFrequencyMhz(80) ceiling above
    .min_freq_mhz = 40,  // drop here when idle; still responsive enough for I2C/SPI/BLE
    .light_sleep_enable = true,
  };
  esp_err_t err = esp_pm_configure(&pmConfig);
  if (err == ESP_OK) {
    Serial.println("[esp32-watch] automatic light sleep ENABLED.");
  } else {
    Serial.printf("[esp32-watch] esp_pm_configure() -> %s — not active (core likely lacks CONFIG_PM_ENABLE)\n",
                   esp_err_to_name(err));
  }
}

// This board has no physical power switch, so this is the closest
// equivalent: a real ESP32 deep sleep (CPU, display, radios all powered
// down — this is what actually saves battery, unlike the backlight-only
// dim the idle timeout does) that wakes back up on the next touch. Deep
// sleep is a full reset — everything except the RTC timer and flash-backed
// state loses power, so the only way back is through setup() again from the
// top. See the wake-from-sleep fast path near the top of setup().
// Shared tail for every path into deep sleep: powers down the peripherals
// and drops the CPU, waking on the next touch. Split out of enterDeepSleep()
// so the critical-battery path (see enterCriticalBatterySleep()) gets
// exactly the same power-down sequencing without duplicating it.
void powerDownAndDeepSleep() {
  // Put the peripherals down before the CPU. Deep sleep only powers down
  // the ESP32 itself — the panel, touch controller and IMU sit on their own
  // rails and keep drawing whatever they were drawing, which on a watch-sized
  // cell is the difference that matters.
  if (imuOk) {
    imu.enableAccel(false);
    imu.enableGyro(false);
  }
  // Auto-sleep is what lets the controller idle *and* still pull INT on a
  // touch — see setAutoSleep() for why this is the opposite of the setting
  // used while awake. If waking by touch ever stops working, this line is
  // the first thing to remove.
  if (touchOk) touch.setAutoSleep(true);

  display.sleep(); // backlight off and the GC9A01 into its own sleep mode
  delay(50);       // let the backlight PWM settle before power drops

  // CST816S holds its INT pin high and pulses it low on a touch event —
  // ext0 wakes the chip the moment that pin goes low. GPIO5 is within the
  // S3's low-power IO domain, which ext0 requires.
  //
  // TODO (hardware pending): physical power button.
  //   The rear header brings out GPIO15/16/17/18/21/33 plus GND and 3V3,
  //   none of which this firmware uses. A momentary button between GND and
  //   one of GPIO15/16/17/18/21 is all the hardware needed — the internal
  //   pull-up idles the pin high and a press pulls it low. Not GPIO33: only
  //   GPIO0-21 are RTC-capable on the S3, and a non-RTC pin cannot wake the
  //   chip from deep sleep at all.
  //
  //   Two firmware changes go with it:
  //     1. ext0 handles exactly one pin, so waking on touch *and* a button
  //        means moving to esp_sleep_enable_ext1_wakeup() with both pins in
  //        the mask and ESP_EXT1_WAKEUP_ANY_LOW. Both idle high, so that
  //        mode suits them without further conditioning.
  //     2. Internal pull-ups are switched off entering deep sleep, so the
  //        button pin needs rtc_gpio_pullup_en() or it floats — and a
  //        floating input will wake the watch at random, which reads as
  //        "the battery drains overnight" rather than as a wiring fault.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TOUCH_INT, 0);
  esp_deep_sleep_start();
  // Never reaches here — the reset vector (setup()) is the only way out.
}

void enterDeepSleep() {
  Serial.println("[esp32-watch] long-press -> entering deep sleep...");
  saveState(); // catch any steps counted since the last periodic save

  // Say so on screen before going dark. Without this, sleeping looks
  // identical to a crash or a reboot — and since waking is a full restart,
  // there's otherwise nothing to distinguish "it slept and woke" from "it
  // never slept at all".
  display.startWrite();
  display.fillScreen(COL_BG);
  display.setTextDatum(middle_center);
  useSharpMedFont();
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString("SLEEP", 120, 112);
  useSharpSmallFont();
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString("touch to wake", 120, 142);
  display.endWrite();
  delay(900);

  display.startWrite();
  display.fillScreen(COL_BG);
  display.endWrite();

  powerDownAndDeepSleep();
}

// Forces sleep immediately when the cell is critically low — before WiFi,
// BLE, or (on a sleep wake) the Vault Boy animation get a chance to spend
// any current, since none of that is worth running at this state of
// charge. This board has needed a physical battery-connector reconnect to
// come back once already after running all the way down (see
// esp32-watch-battery-reconnect-fix in project memory) — whatever caused
// that, the actual fix is never letting the cell get that low again, not
// hoping the recovery works better next time. Shows a brief warning first:
// a watch that goes dark with no explanation reads as broken, which is
// exactly the confusion that started this in the first place.
void enterCriticalBatterySleep(int mv) {
  Serial.printf("[esp32-watch] critical battery (%dmV) — sleeping immediately to protect the cell\n", mv);

  display.startWrite();
  display.fillScreen(COL_BG);
  display.setTextDatum(middle_center);
  useSharpMedFont();
  display.setTextColor(COL_AMBER, COL_BG);
  display.drawString("BATTERY LOW", 120, 104);
  useSharpSmallFont();
  display.setTextColor(COL_AMBER, COL_BG);
  display.drawString("charge to continue", 120, 136);
  char mvbuf[16];
  snprintf(mvbuf, sizeof(mvbuf), "(%dmV)", mv);
  display.drawString(mvbuf, 120, 158);
  display.endWrite();
  delay(1500); // long enough to actually read before it goes dark again

  display.startWrite();
  display.fillScreen(COL_BG);
  display.endWrite();

  powerDownAndDeepSleep();
}

// Un-dims the backlight and marks the screen for a redraw — shared by every
// wake path (touch, an incoming notification, and now raise-to-wake) so
// they can't drift out of sync with each other. Also clears the raise-to-
// wake burst tracker: if some *other* wake path fires mid-burst, that
// burst's context (how long it's been rotating) is now stale and shouldn't
// resolve later against a screen state that's already changed.
void wakeScreen(uint32_t now) {
  display.setBrightness(BRIGHT_ACTIVE);
  backlightDimmed = false;
  forceRedraw = true;
  lastActivityMs = now;
  raiseGyroActive = false;
}

// Detects "raised the wrist to look at the watch" from the gyro+accel and
// wakes the screen the same way a touch would. Two-part heuristic: (1) a
// burst of rotation fast enough, and brief enough, to look like a
// deliberate wrist-turn rather than ambient jitter or a slow drift: then
// (2) once that burst ends, check whether the accelerometer settled into
// roughly the orientation you'd expect if the screen is now facing you.
// Requiring both cuts down on false wakes that either alone would miss —
// e.g. gyro motion alone would also fire on any fast wrist movement that
// never actually ends with the watch face up (swinging your arm while
// walking); the accel orientation alone can't tell a deliberate raise from
// just holding your arm still in that position.
//
// Only meaningful (and only called) while dimmed — there's nothing to wake
// if the screen's already on.
void updateRaiseToWake(bool accelOk, float ax, float ay, float az, uint32_t now) {
  float gyroMag = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
  bool fastNow = gyroMag > RAISE_GYRO_THRESHOLD_DPS;

  if (fastNow && !raiseGyroActive) {
    raiseGyroActive = true;
    raiseGyroStartMs = now;
    raiseGyroPeak = gyroMag;
  } else if (fastNow && raiseGyroActive) {
    if (gyroMag > raiseGyroPeak) raiseGyroPeak = gyroMag;
  } else if (!fastNow && raiseGyroActive) {
    uint32_t heldMs = now - raiseGyroStartMs;
    float peak = raiseGyroPeak;
    raiseGyroActive = false;

    bool durationLooksRight = heldMs >= RAISE_GYRO_MIN_MS && heldMs <= RAISE_GYRO_MAX_MS;
    bool offCooldown = now - lastRaiseWakeMs > RAISE_COOLDOWN_MS;
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    float dot = (accelOk && amag > 0.1f)
                    ? (ax * RAISE_ACCEL_TARGET_X + ay * RAISE_ACCEL_TARGET_Y + az * RAISE_ACCEL_TARGET_Z) / (amag * 9.8f)
                    : -9.0f;

    // Every candidate motion is logged, fired or not, with the numbers each
    // gate actually saw. The thresholds and the target orientation were
    // guesses about how this IMU is mounted relative to "screen facing
    // you"; this is what turns them into measured values. ax/ay/az here is
    // the settled orientation at the end of the motion.
    Serial.printf("[esp32-watch] raise? held=%lums peak=%.0fdps a=(%.1f,%.1f,%.1f) dot=%.2f dur=%d cool=%d\n",
                  (unsigned long)heldMs, peak, ax, ay, az, dot,
                  durationLooksRight, offCooldown);

    if (durationLooksRight && offCooldown && dot > RAISE_ACCEL_DOT_MIN) {
      wakeScreen(now);
      lastRaiseWakeMs = now;
      Serial.println("[esp32-watch] raise-to-wake triggered.");
    }
  }
}

// ---------- drawing helpers ----------

// Sharp bold monospace face for everything except the time (which stays on
// Font7 for the 7-segment digital-clock look) — the old bitmap fonts read
// too soft/rounded up close. Always set font+size together so there's no
// risk of a leftover scale from one screen leaking into another.
void useSharpSmallFont() { display.setFont(&fonts::FreeMonoBold9pt7b); display.setTextSize(0.9f); }
void useSharpMedFont()   { display.setFont(&fonts::FreeMonoBold12pt7b); display.setTextSize(1.0f); }
// A step up from the small face for the STAT screen's date/weather rows,
// but held just under full medium: at 1.0 a full-width date overruns the
// round panel, and these rows sit where the circle is already narrowing.
void useSharpRowFont()   { display.setFont(&fonts::FreeMonoBold12pt7b); display.setTextSize(0.85f); }

// Trims a string until it fits maxW pixels in the *currently set* font,
// marking the cut with "..." — the FreeMono faces have no ellipsis glyph.
// Used for notification text, where an app name plus a sender can easily
// outrun the usable width of a round 240px panel.
String fitToWidth(const String &s, int maxW) {
  if (display.textWidth(s) <= maxW) return s;
  String out = s;
  while (out.length() > 1 && display.textWidth(out + "...") > maxW) {
    out.remove(out.length() - 1);
  }
  return out + "...";
}

// Bluetooth rune, traced from the real logo (spine + one zigzag flag) and
// drawn with anti-aliased lines instead of the old plain on/off dot.
void drawBluetoothIcon(int cx, int cy, bool connected, uint16_t accent = COL_GREEN) {
  const float w = 8, h = 14;
  float x0 = cx - w / 2, y0 = cy - h / 2;
  // Dim still means "not connected" — that's state, not decoration, so it
  // survives the per-screen tinting.
  uint16_t col = connected ? accent : COL_GREEN_DIM;
  float topX = x0 + w / 2, topY = y0;
  float ctrX = x0 + w / 2, ctrY = y0 + h / 2;
  float botX = x0 + w / 2, botY = y0 + h;
  float upX = x0 + w, upY = y0 + h * 0.28f;
  float loX = x0 + w, loY = y0 + h * 0.72f;
  // A true hairline's AA edge is only a fraction of a pixel wide — barely
  // visible on a physical screen this dense. A bit of real width gives the
  // blending something to actually show.
  const float lw = 1.4f;
  display.drawWideLine(topX, topY, ctrX, ctrY, lw, col);
  display.drawWideLine(topX, topY, upX, upY, lw, col);
  display.drawWideLine(upX, upY, ctrX, ctrY, lw, col);
  display.drawWideLine(ctrX, ctrY, botX, botY, lw, col);
  display.drawWideLine(ctrX, ctrY, loX, loY, lw, col);
  display.drawWideLine(loX, loY, botX, botY, lw, col);
}

// Battery glyph (rounded outline + terminal nub + proportional fill bar)
// instead of the old "BATT 82%" text label.
void drawBatteryIcon(int x, int y, int pct, uint16_t accent = COL_GREEN) {
  const int BW = 20, BH = 11, RAD = 3, NUBW = 2, NUBH = 5, BORDER = 2, INSET = 2;
  display.fillSmoothRoundRect(x, y, BW, BH, RAD, accent);
  display.fillSmoothRoundRect(x + BORDER, y + BORDER, BW - 2 * BORDER, BH - 2 * BORDER, RAD - BORDER, COL_BG);
  display.fillRect(x + BW, y + (BH - NUBH) / 2, NUBW, NUBH, accent);

  if (pct == BATT_UNKNOWN) return; // empty shell — no cell to represent

  int innerW = BW - 2 * BORDER - 2 * INSET;
  int fw = innerW * pct / 100;
  if (fw < 0) fw = 0;
  if (fw > innerW) fw = innerW;
  if (fw > 0) {
    // Amber below 20% regardless of the section's colour — a nearly flat
    // battery should look the same wherever you happen to be looking.
    uint16_t col = pct > 20 ? accent : COL_AMBER;
    display.fillRect(x + BORDER + INSET, y + BORDER + INSET, fw, BH - 2 * BORDER - 2 * INSET, col);
  }
}

// Bluetooth + battery + percentage, centered together as one group — the
// same exact position/style on every screen. Width measured at runtime so
// it stays centered regardless of whether the percentage is 1, 2, or 3
// digits.
void drawBleBattCluster(int battPct, uint16_t col = COL_GREEN) {
  display.setTextDatum(middle_center);
  useSharpMedFont();
  char pctbuf[8];
  if (battPct == BATT_UNKNOWN) {
    snprintf(pctbuf, sizeof(pctbuf), "--");
  } else {
    snprintf(pctbuf, sizeof(pctbuf), "%d%%", battPct);
  }
  int pctW = display.textWidth(pctbuf);
  const int BT_W = 8, GAP1 = 10, BATT_W = 22, GAP2 = 5;
  int totalW = BT_W + GAP1 + BATT_W + GAP2 + pctW;
  int startX = 120 - totalW / 2;
  int rowY = 213;

  drawBluetoothIcon(startX + BT_W / 2, rowY, bleConnected, col);
  int battX = startX + BT_W + GAP1;
  drawBatteryIcon(battX, rowY - 5, battPct, col);
  display.setTextDatum(middle_left);
  display.setTextColor(col, COL_BG);
  display.drawString(pctbuf, battX + BATT_W + GAP2, rowY);
  display.setTextDatum(middle_center);
}

// ---------- temperature rendering ----------
// Two things make a plain "23°C" string impossible to just drawString():
//   1. Font7 is the 7-segment face — digits only, no letters at all, so a
//      "C" appended to the number silently vanishes (this is exactly why
//      the big temperature used to render as a bare number).
//   2. None of the fonts here have a degree glyph — the Adafruit GFX
//      FreeMono faces stop at 0x7E.
// So the number, the degree ring, and the "C" are drawn as three separate
// pieces, measured and laid out as one centered group.

// Big Font7 temperature with a small degree+C to its right, centered on cx.
static const int BIGTEMP_GAP = 8, BIGTEMP_RING_R = 5, BIGTEMP_RING_GAP = 4;

// Width of the whole number+degree+C group. Split out from the drawing so
// callers can lay the group out beside the weather icon and centre the two
// together as one unit.
int bigTempWidth(int tempC) {
  char numbuf[8];
  snprintf(numbuf, sizeof(numbuf), "%d", tempC);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  int numW = display.textWidth(numbuf);
  display.setTextSize(1.0f);
  useSharpMedFont();
  return numW + BIGTEMP_GAP + BIGTEMP_RING_R * 2 + BIGTEMP_RING_GAP + display.textWidth("C");
}

// Draws the group with its left edge at x (not centred — see bigTempWidth).
void drawBigTempAt(int x, int cy, int tempC, uint16_t col = COL_GREEN) {
  char numbuf[8];
  snprintf(numbuf, sizeof(numbuf), "%d", tempC);

  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  int numW = display.textWidth(numbuf);
  display.setTextSize(1.0f);

  const int GAP = BIGTEMP_GAP, RING_R = BIGTEMP_RING_R, RING_GAP = BIGTEMP_RING_GAP;

  display.setTextDatum(middle_left);
  display.setTextColor(col, COL_BG);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  display.drawString(numbuf, x, cy);
  display.setTextSize(1.0f);

  // Ring sits up near the top of the digits, the way a degree mark reads.
  int ringX = x + numW + GAP + RING_R;
  int ringY = cy - 13;
  display.drawCircle(ringX, ringY, RING_R, col);
  display.drawCircle(ringX, ringY, RING_R - 1, col); // doubled so it survives at this size

  useSharpMedFont();
  display.setTextColor(col, COL_BG);
  display.drawString("C", ringX + RING_R + RING_GAP, cy);
  display.setTextDatum(middle_center);
}

// ---------- weather icons ----------
// Drawn from primitives rather than stored as bitmaps: they're simple
// shapes, they scale to whatever box you hand them, and they stay in the
// single-colour phosphor look without another blob of pixel data in flash.
// Every shape is expressed as a percentage of `s` so the whole set moves
// together if the size changes.

// Prefers the OpenWeatherMap condition code (exact, language-independent)
// and only falls back to sniffing the condition text when the code is
// missing — provider text is free-form and localised, so it's the less
// trustworthy of the two.
WeatherIcon weatherIconFor(int code, const String &txt) {
  if (code >= 200 && code < 300) return WI_STORM;
  if (code >= 300 && code < 600) return WI_RAIN;  // drizzle + rain
  if (code >= 600 && code < 700) return WI_SNOW;
  if (code >= 700 && code < 800) return WI_FOG;
  if (code == 800) return WI_SUN;
  if (code == 801 || code == 802) return WI_PARTLY;
  if (code == 803 || code == 804) return WI_CLOUD;

  String t = txt;
  t.toLowerCase();
  if (t.indexOf("thunder") >= 0 || t.indexOf("storm") >= 0) return WI_STORM;
  if (t.indexOf("snow") >= 0 || t.indexOf("sleet") >= 0 || t.indexOf("hail") >= 0) return WI_SNOW;
  if (t.indexOf("rain") >= 0 || t.indexOf("shower") >= 0 || t.indexOf("drizzle") >= 0) return WI_RAIN;
  if (t.indexOf("fog") >= 0 || t.indexOf("mist") >= 0 || t.indexOf("haze") >= 0) return WI_FOG;
  if (t.indexOf("partly") >= 0 || t.indexOf("few") >= 0 || t.indexOf("scattered") >= 0) return WI_PARTLY;
  if (t.indexOf("clear") >= 0 || t.indexOf("sun") >= 0) return WI_SUN;
  // Anything unrecognised gets the neutral cloud rather than a confident
  // sun — better to under-claim than to draw a clear sky over a storm.
  return WI_CLOUD;
}

// Sun disc plus eight rays, centred in the box (or wherever cx/cy say).
void drawIconSun(int cx, int cy, int s, uint16_t col) {
  int r = s * 20 / 100;
  display.fillCircle(cx, cy, r, col);
  int inner = r + s * 6 / 100, outer = r + s * 15 / 100;
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4.0f;
    display.drawWideLine(cx + cosf(a) * inner, cy + sinf(a) * inner,
                         cx + cosf(a) * outer, cy + sinf(a) * outer, s * 5 / 100, col);
  }
}

// Cloud built from three puffs over a flat-bottomed rounded base.
void drawIconCloud(int x, int y, int s, uint16_t col) {
  display.fillCircle(x + s * 40 / 100, y + s * 46 / 100, s * 20 / 100, col);
  display.fillCircle(x + s * 63 / 100, y + s * 55 / 100, s * 16 / 100, col);
  display.fillCircle(x + s * 22 / 100, y + s * 56 / 100, s * 14 / 100, col);
  display.fillSmoothRoundRect(x + s * 10 / 100, y + s * 56 / 100,
                              s * 70 / 100, s * 20 / 100, s * 8 / 100, col);
}

// Three slanted streaks under a cloud, shared by rain and (shorter) drizzle.
void drawIconRainStreaks(int x, int y, int s, uint16_t col) {
  for (int i = 0; i < 3; i++) {
    int sx = x + s * (24 + i * 20) / 100;
    display.drawWideLine(sx, y + s * 80 / 100, sx - s * 7 / 100, y + s * 96 / 100, s * 5 / 100, col);
  }
}

void drawWeatherIcon(int x, int y, int s, WeatherIcon icon, uint16_t col) {
  // Wipe the box first: shapes drawn with fills don't erase what a
  // different icon left behind on an earlier frame.
  display.fillRect(x, y, s, s, COL_BG);

  switch (icon) {
    case WI_SUN:
      drawIconSun(x + s / 2, y + s / 2, s, col);
      break;

    case WI_PARTLY:
      // Sun peeking out of the top-left, cloud overlapping its lower right.
      drawIconSun(x + s * 38 / 100, y + s * 36 / 100, s * 78 / 100, col);
      display.fillCircle(x + s * 40 / 100, y + s * 46 / 100, s * 20 / 100, COL_BG); // keep the cloud's edge readable against the rays
      drawIconCloud(x, y + s * 12 / 100, s, col);
      break;

    case WI_CLOUD:
      drawIconCloud(x, y + s * 6 / 100, s, col);
      break;

    case WI_RAIN:
      drawIconCloud(x, y - s * 6 / 100, s, col);
      drawIconRainStreaks(x, y - s * 6 / 100, s, col);
      break;

    case WI_SNOW:
      drawIconCloud(x, y - s * 6 / 100, s, col);
      for (int i = 0; i < 3; i++) {
        display.fillCircle(x + s * (26 + i * 20) / 100, y + s * 82 / 100, s * 5 / 100, col);
      }
      break;

    case WI_STORM:
      drawIconCloud(x, y - s * 8 / 100, s, col);
      // Lightning bolt: two offset triangles make the classic zigzag.
      display.fillTriangle(x + s * 52 / 100, y + s * 66 / 100,
                           x + s * 34 / 100, y + s * 66 / 100,
                           x + s * 46 / 100, y + s * 90 / 100, col);
      display.fillTriangle(x + s * 56 / 100, y + s * 62 / 100,
                           x + s * 40 / 100, y + s * 78 / 100,
                           x + s * 58 / 100, y + s * 78 / 100, col);
      break;

    case WI_FOG:
      drawIconCloud(x, y - s * 12 / 100, s, col);
      for (int i = 0; i < 3; i++) {
        int ly = y + s * (68 + i * 12) / 100;
        int inset = (i % 2) ? s * 22 / 100 : s * 12 / 100;
        display.fillSmoothRoundRect(x + inset, ly, s - inset * 2, s * 5 / 100, s * 2 / 100, col);
      }
      break;
  }
}

// Small one-line "23°C  clear sky" row, centered on cx. Same three-piece
// trick as the big temperature, at the small text size.
void drawTempRow(int cx, int cy, int tempC, const String &desc, uint16_t col) {
  useSharpRowFont();
  char numbuf[8];
  snprintf(numbuf, sizeof(numbuf), "%d", tempC);

  int numW = display.textWidth(numbuf);
  int cW = display.textWidth("C");
  const int GAP = 3, RING_R = 3, RING_GAP = 2;

  // Trim the condition to whatever room is left after the temperature. At
  // this larger size a long one ("partly cloudy") would otherwise run off
  // both edges of the round panel.
  const int ROW_MAX_W = 200;
  int fixedW = numW + GAP + RING_R * 2 + RING_GAP + cW;
  String tail = desc.length() ? fitToWidth("  " + desc, ROW_MAX_W - fixedW) : "";
  int tailW = tail.length() ? display.textWidth(tail) : 0;
  int totalW = numW + GAP + RING_R * 2 + RING_GAP + cW + tailW;
  int x = cx - totalW / 2;

  display.setTextDatum(middle_left);
  display.setTextColor(col, COL_BG);
  display.drawString(numbuf, x, cy);
  x += numW + GAP + RING_R;
  display.drawCircle(x, cy - 5, RING_R, col);
  x += RING_R + RING_GAP;
  display.drawString("C", x, cy);
  x += cW;
  if (tail.length()) display.drawString(tail, x, cy);
  display.setTextDatum(middle_center);
}

void drawStatScreen(struct tm &t, int steps, int battPct) {
  display.setTextDatum(middle_center);

  // Notification banner above the time — "Signal - Dakota Carrie" style,
  // app name and sender, no message body (that's what the NOTIF screen is
  // for). Blank when nothing is pending, so the clock face stays clean.
  useSharpSmallFont();
  if (notifCount) {
    // Always the newest (slot 0), regardless of where the NOTIF screen is
    // scrolled to — this line is a "latest activity" indicator, not a view
    // into the list.
    String banner = notifs[0].src;
    if (notifs[0].title.length()) banner += banner.length() ? " - " + notifs[0].title : notifs[0].title;
    if (!banner.length()) banner = "NOTIFICATION"; // neither field supplied
    display.setTextColor(COL_GREEN, COL_BG);
    display.drawString(fitToWidth(banner, 205), 120, 74);
  } else {
    // Painted over rather than skipped: redraws don't clear the screen, so
    // a dismissed banner would otherwise stay on the panel until something
    // else forced a full repaint.
    display.setTextColor(COL_GREEN, COL_BG);
    display.drawString("                        ", 120, 74);
  }

  // Time — centered in the middle of the screen, everything else stacks
  // below it. The whole block (banner/time/date/weather) was confirmed on
  // real hardware to run into the fixed BLE/battery cluster at y=213 with
  // the previous spacing — the weather condition row landed only 13px above
  // it. Shifted every row here up by another 10px (uniformly, so the
  // spacing *between* them is unchanged from what wasn't reported as
  // broken) to open that up to 23px, matching the gap already used on the
  // WEATHER screen. The cluster itself stays put; it's shared by every
  // screen.
  int hour12 = t.tm_hour % 12;
  if (hour12 == 0) hour12 = 12;
  char timebuf[16];
  snprintf(timebuf, sizeof(timebuf), "%d:%02d:%02d", hour12, t.tm_min, t.tm_sec);
  display.setTextColor(COL_GREEN, COL_BG);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  display.drawString(timebuf, 120, 102);
  display.setTextSize(1.0f);

  // Everything below uses a sharp bold monospace face instead of Font7
  // (the time's font is 7-segment-style and can't render letters at all),
  // at full green rather than the dim shade — these were noticeably harder
  // to read than the clock above them.
  display.setTextColor(COL_GREEN, COL_BG);

  char datebuf[32];
  static const char *wd[7] = {"Sun.","Mon.","Tue.","Wed.","Thu.","Fri.","Sat."};
  // Abbreviated deliberately: at this larger size the full month name runs
  // past the usable width of a round 240px panel ("Wed. September 30, 2026"
  // needs ~270px against ~220 available at this height).
  static const char *mo[12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  if (timeSynced) {
    snprintf(datebuf, sizeof(datebuf), "%s %s %d, %d", wd[t.tm_wday], mo[t.tm_mon], t.tm_mday, t.tm_year + 1900);
  } else {
    snprintf(datebuf, sizeof(datebuf), "-- NO SIGNAL --");
  }
  useSharpRowFont();
  display.drawString(datebuf, 120, 142);

  // Current temperature + condition only — the high/low pair lives on the
  // dedicated WEATHER screen, where there's room to label it properly.
  //
  // Two rows, not one: squeezing "72°C  scattered clouds" onto a single
  // line at the row font left barely any width for the condition text, so
  // anything past one short word got truncated with "...". Splitting them
  // gives the condition its own row at the full row width and a smaller
  // font — between the two, real OpenWeatherMap condition strings (almost
  // always one to three words) should clear it without truncating at all;
  // fitToWidth stays on as a safety net for the rare one that doesn't.
  if (hasWeather && !weatherTooStale()) {
    // Yellow to match the WEATHER screen this row summarises. Empty
    // description centers just the temp+ring+C group on its own row.
    drawTempRow(120, 170, weatherTempC, "", COL_YELLOW);
    useSharpSmallFont();
    display.setTextColor(COL_YELLOW, COL_BG);
    display.drawString(fitToWidth(weatherTxt, 205), 120, 190);
  } else {
    // Blank both rows so a stale reading can't linger between redraws.
    useSharpRowFont();
    display.setTextColor(COL_YELLOW, COL_BG);
    display.drawString("                    ", 120, 170);
    useSharpSmallFont();
    display.drawString("                        ", 120, 190);
  }

  drawBleBattCluster(battPct);
}

void drawDataScreen(int steps, int battPct) {
  display.setTextDatum(middle_center);

  useSharpMedFont();
  display.setTextColor(COL_ORANGE, COL_BG);
  display.drawString("Steps", 120, 80);

  char stepbuf[12];
  snprintf(stepbuf, sizeof(stepbuf), "%lu", (unsigned long)steps);
  display.setTextColor(COL_ORANGE, COL_BG);
  display.setFont(&fonts::Font7);
  display.drawString(stepbuf, 120, 120);

  // Reset affordance / confirmation prompt, in place of the distance line
  // while armed so nothing has to shuffle position.
  if (stepsResetArmedMs && millis() - stepsResetArmedMs < STEPS_RESET_ARM_MS) {
    useSharpSmallFont();
    display.setTextColor(COL_AMBER, COL_BG);
    char prompt[28];
    snprintf(prompt, sizeof(prompt), " RESET? TAP %u MORE ", STEPS_RESET_TAPS - stepsResetTaps);
    display.drawString(prompt, 120, 168);
    display.setTextColor(COL_ORANGE, COL_BG);
    display.drawString("                    ", 120, 188);
    drawBleBattCluster(battPct, COL_ORANGE);
    return;
  }

  float km = steps * 0.00075f; // ~0.75m average stride
  char distbuf[24];
  snprintf(distbuf, sizeof(distbuf), "~%.2f KM", km);
  useSharpSmallFont();
  display.setTextColor(COL_ORANGE, COL_BG);
  display.drawString(distbuf, 120, 168);
  display.drawString(" tap 4x to reset  ", 120, 188);

  drawBleBattCluster(battPct, COL_ORANGE);
}

// Bar chart of the past week: the 6 completed days from stepHist plus
// today's count read live from stepCounter, oldest to newest, left to
// right. Grouped with DATA (shares its orange accent) since it's really an
// extension of the same stat rather than its own category.
void drawHistoryScreen(int battPct) {
  display.setTextDatum(middle_center);

  useSharpMedFont();
  display.setTextColor(COL_ORANGE, COL_BG);
  display.drawString("7-Day Steps", 120, 56);

  uint32_t vals[7];
  for (uint8_t i = 0; i < 6; i++) vals[i] = stepHist[i];
  vals[6] = stepCounter.count(); // today, updates live as you walk

  uint32_t maxVal = 1; // floor of 1 avoids a divide-by-zero with no history yet
  for (uint8_t i = 0; i < 7; i++) if (vals[i] > maxVal) maxVal = vals[i];

  // Bars sit in the round panel's wide middle band — same reasoning as the
  // boot log's line placement: too near the top/bottom of a 240px circle
  // and content gets physically cut off by the bezel.
  const int CHART_TOP = 84, CHART_BOTTOM = 176, CHART_H = CHART_BOTTOM - CHART_TOP;
  const int BAR_W = 16, BAR_GAP = 6;
  int totalW = 7 * BAR_W + 6 * BAR_GAP;
  int x0 = 120 - totalW / 2;

  // Day-of-week labels, oldest to newest, ending on today — computed off
  // the live clock rather than stored, since only the totals need
  // persisting. Reads as today's weekday initial if the clock's never been
  // synced (tm_wday defaults to Thursday, epoch 0), which is cosmetically
  // wrong but harmless — there's no step history worth labelling yet at
  // that point anyway.
  static const char *dowInitial[7] = {"S", "M", "T", "W", "T", "F", "S"};
  time_t nowEpoch = time(nullptr);
  struct tm lt;
  localtime_r(&nowEpoch, &lt);

  useSharpSmallFont();
  for (uint8_t i = 0; i < 7; i++) {
    int barH = (int)((uint64_t)vals[i] * CHART_H / maxVal);
    if (barH < 1 && vals[i] > 0) barH = 1; // a real but tiny day still shows a sliver, not nothing
    int bx = x0 + i * (BAR_W + BAR_GAP);
    bool isToday = (i == 6);
    uint16_t col = isToday ? COL_ORANGE : COL_GREEN_DIM;

    // Clear the whole column first — a day that used to be the tallest bar
    // and got scaled down since (maxVal grew) would otherwise leave its old
    // height as a ghost above the new, shorter one.
    display.fillRect(bx, CHART_TOP, BAR_W, CHART_H, COL_BG);
    if (barH > 0) display.fillRect(bx, CHART_BOTTOM - barH, BAR_W, barH, col);

    int wday = ((lt.tm_wday - (6 - i)) % 7 + 7) % 7; // negative-safe mod: this bar is (6-i) days before today
    display.setTextColor(col, COL_BG);
    display.drawString(dowInitial[wday], bx + BAR_W / 2, CHART_BOTTOM + 12);
  }

  drawBleBattCluster(battPct, COL_ORANGE);
}

// Fuller weather view than the compact line embedded in STAT — same data
// (hasWeather/weatherTempC/etc., fed over BLE by Gadgetbridge, see
// handleGadgetbridgeLine()), laid out like DATA's big-number-plus-detail
// style instead of squeezed into one line.
void drawWeatherScreen(int battPct) {
  display.setTextDatum(middle_center);

  bool showWeather = hasWeather && !weatherTooStale();

  useSharpMedFont();
  display.setTextColor(COL_YELLOW, COL_BG);
  display.drawString("Weather", 120, showWeather ? 56 : 80);

  if (!showWeather) {
    display.setTextColor(COL_YELLOW, COL_BG);
    display.drawString("-- NO DATA --", 120, 120);
    useSharpSmallFont();
    // Gadgetbridge has no weather source of its own — it only relays what
    // a companion weather app (Breezy Weather, Tiny Weather Forecast
    // Germany, QuickWeather) broadcasts to it, so pairing alone never
    // produces weather. Say that, rather than implying the pairing is the
    // missing piece.
    display.drawString("needs a weather app", 120, 148);
    display.drawString("feeding Gadgetbridge", 120, 168);
    display.drawString("e.g. Breezy Weather", 120, 188);
  } else {
    // Icon and temperature are laid out as one group and centred together,
    // so the pair stays balanced whether the reading is "-8" or "100".
    const int ICON_S = 46, ICON_GAP = 8;
    int tempW = bigTempWidth(weatherTempC);
    int groupX = 120 - (ICON_S + ICON_GAP + tempW) / 2;
    drawWeatherIcon(groupX, 100 - ICON_S / 2, ICON_S,
                    weatherIconFor(weatherCode, weatherTxt), COL_YELLOW);
    drawBigTempAt(groupX + ICON_S + ICON_GAP, 100, weatherTempC, COL_YELLOW);

    // Condition, with an age suffix once the reading is no longer fresh —
    // this data can be restored from flash across a reboot, so it must not
    // present itself as current when it isn't. Padded on both sides so a
    // longer caption from the previous frame can't leave fragments behind.
    useSharpSmallFont();
    display.setTextColor(COL_YELLOW, COL_BG);
    long age = weatherAgeSec();
    String line = weatherTxt;
    if (age > WEATHER_FRESH_SEC) {
      char ageBuf[16];
      if (age < 3600) snprintf(ageBuf, sizeof(ageBuf), " (%ldm old)", age / 60);
      else snprintf(ageBuf, sizeof(ageBuf), " (%ldh old)", age / 3600);
      line += ageBuf;
    }
    display.drawString("  " + line + "  ", 120, 138);

    // High/Low spelled out at the medium size, one per line — the pair
    // won't fit on a single line at this size, and stacking them leaves
    // clearance above the BLE/battery cluster at y=213.
    useSharpMedFont();
    display.setTextColor(COL_YELLOW, COL_BG);
    char hiBuf[16], loBuf[16];
    snprintf(hiBuf, sizeof(hiBuf), "High: %d", weatherHiC);
    snprintf(loBuf, sizeof(loBuf), "Low: %d", weatherLoC);
    display.drawString(hiBuf, 120, 166);
    display.drawString(loBuf, 120, 190);
  }

  drawBleBattCluster(battPct, COL_YELLOW);
}

// Draws `text` word-wrapped inside a `maxWidth`-px column, centered,
// starting at `y`, one line per `lineHeight` px. Returns nothing — just
// best-effort fits what it can in `maxLines`.
void drawWrapped(const String &text, int y, int maxWidth, int lineHeight, int maxLines) {
  String word, line;
  int lineNum = 0;
  size_t i = 0;
  while (i <= text.length() && lineNum < maxLines) {
    char c = (i < text.length()) ? text[i] : ' ';
    if (c == ' ' || i == text.length()) {
      String candidate = line.length() ? (line + " " + word) : word;
      if (display.textWidth(candidate) > maxWidth && line.length()) {
        display.drawString(line, 120, y + lineNum * lineHeight);
        lineNum++;
        line = word;
      } else {
        line = candidate;
      }
      word = "";
      if (i == text.length()) break;
    } else {
      word += c;
    }
    i++;
  }
  if (lineNum < maxLines && line.length()) {
    display.drawString(line, 120, y + lineNum * lineHeight);
  }
}

// ---------- stopwatch / timer ----------
uint32_t stopwatchElapsedMs() {
  return swAccumMs + (swRunning ? (millis() - swStartMs) : 0);
}

uint32_t timerRemainingMs() {
  if (!timerRunning) return timerRemainMs;
  uint32_t now = millis();
  return (now >= timerEndMs) ? 0 : (timerEndMs - now);
}

void timerLoad(uint8_t presetIdx) {
  timerPresetIdx = presetIdx % TIMER_PRESET_COUNT;
  timerRemainMs = TIMER_PRESETS_SEC[timerPresetIdx] * 1000UL;
  timerRunning = false;
  timerFired = false;
}

void drawStopwatchScreen(int battPct) {
  display.setTextDatum(middle_center);
  useSharpMedFont();
  display.setTextColor(COL_RED, COL_BG);
  display.drawString("Stopwatch", 120, 66);

  uint32_t ms = stopwatchElapsedMs();
  // Font7 has no letters but does carry digits, ':' and '.', which is
  // exactly what a stopwatch readout needs — so it gets the same
  // seven-segment treatment as the clock.
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu:%02lu.%lu",
           (unsigned long)(ms / 60000UL),
           (unsigned long)((ms / 1000UL) % 60UL),
           (unsigned long)((ms / 100UL) % 10UL));
  display.setTextColor(COL_RED, COL_BG);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.75f);
  display.drawString(buf, 120, 118);
  display.setTextSize(1.0f);

  useSharpSmallFont();
  display.setTextColor(COL_RED, COL_BG);
  display.drawString(swRunning ? " tap to stop " : " tap to start ", 120, 162);
  display.drawString(swRunning || stopwatchElapsedMs() == 0 ? "             " : " swipe to clear ", 120, 182);

  drawBleBattCluster(battPct, COL_RED);
}

void drawTimerScreen(int battPct) {
  display.setTextDatum(middle_center);
  useSharpMedFont();
  display.setTextColor(COL_PURPLE, COL_BG);
  display.drawString("Timer", 120, 66);

  uint32_t ms = timerRemainingMs();
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu:%02lu",
           (unsigned long)(ms / 60000UL), (unsigned long)((ms / 1000UL) % 60UL));

  // Amber once it has expired — the one state that wants to be noticed
  // from across a room, and the palette already reserves amber for that.
  display.setTextColor(timerFired ? COL_AMBER : COL_PURPLE, COL_BG);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  display.drawString(buf, 120, 118);
  display.setTextSize(1.0f);

  useSharpSmallFont();
  if (timerFired) {
    display.setTextColor(COL_AMBER, COL_BG);
    display.drawString("  -- TIME UP --  ", 120, 162);
    display.setTextColor(COL_PURPLE, COL_BG);
    display.drawString("  tap to reset   ", 120, 182);
  } else {
    display.setTextColor(COL_PURPLE, COL_BG);
    display.drawString(timerRunning ? "  tap to pause  " : "  tap to start  ", 120, 162);
    display.drawString(timerRunning ? "                " : " swipe: set time ", 120, 182);
  }

  drawBleBattCluster(battPct, COL_PURPLE);
}

void drawNotifScreen(int battPct) {
  display.setTextDatum(middle_center);
  useSharpSmallFont();
  display.setTextColor(COL_BLUE, COL_BG);
  // Header doubles as the position indicator once more than one is held,
  // so it's obvious there are others to scroll to.
  if (notifCount > 1) {
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "NOTIFICATION %u/%u", notifIndex + 1, notifCount);
    display.drawString(hdr, 120, 46);
  } else {
    display.drawString("   NOTIFICATION   ", 120, 46);
  }

  if (!notifCount) {
    display.setTextColor(COL_BLUE, COL_BG);
    display.drawString("-- NONE YET --", 120, 120);
    // The setup hint is only useful before there's a phone connected —
    // once BLE is up, "none yet" just means nothing has arrived, and the
    // instructions are noise.
    if (!bleConnected) {
      display.drawString("pair Gadgetbridge to", 120, 142);
      display.drawString("start receiving these", 120, 162);
    } else {
      // Clear whatever the hint left behind on the previous frame.
      display.drawString("                     ", 120, 142);
      display.drawString("                     ", 120, 162);
    }
  } else {
    const Notif &n = notifs[notifIndex];
    display.setTextColor(COL_BLUE, COL_BG);
    display.drawString(n.src.length() ? n.src : "NOTIFICATION", 120, 70);

    useSharpSmallFont();
    display.setTextColor(COL_BLUE, COL_BG);
    drawWrapped(n.title, 92, 190, 16, 2);

    display.setTextColor(COL_BLUE, COL_BG);
    drawWrapped(n.body, 134, 190, 16, 4);

    // Hint that there's more than one, and which way to move through them.
    if (notifCount > 1) {
      display.setTextColor(COL_BLUE, COL_BG);
      display.drawString(notifIndex > 0 ? "^" : " ", 120, 60);
      display.drawString(notifIndex + 1 < notifCount ? "v" : " ", 120, 198);
    }
  }

  drawBleBattCluster(battPct, COL_BLUE);
}

// Now-playing, fed by Gadgetbridge's "musicinfo"/"musicstate" messages (see
// handleGadgetbridgeLine()). Grouped with NOTIF — same blue accent — since
// both are "things the phone told us" screens. Tapping toggles play/pause
// via sendMusicControl(), which is flagged there as unverified against a
// real Gadgetbridge session; the display half of this screen has no such
// caveat, since it's parsed the same well-established way as weather/notify.
void drawMusicScreen(int battPct) {
  display.setTextDatum(middle_center);

  useSharpMedFont();
  display.setTextColor(COL_BLUE, COL_BG);
  display.drawString("Now Playing", 120, 66);

  if (!hasMusic) {
    display.setTextColor(COL_BLUE, COL_BG);
    display.drawString("-- NOTHING --", 120, 120);
    useSharpSmallFont();
    display.drawString("play something on", 120, 148);
    display.drawString("the phone to see it", 120, 168);
  } else {
    useSharpSmallFont();
    display.setTextColor(COL_BLUE, COL_BG);
    drawWrapped(musicTrack.length() ? musicTrack : "(unknown track)", 100, 200, 18, 2);
    display.setTextColor(COL_BLUE, COL_BG);
    drawWrapped(musicArtist, 140, 200, 18, 1);

    // Play/pause glyph, drawn from primitives the same way the weather
    // icons are — a filled triangle for "playing", two bars for "paused".
    display.setTextColor(COL_BLUE, COL_BG);
    int cx = 120, cy = 178, s = 16;
    if (musicPlaying) {
      display.fillTriangle(cx - s / 2, cy - s / 2, cx - s / 2, cy + s / 2, cx + s / 2, cy, COL_BLUE);
    } else {
      display.fillRect(cx - s / 2, cy - s / 2, s / 3, s, COL_BLUE);
      display.fillRect(cx + s / 6, cy - s / 2, s / 3, s, COL_BLUE);
    }

    display.setTextColor(COL_GREEN_DIM, COL_BG);
    display.drawString("tap: play / pause", 120, 198);
  }

  drawBleBattCluster(battPct, COL_BLUE);
}

// Vault Boy, traced from a real reference photo down to a 1bpp bitmap (see
// vaultboy_bitmap.h) rather than freehand-drawn — this is what actually
// gets the proportions and pose right. Rendered as a green silhouette with
// the linework (face, collar, sleeve seams) punched out in black, matching
// the rest of the UI's monochrome CRT look.
void drawVaultBoy(int x, int y, uint8_t brightness) {
  // Scale the green dimmer during the flicker-in for a power-on feel.
  uint16_t flickerGreen = lgfx::color565((uint8_t)(85 * brightness / 255), (uint8_t)(255 * brightness / 255), (uint8_t)(110 * brightness / 255));
  display.drawBitmap(x, y, VAULTBOY_BITMAP, VAULTBOY_W, VAULTBOY_H, flickerGreen, COL_BG);
}

// ---------- boot terminal ----------
// Boot now reads as a scrolling terminal log rather than a static splash:
// each real step reports itself as it happens, so the screen shows what the
// watch is genuinely doing (and, when something fails, where it stopped).
//
// Lines live in the vertical band where the round panel is widest. At the
// very top and bottom of a 240px circle the usable width collapses, so text
// placed there gets physically cut off by the bezel even though it's inside
// the framebuffer.
static const uint8_t BOOT_LOG_MAX = 8;
static const int BOOT_LOG_X = 26, BOOT_LOG_Y0 = 66, BOOT_LOG_LINE_H = 16;
String bootLines[BOOT_LOG_MAX];
uint8_t bootLineCount = 0;

void bootLog(const String &msg) {
  Serial.printf("[esp32-watch] %s\n", msg.c_str());

  if (bootLineCount == BOOT_LOG_MAX) {
    for (uint8_t i = 1; i < BOOT_LOG_MAX; i++) bootLines[i - 1] = bootLines[i];
    bootLineCount--;
  }
  bootLines[bootLineCount++] = msg;

  display.startWrite();
  display.fillScreen(COL_BG);
  display.setTextDatum(top_left);
  useSharpSmallFont();
  for (uint8_t i = 0; i < bootLineCount; i++) {
    // Older lines fade back, so the eye lands on the newest one — the CRT
    // equivalent of scrollback falling away.
    display.setTextColor(i + 1 == bootLineCount ? COL_GREEN : COL_GREEN_DIM, COL_BG);
    display.drawString(bootLines[i], BOOT_LOG_X, BOOT_LOG_Y0 + i * BOOT_LOG_LINE_H);
  }
  display.endWrite();
  display.setTextDatum(middle_center);

  // Long enough to actually read as it goes by. The whole sequence is under
  // two seconds, and the slow parts of boot (WiFi, NTP) dwarf it anyway.
  delay(110);
}

// Vault Boy still gets his moment, after the log and before the watch face.
void playBootAnimation() {
  display.fillScreen(COL_BG);
  display.setTextDatum(middle_center);

  // Centered on the round face with room top and bottom to clear the
  // glass — the framebuffer is square but the visible screen is a circle,
  // so content placed too near the top/bottom corners gets physically cut
  // off even though it's "on screen" in the buffer.
  int bx = (240 - VAULTBOY_W) / 2;
  int by = 6;

  // CRT power-on flicker: a couple of quick dim/bright pulses before it
  // settles, instead of a fabricated second "eyes open" frame.
  uint8_t flickerSteps[] = {60, 20, 140, 60, 255};
  for (uint8_t b : flickerSteps) {
    display.startWrite();
    drawVaultBoy(bx, by, b);
    display.endWrite();
    delay(90);
  }
  delay(400);
}

// Solid full-brightness fill — see SCREEN_FLASHLIGHT. No BLE/battery
// cluster drawn over it (render() skips the cluster for this screen): the
// point is raw light output, and it would only cut into that.
void drawFlashlightScreen() {
  display.fillScreen(COL_WHITE);
}

void render(bool fullClear) {
  display.startWrite();
  if (fullClear) {
    display.fillScreen(COL_BG);
  }

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  int steps = stepCounter.count();
  int battPct = readBatteryPercent();

  if (currentScreen == SCREEN_STAT) drawStatScreen(t, steps, battPct);
  else if (currentScreen == SCREEN_DATA) drawDataScreen(steps, battPct);
  else if (currentScreen == SCREEN_HISTORY) drawHistoryScreen(battPct);
  else if (currentScreen == SCREEN_WEATHER) drawWeatherScreen(battPct);
  else if (currentScreen == SCREEN_NOTIF) drawNotifScreen(battPct);
  else if (currentScreen == SCREEN_MUSIC) drawMusicScreen(battPct);
  else if (currentScreen == SCREEN_STOPWATCH) drawStopwatchScreen(battPct);
  else if (currentScreen == SCREEN_TIMER) drawTimerScreen(battPct);
  else drawFlashlightScreen(); // SCREEN_FLASHLIGHT

  display.endWrite();
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[esp32-watch] boot");

  // Deep sleep (enterDeepSleep(), triggered by a long-press on the DATA
  // screen — this board's only "power button") is a full reset: everything
  // except the RTC timer and flash-backed state loses power, and setup()
  // is the only way back. Detect that case so wake can skip the slow parts
  // (flicker animation, WiFi+NTP round trip) instead of paying full boot
  // cost every time you just want to glance at the time.
  bool wokeFromSleep = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);
  if (wokeFromSleep) Serial.println("[esp32-watch] waking from deep sleep (touch)...");

  // Load the DST-aware timezone rule unconditionally, independent of
  // WiFi — BLE time sync (from Gadgetbridge, see handleGadgetbridgeLine)
  // can now set the clock without ever touching WiFi, and it needs this
  // rule in place to convert the UTC it receives into correct local time.
  setenv("TZ", POSIX_TZ, 1);
  tzset();

  // This app is nowhere near CPU-bound (a clock face + light I2C polling),
  // so run at the lowest clock WiFi still tolerates reliably instead of
  // the default 240MHz — meaningfully less power for identical behavior.
  // This is the floor either way; setupPowerManagement() below tries to do
  // even better (dynamic scaling + light sleep) on top of it, when the core
  // build actually supports it.
  setCpuFrequencyMhz(80);
  setupPowerManagement();

  Serial.println("[esp32-watch] display.init()...");
  bool dispOk = display.init();
  Serial.printf("[esp32-watch] display.init() -> %d\n", dispOk);
  display.setRotation(1); // 90° clockwise
  display.setBrightness(BRIGHT_ACTIVE);
  lastActivityMs = millis();
  nextFlickerMs = lastActivityMs + 4000 + random(4000); // first CRT flicker blip, not an instant one

  // The boot terminal is skipped entirely on a sleep wake — there it would
  // just be a delay between "touched the watch" and "screen shows
  // something", which is the opposite of what waking should feel like.
  display.startWrite();
  display.fillScreen(COL_BG);
  display.endWrite();
  if (!wokeFromSleep) {
    bootLog("> ESP32-WATCH");
    bootLog("> RobCo Industries");
    bootLog(dispOk ? "> display....OK" : "> display..FAIL");
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  // (WiFi/NTP sync happens further down, after touch+IMU are probed, so
  //  the ~15s WiFi wait doesn't hide sensor problems behind a blank pause.)
  Wire.setClock(400000);
  Wire.setTimeOut(50); // ms — never let a stuck bus hang setup()

  if (!wokeFromSleep) bootLog("> scanning i2c...");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[esp32-watch]   found device at 0x%02X\n", addr);
      if (!wokeFromSleep) {
        char line[24];
        snprintf(line, sizeof(line), "  found 0x%02X", addr);
        bootLog(line);
      }
    }
  }

  for (int attempt = 1; attempt <= 4 && !touchOk; attempt++) {
    touchOk = touch.begin(Wire, PIN_TOUCH_RST, PIN_TOUCH_INT);
    Serial.printf("[esp32-watch] touch.begin() attempt %d -> %d\n", attempt, touchOk);
    if (!touchOk) delay(150);
  }
  if (!wokeFromSleep) bootLog(touchOk ? "> touch......OK" : "> touch....FAIL");

  Serial.println("[esp32-watch] probing IMU...");
  imuOk = imu.begin(Wire, QMI8658_ADDRESS_HIGH) || imu.begin(Wire, QMI8658_ADDRESS_LOW);
  Serial.printf("[esp32-watch] imu.begin() -> %d\n", imuOk);
  if (imuOk) {
    imu.enableAccel(true);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_4G);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setAccelUnit_mps2(true);

    // Off by default (see GYRO_ENABLED) — nothing reads gyroX/Y/Z yet, and
    // a running gyro costs real current for zero current benefit. 512dps
    // covers a sharp wrist flick with headroom, for whenever a gesture/
    // orientation feature (rejecting wrist-twist from step counts,
    // raise-to-wake, auto-flip for the other wrist) actually wants it.
    if (GYRO_ENABLED) {
      imu.enableGyro(true);
      imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
      imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
      imu.setGyroUnit_dps(true);
    }
  }
  if (!wokeFromSleep) bootLog(imuOk ? "> imu........OK" : "> imu......FAIL");

  loadState();
  timerLoad(timerPresetIdx); // arm the default duration so the screen has something to show

  // Critical-battery check, before spending any current on WiFi, BLE, or
  // (on a sleep wake) the Vault Boy animation — none of that is worth
  // running at this state of charge, and skipping it here is what actually
  // keeps the cell from ever reaching the danger zone this board has
  // needed a physical reconnect to recover from once already. A reading
  // outside the plausible range is BATT_UNKNOWN, not "critical" — that
  // could just mean no cell is installed (e.g. bench-testing on USB alone),
  // which isn't a low-battery situation at all.
  int battMv = readBatteryMilliVolts();
  if (battMv >= BATT_MIN_PLAUSIBLE_MV && battMv <= BATT_CRITICAL_MV) {
    enterCriticalBatterySleep(battMv); // never returns
  }

  if (!wokeFromSleep) bootLog("> bluetooth up");
  setupBLE();

  bool wifiOk;
  if (wokeFromSleep) {
    // The RTC domain (and the calendar clock it drives, via time()) stays
    // powered through deep sleep — only regular RAM resets, which is why
    // timeSynced itself comes back false here even though the underlying
    // clock is still correct.
    timeSynced = (time(nullptr) > 1700000000);

    // Resync by real elapsed wall-clock time, not the millis()-based timer
    // loop() uses while continuously awake — millis() resets to 0 on every
    // wake, so a watch that gets napped often would otherwise never
    // actually hit a 4h-of-millis() threshold and drift uncorrected
    // indefinitely. This still skips WiFi entirely on the common case (a
    // quick glance well within the interval), which is the whole point of
    // the fast-wake path — it just no longer skips it *forever*.
    if (wifiResyncDue(WAKE_RESYNC_INTERVAL_SEC)) {
      Serial.println("[esp32-watch] wake: resync due, trying a quick WiFi reconnect...");
      rtcLastSyncAttemptEpoch = time(nullptr); // stamp before attempting — throttles retries even on failure
      bool resynced = syncTimeViaWifi(8000);   // saved creds only, no portal — same as loop()'s periodic resync
      wifiOk = resynced || timeSynced;         // a failed attempt doesn't invalidate a clock that was already fine
    } else {
      wifiOk = timeSynced;
      Serial.println("[esp32-watch] wake: recently synced, skipping WiFi.");
    }

    // Vault Boy on every wake now, not just cold boot — costs the ~0.85s
    // playBootAnimation() takes (five flicker steps plus a settle delay),
    // which is a real, deliberate cost against the fast-wake path's whole
    // point, but that's the trade asked for: waking should feel like the
    // watch turning on, not just the screen un-dimming. The scanning-log
    // lines above this stay cold-boot-only, though — those exist to narrate
    // WiFi/NTP/sensor probing, which this path mostly skips anyway.
    playBootAnimation();
  } else {
    bootLog("> wifi connect...");
    wifiOk = wifiConnectOrSetup();
    rtcLastSyncAttemptEpoch = time(nullptr);

    // wifiConnectOrSetup() may have taken over the whole screen for the
    // setup portal, so the log is re-rendered from scratch by the next
    // bootLog() rather than assumed to still be on the panel.
    bootLog(wifiOk ? "> ntp sync...OK" : "> ntp sync.FAIL");
    bootLog("> READY");
    delay(350);

    // Vault Boy closes the sequence, after the log and before the watch
    // face — the terminal says what happened, he says whose watch it is.
    playBootAnimation();
  }

  // Skip the settle delay on a sleep wake — the whole point of that path is
  // getting back to something useful on screen as fast as possible.
  if (!wokeFromSleep) delay(200);
  Serial.println("[esp32-watch] ESP32-WATCH online.");
}

void loop() {
  uint32_t now = millis();

  // Poll the IMU at ~50Hz and feed the pedometer (and, while dimmed,
  // raise-to-wake).
  if (imuOk && now - lastImuPollMs >= 20) {
    lastImuPollMs = now;
    float ax, ay, az;
    bool accelOk = imu.readAccelMPS2(ax, ay, az);
    if (accelOk) {
      stepCounter.update(ax, ay, az);
    }
    if (GYRO_ENABLED) {
      imu.readGyro(gyroX, gyroY, gyroZ); // dps, per setGyroUnit_dps() above
      // Only worth checking while there's actually something to wake —
      // skips the extra math the rest of the time, and sidesteps the whole
      // "what if a burst spans a wake that happened some other way" question.
      if (backlightDimmed) updateRaiseToWake(accelOk, ax, ay, az, now);

      // Temporary calibration aid (IMU_TRACE): raise-to-wake's thresholds
      // and its idea of "screen facing you" were guesses about how this
      // IMU is mounted, and guessing the sign of one axis wrong is enough
      // to stop it ever firing. Logging the vectors periodically is what
      // turns that into a measurement. Turn off once calibrated.
      if (IMU_TRACE && now - lastImuTraceMs >= 1000) {
        lastImuTraceMs = now;
        Serial.printf("[esp32-watch] imu a=(%.1f,%.1f,%.1f) g=(%.0f,%.0f,%.0f) dim=%d\n",
                      ax, ay, az, gyroX, gyroY, gyroZ, backlightDimmed);
      }
    }
  }

  // Day rollover: once we know real wall-clock time, reset the step count
  // the first time we notice the calendar day has changed. Checked every
  // pass, so a watch booting into a new day resets as soon as its clock is
  // valid, not only if it happened to be awake at midnight.
  //
  // The marker is a local YYYYMMDD, not the old time()/86400. That was a
  // count of UTC days, so the counter rolled over at UTC midnight — 8pm
  // local here — rather than at the local midnight anyone would expect.
  if (timeSynced) {
    time_t nowEpoch = time(nullptr);
    struct tm lt;
    localtime_r(&nowEpoch, &lt);
    uint32_t today = (uint32_t)((lt.tm_year + 1900) * 10000u + (lt.tm_mon + 1) * 100u + lt.tm_mday);
    if (lastDayMarker != 0 && today != lastDayMarker) {
      uint32_t finished = stepCounter.count(); // the day that's ending, before it's wiped
      for (uint8_t i = 0; i + 1 < 6; i++) stepHist[i] = stepHist[i + 1]; // drop the oldest
      stepHist[5] = finished; // most recently completed day, for the HISTORY screen
      Serial.printf("[esp32-watch] new day (%lu -> %lu), resetting steps (yesterday: %lu)\n",
                    (unsigned long)lastDayMarker, (unsigned long)today, (unsigned long)finished);
      stepCounter.set(0);
      lastDayMarker = today;
      saveState(); // persist immediately so a reboot can't resurrect yesterday's count
    }
    lastDayMarker = today;
  }

  // Touch: swipe left/right cycles screens (right = next, left = previous);
  // a touch that starts while the backlight is dimmed just wakes it
  // instead. Any touch resets the idle-dim timer.
  //
  // Debounced against a marginal/noisy connection: a glitchy I2C read
  // occasionally comes back looking like "finger down" with nothing
  // touching it. We reject any read with out-of-range coordinates
  // outright (a strong sign of a corrupted byte), and otherwise only
  // trust "down" once we've seen it on 3 polls in a row, ~30ms apart.
  if (touchOk && now - lastTouchPollMs >= 30) {
    lastTouchPollMs = now;
    int16_t tx, ty;
    bool rawDown = touch.read(tx, ty);
    if (rawDown && (tx < 0 || tx > 239 || ty < 0 || ty > 239)) rawDown = false;
    touchDownStreak = rawDown ? min((int)touchDownStreak + 1, 5) : 0;
    bool stableDown = touchDownStreak >= 3;

    // Waking the screen deliberately bypasses the 3-poll debounce that
    // gates *navigation*. That debounce exists so a noisy read can't
    // trigger a screen change, but the cost of a spurious wake is only a
    // brief backlight blip — whereas requiring ~90ms of sustained contact
    // makes a quick tap feel like it did nothing. (Deliberately not using
    // the INT pin as a second signal here: if it ever idles low, this
    // would re-wake every poll and the screen would never dim again —
    // a worse failure than a slightly less sensitive wake.)
    if (backlightDimmed && rawDown) {
      wakeScreen(now);
      // The finger that woke the screen must not also navigate. This has
      // to outlive wakeScreen() clearing backlightDimmed, since the
      // gesture-start branch below derives touchWasDimWake from that flag
      // — so it's tracked separately and held until the finger lifts.
      wakeTouchPending = true;
    }
    if (!rawDown && touchDownStreak == 0) wakeTouchPending = false;

    if (stableDown && !prevTouchDown) {
      // Gesture just started.
      touchStartX = touchLastX = tx;
      touchStartY = touchLastY = ty;
      touchWasDimWake = backlightDimmed || wakeTouchPending;
      touchDownStartMs = now;
      if (backlightDimmed) {
        wakeScreen(now);
      }
      lastActivityMs = now; // reset the idle timer on every touch, dimmed or not
    } else if (stableDown && prevTouchDown) {
      // Gesture in progress — track the latest position for when it ends.
      touchLastX = tx;
      touchLastY = ty;
      lastActivityMs = now;

    } else if (!stableDown && prevTouchDown) {
      // Finger lifted — this is where every gesture is resolved.
      {
        int16_t rawDx = touchLastX - touchStartX;
        int16_t rawDy = touchLastY - touchStartY;
        int16_t visualSwipe = SWIPE_AXIS_IS_RAW_Y ? rawDy : rawDx;
        if (SWIPE_INVERT) visualSwipe = -visualSwipe;
        uint32_t heldMs = now - touchDownStartMs;
        bool heldStill = abs(rawDx) < 15 && abs(rawDy) < 15;

        // The axis perpendicular to the screen-changing swipe is free, so
        // it scrolls the notification list. Whichever axis moved further
        // decides which gesture this was, so a slightly diagonal swipe
        // still does the one it looked most like.
        int16_t visualScroll = SWIPE_AXIS_IS_RAW_Y ? rawDx : rawDy;
        if (VSWIPE_INVERT) visualScroll = -visualScroll;

        Serial.printf("[esp32-watch] lift: held=%lums dx=%d dy=%d screen=%u dimWake=%d\n",
                      (unsigned long)heldMs, rawDx, rawDy, currentScreen, touchWasDimWake);

        bool scrollGesture = !touchWasDimWake && abs(visualScroll) >= SWIPE_MIN_DIST &&
                             abs(visualScroll) > abs(visualSwipe);
        bool tapGesture = !touchWasDimWake && heldStill && heldMs < TAP_MAX_MS;

        if (scrollGesture && currentScreen == SCREEN_NOTIF && notifCount > 1) {
          if (visualScroll > 0 && notifIndex + 1 < notifCount) notifIndex++;
          else if (visualScroll < 0 && notifIndex > 0) notifIndex--;
          notifAutoShown = false; // user's driving now, stop the auto-revert timer
          forceRedraw = true;

        } else if (scrollGesture && currentScreen == SCREEN_STOPWATCH && !swRunning) {
          // Clearing only while stopped: wiping a running stopwatch with a
          // stray swipe would be a genuinely annoying way to lose a timing.
          swAccumMs = 0;
          forceRedraw = true;

        } else if (scrollGesture && currentScreen == SCREEN_TIMER && !timerRunning) {
          // Cycles the preset list. Only while stopped, for the same reason.
          timerLoad(visualScroll > 0 ? timerPresetIdx + 1
                                     : timerPresetIdx + TIMER_PRESET_COUNT - 1);
          forceRedraw = true;

        } else if (tapGesture && currentScreen == SCREEN_STOPWATCH) {
          if (swRunning) {
            swAccumMs += millis() - swStartMs; // bank the running segment
            swRunning = false;
          } else {
            swStartMs = millis();
            swRunning = true;
          }
          forceRedraw = true;

        } else if (tapGesture && currentScreen == SCREEN_TIMER) {
          if (timerFired) {
            timerLoad(timerPresetIdx); // acknowledge and rearm at the same duration
          } else if (timerRunning) {
            timerRemainMs = timerRemainingMs(); // freeze what's left
            timerRunning = false;
          } else if (timerRemainMs > 0) {
            timerEndMs = millis() + timerRemainMs;
            timerRunning = true;
          }
          forceRedraw = true;

        } else if (tapGesture && currentScreen == SCREEN_DATA) {
          // Four deliberate taps, each within STEPS_RESET_ARM_MS of the last,
          // before a day's step count is wiped. Nothing else on the watch is
          // destructive, and the count can't be recovered once gone, so the
          // gesture is made hard to perform by accident rather than merely
          // confirmed once.
          if (stepsResetArmedMs && now - stepsResetArmedMs < STEPS_RESET_ARM_MS) {
            stepsResetTaps++;
          } else {
            stepsResetTaps = 1; // stale run — start counting again
          }
          stepsResetArmedMs = now;

          if (stepsResetTaps >= STEPS_RESET_TAPS) {
            stepCounter.set(0);
            saveState();
            stepsResetArmedMs = 0;
            stepsResetTaps = 0;
            Serial.println("[esp32-watch] steps reset by user.");
          }
          forceRedraw = true;

        } else if (tapGesture && currentScreen == SCREEN_MUSIC && hasMusic) {
          sendMusicControl(musicPlaying ? "pause" : "play");
          // Flip the local flag optimistically rather than waiting for a
          // musicstate echo back — Gadgetbridge doesn't reliably send one
          // for a command the watch itself issued, and a button that
          // doesn't visibly respond to its own tap reads as broken even
          // when the phone-side command worked fine.
          musicPlaying = !musicPlaying;
          forceRedraw = true;

        } else

        // A touch that woke the dimmed screen is barred from *navigating*
        // — you shouldn't land on a different screen just for looking at
        // the watch. It is NOT barred from long-press actions: holding
        // still for three full seconds is unambiguous intent, and gating
        // those on the screen already being lit made them nearly
        // impossible to trigger in practice, since the backlight dims
        // after 8s and almost every real press therefore starts dimmed.
        if (abs(visualSwipe) >= SWIPE_MIN_DIST && !touchWasDimWake) {
          if (visualSwipe > 0) currentScreen = (currentScreen + 1) % NUM_SCREENS;
          else currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
          notifAutoShown = false; // user's driving now, stop the auto-revert timer
          forceRedraw = true;

        } else if (heldStill && heldMs >= LONG_PRESS_MS && heldMs <= LONG_PRESS_MAX_MS) {
          // Duration, not screen, selects the action. Keying them off which
          // screen you were on made this unpredictable in practice: waking
          // from deep sleep is a full reboot, which lands you back on STAT,
          // so the natural "hold again to sleep again" landed on the portal
          // instead and looked like sleep was broken.
          //   ~3-6s  -> deep sleep (this board's only "power button")
          //   ~6-12s -> reopen the WiFi setup portal
          // Long-press actions fire on *release*, not while still held.
          // Firing mid-hold meant anything resting against the screen — the
          // watch set face down, a sleeve, a cable pressing on it — sailed
          // past the threshold and silently opened the WiFi portal
          // (observed doing exactly that in a serial log, which also knocks
          // Gadgetbridge offline for as long as the portal is up). A
          // resting object never lifts, so release-triggering can't be
          // fooled by one; the upper bound covers it eventually being
          // removed.
          Serial.printf("[esp32-watch] long-press released after %lums on screen %u\n",
                        (unsigned long)heldMs, currentScreen);
          if (heldMs < SLEEP_PRESS_MAX_MS) {
            enterDeepSleep(); // never returns — next code that runs is setup()
          } else {
            forceWifiSetupPortal();
            // Minutes may have passed inside that blocking call — the touch
            // chip's idea of "finger down" is stale, so start clean rather
            // than risk misreading whatever it reports on the next poll.
            prevTouchDown = false;
            touchDownStreak = 0;
            lastActivityMs = millis();
            return;
          }
        }
      }
    }
    prevTouchDown = stableDown;
  }

  // A notification that popped itself up automatically reverts to the
  // clock face after a few seconds, unless the user has since taken over
  // navigation (handled above by clearing notifAutoShown on manual taps).
  if (notifAutoShown && now > notifAutoRevertMs) {
    notifAutoShown = false;
    currentScreen = 0;
    forceRedraw = true;
  }

  // Flashlight: full brightness for as long as that screen is up, restored
  // to whatever brightness applied before (active or already-dimmed) the
  // moment you swipe away. Tracked by transition rather than just "if
  // showing, set 255 every loop" so it doesn't fight the dim/flicker logic
  // below for control of the backlight while it's not actually in use.
  bool isFlashlightNow = (currentScreen == SCREEN_FLASHLIGHT);
  if (isFlashlightNow && !wasFlashlightScreen) {
    display.setBrightness(255); // brighter than the normal BRIGHT_ACTIVE ceiling — this is a torch
  } else if (!isFlashlightNow && wasFlashlightScreen) {
    display.setBrightness(backlightDimmed ? BRIGHT_DIM : BRIGHT_ACTIVE);
  }
  wasFlashlightScreen = isFlashlightNow;

  // Dim the backlight after a few seconds of no touch — it's the single
  // biggest power draw on this board, and a watch face doesn't need to
  // stay at full brightness when nobody's looking at it. Held off on the
  // flashlight screen for the obvious reason.
  if (touchOk && !backlightDimmed && !isFlashlightNow && now - lastActivityMs > IDLE_DIM_TIMEOUT_MS) {
    display.setBrightness(BRIGHT_DIM);
    backlightDimmed = true;
  }

  // A faint CRT power-flicker blip, purely cosmetic: a brief brightness dip
  // and recovery at a random interval, while there's an active, undimmed
  // screen to see it on. Skipped on the flashlight screen for the same
  // reason the idle-dim is — and skipped while dimmed both because it'd be
  // barely visible at BRIGHT_DIM and because there'd be nothing watching it
  // happen anyway.
  if (!backlightDimmed && !isFlashlightNow && now >= nextFlickerMs) {
    display.setBrightness(BRIGHT_ACTIVE > 40 ? BRIGHT_ACTIVE - 40 : BRIGHT_ACTIVE / 2);
    delay(30);
    display.setBrightness(BRIGHT_ACTIVE);
    nextFlickerMs = now + 4000 + random(4000); // next one due 4-8s from now
  }

  // Sleep the whole watch after a long stretch untouched. Held off while a
  // stopwatch or timer is counting: deep sleep is a full reset, so sleeping
  // through a running timer would silently destroy it and the alarm would
  // never fire — the one case where "nobody has touched it" plainly does
  // not mean "nothing is happening". Held off on the flashlight screen too:
  // falling asleep mid-use would be a strange way for a torch to behave.
  if (touchOk && AUTO_SLEEP_TIMEOUT_MS && !swRunning && !timerRunning && !isFlashlightNow &&
      now - lastActivityMs > AUTO_SLEEP_TIMEOUT_MS) {
    Serial.println("[esp32-watch] idle timeout -> deep sleep");
    enterDeepSleep(); // never returns
  }

  // Keep the BLE "steps" characteristic fresh for anyone who reads it.
  static uint32_t lastStepsPublish = 0;
  if (now - lastStepsPublish > 2000) {
    lastStepsPublish = now;
    if (stepsChar) {
      char buf[12];
      snprintf(buf, sizeof(buf), "%lu", (unsigned long)stepCounter.count());
      stepsChar->setValue((uint8_t *)buf, strlen(buf));
    }
  }

  // A clock only needs to redraw when the second (or screen/state) changes —
  // keeps things flicker-free without needing a second framebuffer.
  //
  // This deliberately keeps redrawing while the backlight is dimmed. An
  // earlier version skipped those frames to save power, on the assumption
  // that a dimmed screen is effectively invisible. It isn't: BRIGHT_DIM is
  // low but perfectly readable, so skipping froze the clock mid-tick and
  // stalled the step count until something happened to wake the watch. A
  // watch face that lies about the time to save a little current is a bad
  // trade.
  time_t nowSec = time(nullptr);
  struct tm t;
  localtime_r(&nowSec, &t);
  // A running stopwatch shows tenths, so it needs redrawing faster than the
  // once-a-second everything else settles for — but only while its own
  // screen is actually up.
  bool fastTick = (currentScreen == SCREEN_STOPWATCH && swRunning &&
                   now - lastFastDrawMs >= 100);
  if (forceRedraw || t.tm_sec != lastRenderedSecond || fastTick) {
    render(forceRedraw);
    lastRenderedSecond = t.tm_sec;
    lastFastDrawMs = now;
    forceRedraw = false;
  }

  // Countdown expiry is checked regardless of which screen is showing, so a
  // timer set and then navigated away from still goes off.
  if (timerRunning && timerRemainingMs() == 0) {
    timerRunning = false;
    timerRemainMs = 0;
    timerFired = true;
    timerFiredMs = now;
    currentScreen = SCREEN_TIMER;
    wakeScreen(now);
    Serial.println("[esp32-watch] timer expired.");
  }

  // The armed step-reset prompt expires on its own; repaint once when it
  // does so the prompt doesn't sit there looking still-armed.
  if (stepsResetArmedMs && now - stepsResetArmedMs >= STEPS_RESET_ARM_MS) {
    stepsResetArmedMs = 0;
    stepsResetTaps = 0;
    forceRedraw = true;
  }

  // Persist steps every 30s so a reboot/charge doesn't lose today's count.
  if (now - lastSaveMs > 30000) {
    lastSaveMs = now;
    saveState();
  }

  // Periodic re-sync in case the software clock drifts over a long day.
  // Same wall-clock-based due check the wake-from-sleep path in setup()
  // uses (see wifiResyncDue()), so the two share one throttle regardless of
  // how many sleep/wake cycles happen in between.
  if (wifiResyncDue()) {
    rtcLastSyncAttemptEpoch = time(nullptr);
    syncTimeViaWifi(10000);
  }

  // Yield instead of spinning flat-out — lets the idle task (and any
  // automatic modem/light sleep underneath it) actually get scheduled.
  delay(5);
}
