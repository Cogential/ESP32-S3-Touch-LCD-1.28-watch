// PIP-BOY 3000 watch face for the Waveshare ESP32-S3-Touch-LCD-1.28
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
//   a "PipBoy-Setup" WiFi hotspot + captive portal to pick a network the
//   first time (or whenever) it can't connect — long-press-and-hold on the
//   STAT screen reopens it on demand, e.g. to switch onto a phone's hotspot.
// - BLE peripheral "PIP-BOY 3000": bondable/pairable from Android's
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

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>

#include "LGFX_Config.hpp"
#include "Touch_CST816S.h"
#include "StepCounter.h"
#include "vaultboy_bitmap.h"
#include "nav_icons.h"

// ---------- WiFi / time ----------
// No hardcoded SSID/password anymore — WiFiManager owns WiFi credentials
// (stored in its own NVS blob, separate from Preferences). First boot with
// nothing saved (or a failed connect) pops a "PipBoy-Setup" WiFi AP + captive
// portal to pick a network, same flow as most consumer IoT gadgets. A
// long-press on the STAT screen re-opens that portal on demand, e.g. to
// switch onto your phone's hotspot. See wifiConnectOrSetup() /
// forceWifiSetupPortal() below.
static const char *WIFI_SETUP_AP_NAME = "PipBoy-Setup";

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
static const uint32_t IDLE_DIM_TIMEOUT_MS = 20000; // default; adjustable at runtime via cfgDimTimeoutMs below

// ---------- user settings ----------
// Everything here was a compile-time constant until the settings screen
// existed; the constants above are now just the defaults these start from.
// Persisted in the same NVS namespace as steps/weather — see loadState()
// and saveSettings().
uint8_t cfgBrightness = BRIGHT_ACTIVE;
uint32_t cfgDimTimeoutMs = IDLE_DIM_TIMEOUT_MS;
bool cfgScanlines = true;
bool cfgTime24h = false;
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

// ---------- touch coordinate remap ----------
// The display is rotated 90° in software (setRotation(1) in setup()), but
// the touch chip (Touch_CST816S.h) reports raw, unrotated panel
// coordinates. pipboy_watch (the hand-drawn sibling of this sketch) never
// needed to fix that up: it only ever read swipe *deltas* and tap/hold
// *duration*, never an absolute tap position, so a raw/rotated mismatch
// was invisible to it. LVGL isn't so forgiving — every widget does real
// hit-testing against absolute coordinates, so a touch has to land where
// it visually looks like it landed.
//
// This mapping started out *derived*, not independently re-measured:
// pipboy_watch's own swipe-axis constants (SWIPE_AXIS_IS_RAW_Y=true,
// SWIPE_INVERT=false, VSWIPE_INVERT=false — all confirmed against real
// hardware) say "raw Y drives visual horizontal, raw X drives visual
// vertical, neither flipped" — a plain (x,y) -> (y,x) transpose, no
// inversions. That held for the X axis, but not Y: a live capture of the
// MUSIC screen's prev/play/next buttons (all at the same visual row, near
// the bottom) showed every tap landing at logical Y~50-70 instead of the
// buttons' actual ~174 — every press missing every button, landing on the
// plain screen background instead ("none of the buttons work"). X matched
// the button columns correctly the whole time; only Y needed the flip.
// Plausible reason SWIPE_INVERT and VSWIPE_INVERT weren't equally reliable
// guides: pipboy_watch's own real-world testing likely exercised
// horizontal (screen-to-screen) swipes far more than the vertical ones
// (NOTIF scroll, stopwatch clear), so VSWIPE_INVERT may never have been as
// thoroughly confirmed as SWIPE_INVERT was.
static const bool TOUCH_INVERT_X = false;
static const bool TOUCH_INVERT_Y = true; // confirmed via live capture, 2026-09-20 — see above

static const int16_t SWIPE_MIN_DIST = 35; // px of movement LVGL treats as a swipe, not a tap (lv_indev)

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

// ---------- Pip-Boy palette ----------
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
static const uint16_t COL_ORANGE    = lgfx::color565(255, 150, 50);   // steps (and step history)
static const uint16_t COL_SCANLINE  = lgfx::color565(0, 30, 10);      // faint CRT texture

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
const uint8_t SCREEN_STAT = 0;       // clock, date, weather line, notification banner — has direct-nav buttons to every other screen
const uint8_t SCREEN_DATA = 1;       // step count
const uint8_t SCREEN_HISTORY = 2;    // 7-day step history bar chart — grouped next to DATA
const uint8_t SCREEN_WEATHER = 3;    // full weather
const uint8_t SCREEN_NOTIF = 4;      // notification list
const uint8_t SCREEN_MUSIC = 5;      // now-playing, from Gadgetbridge — grouped next to NOTIF
const uint8_t SCREEN_STOPWATCH = 6;
const uint8_t SCREEN_SETTINGS = 7;   // reached by swiping down on STAT, deliberately not in the nav ring
const uint8_t NUM_SCREENS = 8;
// TIMER and FLASHLIGHT removed by request — see git history (or
// pipboy_watch.ino, which still has both) if either needs reviving.
// Nothing persists currentScreen across a boot (it always starts back at
// SCREEN_STAT — see wokeFromSleep in setup()), so renumbering these between
// firmware versions is safe: no stored value anywhere depends on the
// specific numbers.

// ---------- stopwatch ----------
// Keeps running while you're on other screens; it's only *displayed* by
// its own screen. Elapsed time is derived from millis() rather than
// counted up in the loop, so it can't drift with however often the loop
// happens to run.
bool swRunning = false;
uint32_t swStartMs = 0;   // millis() at the last start
uint32_t swAccumMs = 0;   // time banked by previous start/stop cycles

// The old hand-rolled touch debounce/gesture-tracking globals (raw poll
// state, gesture-start position, etc.) are gone — LVGL's input device
// layer owns all of that now (debouncing, tap vs. swipe vs. long-press
// classification). See lvglTouchRead() and the per-widget event callbacks
// near setup().
bool forceRedraw = true; // still means "refresh visible dynamic content now" — consumed by refreshActiveScreen()
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
// Duration/position for the MUSIC screen's progress ring. Gadgetbridge
// doesn't push position every second — it arrives occasionally in
// "musicstate" — so the displayed position is extrapolated forward from
// the last known value using millis() while playing, the same "derive
// from a timestamp, don't count up in the loop" approach the stopwatch and
// timer already use elsewhere in this file.
long musicDurSec = 0;
long musicPositionSec = 0;
uint32_t musicPositionStampMs = 0;

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
int lastRenderedSecond = -1;

// ---------- raise-to-wake state ----------
bool raiseGyroActive = false;   // currently inside a rotation burst above RAISE_GYRO_THRESHOLD_DPS
uint32_t raiseGyroStartMs = 0;  // when the current burst began
float raiseGyroPeak = 0;        // fastest rotation seen during it, for calibration logging
uint32_t lastRaiseWakeMs = 0;   // last time raise-to-wake actually fired, for RAISE_COOLDOWN_MS

// ---------- BLE ----------
// Custom "Pip-Boy Time Service". The companion web page writes 4 bytes
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
    Serial.println("[pipboy] BLE client connected.");
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    forceRedraw = true;
    Serial.println("[pipboy] BLE client disconnected, re-advertising.");
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
  Serial.printf("[pipboy] GB rx: %.120s\n", line.c_str());

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
        Serial.printf("[pipboy] time synced via BLE (Gadgetbridge): %ld\n", (long)epoch);
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
    Serial.printf("[pipboy] GB JSON parse failed: %s\n", err.c_str());
    return;
  }

  const char *type = doc["t"] | "";

  if (strcmp(type, "notify") == 0) {
    long id = doc["id"] | 0L;
    notifAdd(id,
             String((const char *)(doc["src"] | "")),
             String((const char *)(doc["title"] | "")),
             String((const char *)(doc["body"] | "")));
    Serial.printf("[pipboy] notify id=%ld: %s / %s (%u held)\n",
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
    Serial.printf("[pipboy] notification dismissed on phone: id=%ld (%u left)\n", id, notifCount);

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
    Serial.printf("[pipboy] weather: %dC (%s)\n", weatherTempC, weatherTxt.c_str());

  } else if (strcmp(type, "musicinfo") == 0) {
    // Bangle.js "musicinfo": {t:"musicinfo", artist, album, track, dur, c, n}
    // — sent once whenever the track changes, not on every poll, so this is
    // deliberately sticky rather than re-derived each render.
    musicArtist = String((const char *)(doc["artist"] | ""));
    musicTrack = String((const char *)(doc["track"] | ""));
    musicDurSec = doc["dur"] | 0L;
    // A new track means the old position no longer applies — reset it
    // rather than carrying the previous song's elapsed time into this
    // one's progress ring until the next musicstate arrives to correct it.
    musicPositionSec = 0;
    musicPositionStampMs = millis();
    hasMusic = true;
    forceRedraw = true;
    Serial.printf("[pipboy] musicinfo: %s - %s\n", musicArtist.c_str(), musicTrack.c_str());

  } else if (strcmp(type, "musicstate") == 0) {
    // Bangle.js "musicstate": {t:"musicstate", state, position, shuffle, repeat}
    // — state is "play" or "pause". Gadgetbridge can send this on its own,
    // with no accompanying musicinfo — e.g. right after BLE connects, if a
    // track was already playing before the watch paired.
    //
    // hasMusic is set here unconditionally again after a previous attempt
    // gated it on musicTrack already having content: that made the "no
    // track name yet" case cosmetically better but broke the feature
    // outright whenever musicinfo never arrives at all (confirmed live —
    // "NOTHING" shown, and the play/pause tap does nothing since its
    // handler also checks hasMusic). Controls working unconditionally
    // matters more than the track label being briefly/permanently blank —
    // see updateMusicScreen() for how the blank-track case is handled at
    // the label level instead, without disabling anything.
    const char *state = doc["state"] | "";
    musicPlaying = (strcmp(state, "play") == 0);
    musicPositionSec = doc["position"] | 0L;
    musicPositionStampMs = millis();
    hasMusic = true;
    forceRedraw = true;
    Serial.printf("[pipboy] musicstate: %s pos=%ld\n", state, musicPositionSec);
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
  BLEDevice::init("Bangle.js Pip-Boy 3000"); // name must match Gadgetbridge's device-name allowlist

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

// Every watch->phone message goes out through here: a blank line to flush
// whatever partial line the far side may be holding, then the JSON with a
// single trailing space before the newline.
//
// That space is load-bearing, not a typo. Gadgetbridge was rejecting these
// as "Malformed json ... unterminated object at character N", and N pinned
// the cause down exactly: 27 for "previous", 24 for "pause", 23 for "next"
// — in each case the position of the closing quote of the value, i.e. it
// received the whole object except the final '}'. One byte, always the
// last one of the line.
//
// An earlier attempt padded *after* the newline, on the theory the tail of
// the transmission was being cut. Those same three numbers came back
// unchanged, ruling that out: what's lost isn't the end of the payload,
// it's the last character of the line — an off-by-one in how the line is
// sliced on the far side. The trailing space gives that off-by-one
// something harmless to eat. If it's ever fixed upstream, a stray trailing
// space inside the line is still valid JSON whitespace, so this stays
// correct either way.
void sendGadgetbridgeLine(const char *json) {
  if (!bleConnected || !nusTxChar) return;
  nusTxChar->setValue((uint8_t *)"\n", 1);
  nusTxChar->notify();

  char line[256];
  int n = snprintf(line, sizeof(line), "%s \n", json);
  if (n <= 0) return;
  if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1; // snprintf reports what it *would* have written
  nusTxChar->setValue((uint8_t *)line, n);
  nusTxChar->notify();
  Serial.printf("[pipboy] GB tx: %.200s\n", json);
}

// Sends a media-control command back to the phone, for the MUSIC screen's
// transport buttons. cmd is one of "play", "pause", "playpause", "next",
// "previous", "volumeup", "volumedown".
//
// Format confirmed against gbmusic.app.js, the actual Bangle.js app
// Gadgetbridge's music support is built against
// (github.com/espruino/BangleApps, apps/gbmusic/app.js) — the same
// {t:"...", ...} convention every other message here uses, not the
// Bangle.musicControl() JS call an earlier guess assumed.
void sendMusicControl(const char *cmd) {
  if (!bleConnected || !nusTxChar) return;
  // Note the space before the newline — it is load-bearing, not a typo.
  //
  // Gadgetbridge rejects these with "Malformed json ... unterminated
  // object at character N", and N pinned the cause down exactly: 27 for
  // "previous", 24 for "pause", 23 for "next". In each case that is the
  // position of the closing quote of the value — i.e. it receives the
  // whole object except the final '}'. One byte, always the last one of
  // the line, always missing.
  //
  // An earlier attempt padded *after* the newline on the theory that the
  // tail of the transmission was being cut. Those same three numbers came
  // back completely unchanged, which rules that out: what's lost isn't the
  // end of the payload, it's the last character of the line itself — an
  // off-by-one in how the line gets sliced out on the far side. So the fix
  // is to give that off-by-one something harmless to eat. A trailing space
  // sits inside the line, after the '}', and gets chopped instead of the
  // brace, leaving complete JSON. If the far side is ever fixed, a stray
  // trailing space inside the line is still valid JSON whitespace, so this
  // stays correct either way.
  char buf[48];
  snprintf(buf, sizeof(buf), "{\"t\":\"music\",\"n\":\"%s\"}", cmd);
  sendGadgetbridgeLine(buf);
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
  Serial.println("[pipboy] connecting to WiFi (saved credentials)...");
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
    Serial.println("[pipboy] WiFi connect failed/timed out.");
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println("[pipboy] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);

  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF); // save power — we don't need WiFi between syncs

  if (gotTime) {
    Serial.println("[pipboy] NTP sync OK.");
    timeSynced = true;
    forceRedraw = true;
  } else {
    Serial.println("[pipboy] NTP sync timed out.");
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
    Serial.println("[pipboy] WiFi setup portal timed out with no connection.");
    forceRedraw = true;
    return false;
  }

  Serial.println("[pipboy] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  if (gotTime) {
    timeSynced = true;
    Serial.println("[pipboy] NTP sync OK.");
  } else {
    Serial.println("[pipboy] NTP sync timed out.");
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
    Serial.println("[pipboy] WiFi connected, fetching NTP time...");
    gotTime = fetchNtpTime(8000);
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    if (gotTime) {
      timeSynced = true;
      Serial.println("[pipboy] NTP sync OK.");
    }
  } else {
    Serial.println("[pipboy] WiFi setup portal cancelled/timed out.");
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
// pipboy-watch-battery-reconnect-fix in project memory), not to shave the
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
      Serial.printf("[pipboy] battery: %dmV -> no plausible cell, reporting unknown\n", mv);
      cachedPct = BATT_UNKNOWN;
    } else {
      cachedPct = lipoPercent(mv);
      Serial.printf("[pipboy] battery: %dmV -> %d%%\n", mv, cachedPct);
    }
  }
  return cachedPct;
}

// ---------- persistence ----------
void loadState() {
  prefs.begin("pipboy", false);
  stepCounter.begin(prefs.getUInt("steps", 0));
  lastDayMarker = prefs.getUInt("dayMarker", 0);

  // Past 6 completed days' totals for the HISTORY screen. getBytes() returns
  // 0 (not sizeof(stepHist)) on first-ever boot, when the key doesn't exist
  // yet — treated the same as "no history", not as corruption.
  size_t histLen = prefs.getBytes("stepHist", stepHist, sizeof(stepHist));
  if (histLen != sizeof(stepHist)) memset(stepHist, 0, sizeof(stepHist));

  // User settings — each falls back to the compile-time default if unset.
  cfgBrightness = (uint8_t)prefs.getUChar("cfgBright", BRIGHT_ACTIVE);
  cfgDimTimeoutMs = prefs.getULong("cfgDim", IDLE_DIM_TIMEOUT_MS);
  cfgScanlines = prefs.getBool("cfgScan", true);
  cfgTime24h = prefs.getBool("cfg24h", false);

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

// Settings are written on change rather than on the periodic saveState()
// tick — they change rarely and a stale value surviving a reboot would be
// more annoying than the handful of extra flash writes.
void saveSettings() {
  prefs.begin("pipboy", false);
  prefs.putUChar("cfgBright", cfgBrightness);
  prefs.putULong("cfgDim", cfgDimTimeoutMs);
  prefs.putBool("cfgScan", cfgScanlines);
  prefs.putBool("cfg24h", cfgTime24h);
  prefs.end();
}

void saveWeather() {
  prefs.begin("pipboy", false);
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

  prefs.begin("pipboy", false);
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
    Serial.println("[pipboy] automatic light sleep ENABLED.");
  } else {
    Serial.printf("[pipboy] esp_pm_configure() -> %s — not active (core likely lacks CONFIG_PM_ENABLE)\n",
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
  Serial.println("[pipboy] long-press -> entering deep sleep...");
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
// pipboy-watch-battery-reconnect-fix in project memory) — whatever caused
// that, the actual fix is never letting the cell get that low again, not
// hoping the recovery works better next time. Shows a brief warning first:
// a watch that goes dark with no explanation reads as broken, which is
// exactly the confusion that started this in the first place.
void enterCriticalBatterySleep(int mv) {
  Serial.printf("[pipboy] critical battery (%dmV) — sleeping immediately to protect the cell\n", mv);

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
  display.setBrightness(cfgBrightness);
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
    Serial.printf("[pipboy] raise? held=%lums peak=%.0fdps a=(%.1f,%.1f,%.1f) dot=%.2f dur=%d cool=%d\n",
                  (unsigned long)heldMs, peak, ax, ay, az, dot,
                  durationLooksRight, offCooldown);

    if (durationLooksRight && offCooldown && dot > RAISE_ACCEL_DOT_MIN) {
      wakeScreen(now);
      lastRaiseWakeMs = now;
      Serial.println("[pipboy] raise-to-wake triggered.");
    }
  }
}

// ---------- drawing helpers (still used by the pre-LVGL boot sequence and
// the modal WiFi-setup/sleep screens, which stay raw LovyanGFX draws — see
// the note above lv_init() in setup() for why) ----------
void useSharpSmallFont() { display.setFont(&fonts::FreeMonoBold9pt7b); display.setTextSize(0.9f); }
void useSharpMedFont()   { display.setFont(&fonts::FreeMonoBold12pt7b); display.setTextSize(1.0f); }

// ---------- weather icons ----------
// Unchanged from pipboy_watch.ino except one thing: every function here
// used to draw straight onto the global `display` object. They now take an
// explicit `lgfx::LGFXBase &gfx` target instead, so the exact same
// pixel-drawing logic can be pointed at either the real panel (unused here)
// or an off-screen LGFX_Sprite — see renderWeatherIconSprite() below, which
// is what actually calls these to feed LVGL an image. Reusing the math this
// way beats redrawing the same shapes as native LVGL primitives from
// scratch.
//
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

void drawIconSun(lgfx::LGFXBase &gfx, int cx, int cy, int s, uint16_t col) {
  int r = s * 20 / 100;
  gfx.fillCircle(cx, cy, r, col);
  int inner = r + s * 6 / 100, outer = r + s * 15 / 100;
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4.0f;
    gfx.drawWideLine(cx + cosf(a) * inner, cy + sinf(a) * inner,
                      cx + cosf(a) * outer, cy + sinf(a) * outer, s * 5 / 100, col);
  }
}

// Cloud built from three puffs over a flat-bottomed rounded base.
void drawIconCloud(lgfx::LGFXBase &gfx, int x, int y, int s, uint16_t col) {
  gfx.fillCircle(x + s * 40 / 100, y + s * 46 / 100, s * 20 / 100, col);
  gfx.fillCircle(x + s * 63 / 100, y + s * 55 / 100, s * 16 / 100, col);
  gfx.fillCircle(x + s * 22 / 100, y + s * 56 / 100, s * 14 / 100, col);
  gfx.fillSmoothRoundRect(x + s * 10 / 100, y + s * 56 / 100,
                           s * 70 / 100, s * 20 / 100, s * 8 / 100, col);
}

// Three slanted streaks under a cloud, shared by rain and (shorter) drizzle.
void drawIconRainStreaks(lgfx::LGFXBase &gfx, int x, int y, int s, uint16_t col) {
  for (int i = 0; i < 3; i++) {
    int sx = x + s * (24 + i * 20) / 100;
    gfx.drawWideLine(sx, y + s * 80 / 100, sx - s * 7 / 100, y + s * 96 / 100, s * 5 / 100, col);
  }
}

void drawWeatherIcon(lgfx::LGFXBase &gfx, int x, int y, int s, WeatherIcon icon, uint16_t col) {
  // Wipe the box first: shapes drawn with fills don't erase what a
  // different icon left behind on an earlier frame.
  gfx.fillRect(x, y, s, s, COL_BG);

  switch (icon) {
    case WI_SUN:
      drawIconSun(gfx, x + s / 2, y + s / 2, s, col);
      break;

    case WI_PARTLY:
      // Sun peeking out of the top-left, cloud overlapping its lower right.
      drawIconSun(gfx, x + s * 38 / 100, y + s * 36 / 100, s * 78 / 100, col);
      gfx.fillCircle(x + s * 40 / 100, y + s * 46 / 100, s * 20 / 100, COL_BG); // keep the cloud's edge readable against the rays
      drawIconCloud(gfx, x, y + s * 12 / 100, s, col);
      break;

    case WI_CLOUD:
      drawIconCloud(gfx, x, y + s * 6 / 100, s, col);
      break;

    case WI_RAIN:
      drawIconCloud(gfx, x, y - s * 6 / 100, s, col);
      drawIconRainStreaks(gfx, x, y - s * 6 / 100, s, col);
      break;

    case WI_SNOW:
      drawIconCloud(gfx, x, y - s * 6 / 100, s, col);
      for (int i = 0; i < 3; i++) {
        gfx.fillCircle(x + s * (26 + i * 20) / 100, y + s * 82 / 100, s * 5 / 100, col);
      }
      break;

    case WI_STORM:
      drawIconCloud(gfx, x, y - s * 8 / 100, s, col);
      // Lightning bolt: two offset triangles make the classic zigzag.
      gfx.fillTriangle(x + s * 52 / 100, y + s * 66 / 100,
                        x + s * 34 / 100, y + s * 66 / 100,
                        x + s * 46 / 100, y + s * 90 / 100, col);
      gfx.fillTriangle(x + s * 56 / 100, y + s * 62 / 100,
                        x + s * 40 / 100, y + s * 78 / 100,
                        x + s * 58 / 100, y + s * 78 / 100, col);
      break;

    case WI_FOG:
      drawIconCloud(gfx, x, y - s * 12 / 100, s, col);
      for (int i = 0; i < 3; i++) {
        int ly = y + s * (68 + i * 12) / 100;
        int inset = (i % 2) ? s * 22 / 100 : s * 12 / 100;
        gfx.fillSmoothRoundRect(x + inset, ly, s - inset * 2, s * 5 / 100, s * 2 / 100, col);
      }
      break;
  }
}

// ---------- LVGL bridge ----------
lv_display_t *lvDisplay = nullptr;
lv_obj_t *screens[NUM_SCREENS]; // one full-screen container per SCREEN_* slot

// Pushes whatever LVGL rendered for `area` straight to the panel —
// synchronous, no DMA hand-off. Same "draw straight to the panel, easier to
// trust, easier to debug" reasoning pipboy_watch's header comment gives for
// not using an offscreen sprite there.
void lvglFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  display.startWrite();
  display.setAddrWindow(area->x1, area->y1, w, h);
  display.writePixels(reinterpret_cast<lgfx::rgb565_t *>(px_map), w * h);
  display.endWrite();
  lv_display_flush_ready(disp);
}

// Raw touch (x,y) -> LVGL logical (x,y). See the "touch coordinate remap"
// comment near TOUCH_INVERT_X/Y up top for what this transform is and how
// confident it is (derived from pipboy_watch's confirmed swipe-axis
// constants, not independently re-measured for absolute positions).
// Debounced the same way pipboy_watch's own touch poll was (see its
// comment on "a glitchy I2C read occasionally comes back looking like
// 'finger down' with nothing touching it"): reject out-of-range coordinates
// outright, and only trust "down" once it's shown up on 3 consecutive polls
// in a row. This version had none of that at first — a single noisy read
// could hand LVGL a phantom multi-second "press" out of pure I2C noise.
void lvglTouchRead(lv_indev_t *indev, lv_indev_data_t *data) {
  static uint8_t downStreak = 0;
  int16_t rx, ry;
  bool rawDown = touchOk && touch.read(rx, ry);
  if (rawDown && (rx < 0 || rx > 239 || ry < 0 || ry > 239)) rawDown = false;
  downStreak = rawDown ? min((int)downStreak + 1, 5) : 0;

  if (downStreak >= 3) {
    data->state = LV_INDEV_STATE_PRESSED;
    int16_t lx = ry, ly = rx; // transpose
    if (TOUCH_INVERT_X) lx = 239 - lx;
    if (TOUCH_INVERT_Y) ly = 239 - ly;
    data->point.x = lx;
    data->point.y = ly;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// Switches the visible screen and keeps currentScreen (still used all over
// — auto-sleep exclusion, BLE popup targeting, etc.) in sync with it.
// Screens slide rather than cutting straight over: an instant swap reads
// as the display glitching, especially with a whole screen's worth of
// content changing at once. Direction carries meaning — going *into* a
// section drops the new screen down from the top, going back up to the
// clock slides away upward — so the motion says which way you moved
// through the UI rather than just being decoration.
void goToScreen(uint8_t idx) {
  if (idx >= NUM_SCREENS) return;
  lv_scr_load_anim_t anim = (idx == SCREEN_STAT) ? LV_SCR_LOAD_ANIM_MOVE_TOP : LV_SCR_LOAD_ANIM_MOVE_BOTTOM;
  currentScreen = idx;
  lv_screen_load_anim(screens[idx], anim, 220, 0, false);
  forceRedraw = true;
}

// ---------- CRT scanline overlay ----------
// Real per-pixel alpha this time (LVGL's image compositor does it; raw
// LovyanGFX draws on pipboy_watch didn't, which is why that attempt was a
// flat, ugly pixel overwrite and got pulled). One small 240x3 ARGB8888
// tile (a lit row plus two transparent rows), tiled down a single object
// parked on lv_layer_top() — the layer LVGL always draws above whichever
// screen is active — so one overlay covers every screen without
// duplicating it per screen.
static const uint8_t SCANLINE_ALPHA = 55; // 0-255; kept faint — this is texture, not a spotlight
static uint8_t scanlineTileBuf[240 * 3 * 4]; // ARGB8888: 3 rows x 240 px x 4 bytes
static lv_image_dsc_t scanlineTileDsc;

void buildScanlineOverlay() {
  for (int y = 0; y < 3; y++) {
    for (int x = 0; x < 240; x++) {
      uint8_t *px = &scanlineTileBuf[(y * 240 + x) * 4];
      bool litRow = (y == 0);
      px[0] = litRow ? 10 : 0;  // B
      px[1] = litRow ? 40 : 0;  // G
      px[2] = litRow ? 5 : 0;   // R — faint dark green, matching COL_SCANLINE's intent
      px[3] = litRow ? SCANLINE_ALPHA : 0; // A — the other two rows stay fully transparent
    }
  }
  scanlineTileDsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  scanlineTileDsc.header.w = 240;
  scanlineTileDsc.header.h = 3;
  scanlineTileDsc.header.cf = LV_COLOR_FORMAT_ARGB8888;
  scanlineTileDsc.header.stride = 240 * 4;
  scanlineTileDsc.data_size = sizeof(scanlineTileBuf);
  scanlineTileDsc.data = scanlineTileBuf;

  lv_obj_t *top = lv_layer_top();
  lv_obj_set_size(top, 240, 240);
  lv_obj_set_pos(top, 0, 0);
  // Non-clickable is essential here, not optional: lv_layer_top() sits
  // visually above the active screen, and LVGL's input targeting checks it
  // too — without this, the overlay would steal every touch meant for the
  // actual screen underneath.
  lv_obj_remove_flag(top, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_opa(top, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(top, 0, 0);
  lv_obj_set_style_bg_image_src(top, &scanlineTileDsc, 0);
  lv_obj_set_style_bg_image_tiled(top, true, 0);
}

// The moving "refresh beam" sweep that used to live here — a bright line
// animating top-to-bottom on a loop — was removed by request. The static
// scanline texture above is the only CRT overlay now.

// ---------- touch gesture handling (LVGL events) ----------
// Replaces pipboy_watch's hand-rolled poll/debounce/gesture state machine —
// LVGL's input device layer already does that debouncing and tap/swipe/
// gesture classification. What's reused wholesale is the *decision logic*
// downstream of "a tap/swipe/long-press just happened": the exact duration
// thresholds (TAP_MAX_MS, LONG_PRESS_MS, SLEEP_PRESS_MAX_MS,
// LONG_PRESS_MAX_MS), the per-screen tap actions, and the
// dim-wake-must-not-also-navigate rule are all identical to pipboy_watch's.
static uint32_t pressStartMs = 0;
static lv_point_t pressStartPoint = {0, 0};
static bool pressHeldStill = true;
static bool pressWasDimWake = false;
// Guards against a RELEASED that arrives with no matching PRESSED seen this
// run — without this, onScreenReleased() would compute
// heldMs = millis() - pressStartMs against pressStartMs's stale zero-value,
// i.e. heldMs = millis(): the *entire elapsed boot time*, which by the time
// boot actually finishes is already several seconds — almost always enough
// to land right in the long-press bucket and trigger deep sleep on
// literally the first touch after power-on. This was a real, confirmed bug
// (reported as "goes to sleep as soon as I touch it"), not a hypothetical.
static bool isPressing = false;

// Every touch that counts as the user doing something has to run through
// here — resets the idle-dim timer and un-dims if the backlight had
// already dropped.
//
// It's a separate function rather than living inside onScreenPressed()
// because that handler is registered on the *screen* objects, and LVGL
// doesn't bubble press events from a child widget up to its parent by
// default. So a tap that lands on a nav badge, a back button or a music
// transport button never reached onScreenPressed() at all: the button did
// its job, but the screen stayed dim and the idle timer kept running as
// if nothing had been touched. Every button handler calls this too.
void noteTouchActivity() {
  lastActivityMs = millis();
  if (backlightDimmed) wakeScreen(millis());
}

void onScreenPressed(lv_event_t *e) {
  isPressing = true;
  pressStartMs = millis();
  lv_indev_t *indev = lv_indev_active();
  if (indev) lv_indev_get_point(indev, &pressStartPoint);
  pressHeldStill = true;
  pressWasDimWake = backlightDimmed;
  noteTouchActivity();
}

void onScreenPressing(lv_event_t *e) {
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;
  lv_point_t p;
  lv_indev_get_point(indev, &p);
  if (abs((int)(p.x - pressStartPoint.x)) >= 15 || abs((int)(p.y - pressStartPoint.y)) >= 15) pressHeldStill = false;
  lastActivityMs = millis();
}

// Tap actions and the long-press power gestures, dispatched by duration —
// same "duration, not screen, selects the long-press action" reasoning
// pipboy_watch's own comment gives (waking from deep sleep always lands
// back on STAT, so keying the action off which screen you were on proved
// unpredictable in practice there; same logic applies here unchanged).
void onScreenReleased(lv_event_t *e) {
  if (!isPressing) return; // no matching press this run — nothing to evaluate, see the comment on isPressing
  isPressing = false;
  uint32_t heldMs = millis() - pressStartMs;
  bool tapGesture = !pressWasDimWake && pressHeldStill && heldMs < TAP_MAX_MS;
  // Long-press is deliberately NOT gated on pressWasDimWake — holding still
  // for 3+ seconds is unambiguous intent even on the touch that woke the
  // screen; see pipboy_watch's identical comment on this.
  bool longPress = pressHeldStill && heldMs >= LONG_PRESS_MS && heldMs <= LONG_PRESS_MAX_MS;

  if (tapGesture && currentScreen == SCREEN_STOPWATCH) {
    if (swRunning) {
      swAccumMs += millis() - swStartMs;
      swRunning = false;
    } else {
      swStartMs = millis();
      swRunning = true;
    }
    forceRedraw = true;

  } else if (tapGesture && currentScreen == SCREEN_DATA) {
    // Four deliberate taps, each within STEPS_RESET_ARM_MS of the last,
    // before a day's step count is wiped — identical gesture to
    // pipboy_watch's, same reasoning (nothing else here is destructive).
    uint32_t now = millis();
    if (stepsResetArmedMs && now - stepsResetArmedMs < STEPS_RESET_ARM_MS) {
      stepsResetTaps++;
    } else {
      stepsResetTaps = 1;
    }
    stepsResetArmedMs = now;
    if (stepsResetTaps >= STEPS_RESET_TAPS) {
      stepCounter.set(0);
      saveState();
      stepsResetArmedMs = 0;
      stepsResetTaps = 0;
      Serial.println("[pipboy] steps reset by user.");
    }
    forceRedraw = true;

  } else if (longPress) {
    Serial.printf("[pipboy] long-press released after %lums on screen %u\n",
                  (unsigned long)heldMs, currentScreen);
    if (heldMs < SLEEP_PRESS_MAX_MS) {
      enterDeepSleep(); // never returns
    } else {
      forceWifiSetupPortal();
      lv_indev_reset(NULL, NULL); // stale press state after a blocking call — same caution pipboy_watch takes
    }
  }
}

// Left/right swipe navigation between screens is gone by request —
// getting around is now entirely via the nav ring on STAT and the back
// button on every other screen. The only gesture left is the
// stopwatch-clear swipe, which was never screen-to-screen navigation.
void onScreenGesture(lv_event_t *e) {
  if (pressWasDimWake) return; // a wake-touch must not also act — see pipboy_watch's identical rule
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(indev);

  if (dir == LV_DIR_BOTTOM && currentScreen == SCREEN_STAT) {
    // The one navigation gesture left: a downward swipe on the clock face
    // pulls settings down. Kept as a gesture rather than a seventh nav
    // badge so the ring's spacing stays as-is. Flip to LV_DIR_TOP if it
    // turns out to read backwards on the panel.
    goToScreen(SCREEN_SETTINGS);

  } else if (dir == LV_DIR_TOP || dir == LV_DIR_BOTTOM) {
    if (currentScreen == SCREEN_STOPWATCH && !swRunning) {
      swAccumMs = 0;
      forceRedraw = true;
    }
  }
}

// ---------- status cluster (Bluetooth + battery + %) ----------
// One label per screen, reusing LVGL's built-in symbol glyphs
// (LV_SYMBOL_BLUETOOTH, LV_SYMBOL_BATTERY_*) instead of the hand-drawn
// battery-shell/Bluetooth-rune pipboy_watch draws itself
// (drawBluetoothIcon()/drawBatteryIcon() there) — a direct, deliberate
// trade of a little hand-drawn character for a lot less code, and it's
// exactly the kind of glyph LVGL ships for this.
lv_obj_t *clusterLabel[NUM_SCREENS];

lv_obj_t *buildStatusCluster(lv_obj_t *screen, lv_color_t accent) {
  lv_obj_t *lbl = lv_label_create(screen);
  lv_obj_set_style_text_color(lbl, accent, 0);
  lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -12);
  return lbl;
}

const char *batterySymbol(int pct) {
  if (pct > 80) return LV_SYMBOL_BATTERY_FULL;
  if (pct > 55) return LV_SYMBOL_BATTERY_3;
  if (pct > 30) return LV_SYMBOL_BATTERY_2;
  if (pct > 10) return LV_SYMBOL_BATTERY_1;
  return LV_SYMBOL_BATTERY_EMPTY;
}

void updateStatusCluster(uint8_t screenIdx, int battPct) {
  if (!clusterLabel[screenIdx]) return;
  char buf[40];
  const char *bt = bleConnected ? LV_SYMBOL_BLUETOOTH " " : "";
  if (battPct == BATT_UNKNOWN) {
    snprintf(buf, sizeof(buf), "%s--", bt);
  } else {
    snprintf(buf, sizeof(buf), "%s%s %d%%", bt, batterySymbol(battPct), battPct);
  }
  lv_label_set_text(clusterLabel[screenIdx], buf);
}

// ---------- weather icon sprite bridge ----------
// Renders drawWeatherIcon() (pipboy_watch's exact hand-drawn icon math,
// see above) into an off-screen LovyanGFX sprite, then hands that buffer
// to LVGL as a plain RGB565 image — the cheapest way to reuse those shapes
// without either re-implementing them as native LVGL primitives, or trying
// to draw raw LovyanGFX calls into LVGL's own framebuffer (which LVGL, as
// the thing that owns and repaints that buffer, would just overwrite on
// its next invalidation pass).
static LGFX_Sprite weatherIconSprite(&display);
static lv_image_dsc_t weatherIconImgDsc;
bool weatherIconSpriteReady = false;
static const int WEATHER_ICON_PX = 48; // was 64 — shrunk to clear the STAT screen's date label above it

void ensureWeatherIconSprite() {
  if (weatherIconSpriteReady) return;
  weatherIconSprite.setColorDepth(16);
  weatherIconSprite.createSprite(WEATHER_ICON_PX, WEATHER_ICON_PX);
  weatherIconSpriteReady = true;
}

// Re-renders the icon into the shared sprite and points imgDsc at its
// buffer. Callers pass the *same* lv_image_dsc_t back into
// lv_image_set_src() after calling this — LVGL just copies out of it, so
// one shared sprite/descriptor pair serves every lv_image that shows a
// weather icon (STAT and WEATHER both do).
void renderWeatherIconSprite(WeatherIcon icon, uint16_t col) {
  ensureWeatherIconSprite();
  weatherIconSprite.fillSprite(COL_BG);
  drawWeatherIcon(weatherIconSprite, 0, 0, WEATHER_ICON_PX, icon, col);
  weatherIconImgDsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  weatherIconImgDsc.header.w = WEATHER_ICON_PX;
  weatherIconImgDsc.header.h = WEATHER_ICON_PX;
  // SWAPPED, not plain RGB565: LovyanGFX's sprite buffer and LVGL's "plain"
  // RGB565 disagree on byte order for this specific path (this bridge
  // hands LVGL a raw LovyanGFX buffer directly, bypassing the main
  // display's own flush pipeline where that's already reconciled). Confirmed,
  // not guessed — decoding the reported "purple cloud" byte-swapped exactly
  // reconstructs COL_YELLOW.
  weatherIconImgDsc.header.cf = LV_COLOR_FORMAT_RGB565_SWAPPED;
  weatherIconImgDsc.header.stride = WEATHER_ICON_PX * 2;
  weatherIconImgDsc.data_size = (uint32_t)WEATHER_ICON_PX * WEATHER_ICON_PX * 2;
  weatherIconImgDsc.data = (const uint8_t *)weatherIconSprite.getBuffer();
}

// ---------- screen: STAT ----------
// Also the only screen with direct-navigation buttons to every other
// screen — small badges ringed around its edge, one per remaining screen,
// so getting to WEATHER or MUSIC doesn't mean swiping through everything
// in between. LVGL symbol glyphs are used where a good match exists
// (bell for notifications, a music note for now-playing); the rest use a
// short 2-letter label instead of guessing at an approximate icon, so
// every badge reads the same way rather than being real icons for some
// screens and mystery glyphs for others.
void onNavButtonClicked(lv_event_t *e) {
  noteTouchActivity();
  uint8_t target = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  goToScreen(target);
}

static const int NAV_BTN_PX = 40; // nav badge diameter

// Nav badge icons are real artwork now (see nav_icons.h) rather than
// shapes drawn from primitives — a hand-built sole/heel/toe shoe
// silhouette simply didn't read at 26px, and the same would eventually be
// true of any detail drawn that small. These are stored white with an
// alpha channel and tinted to each badge's accent colour via LVGL's image
// recolor style, so one bitmap serves whatever colour the badge is.
static lv_image_dsc_t navWalkImgDsc;
static lv_image_dsc_t navSunImgDsc;

static void initNavIconDsc(lv_image_dsc_t &dsc, const uint8_t *map, int w, int h) {
  dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  dsc.header.w = w;
  dsc.header.h = h;
  dsc.header.cf = LV_COLOR_FORMAT_ARGB8888;
  dsc.header.stride = w * 4;
  dsc.data_size = (uint32_t)w * h * 4;
  dsc.data = map;
}

void initNavIcons() {
  initNavIconDsc(navWalkImgDsc, NAV_ICON_WALK_MAP, NAV_ICON_WALK_W, NAV_ICON_WALK_H);
  initNavIconDsc(navSunImgDsc, NAV_ICON_SUN_MAP, NAV_ICON_SUN_W, NAV_ICON_SUN_H);
}

// Shared shell for every nav badge — the caller adds whatever goes inside
// (a text label, a symbol glyph, or an image).
lv_obj_t *makeNavButtonShell(lv_obj_t *parent, lv_color_t accent, uint8_t targetScreen, int dx, int dy) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, NAV_BTN_PX, NAV_BTN_PX);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, accent, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_bg_color(btn, accent, LV_STATE_PRESSED); // visible feedback on press
  lv_obj_align(btn, LV_ALIGN_CENTER, dx, dy);
  lv_obj_add_event_cb(btn, onNavButtonClicked, LV_EVENT_CLICKED, (void *)(uintptr_t)targetScreen);
  return btn;
}

lv_obj_t *makeNavButton(lv_obj_t *parent, const char *label, lv_color_t accent, uint8_t targetScreen, int dx, int dy) {
  lv_obj_t *btn = makeNavButtonShell(parent, accent, targetScreen, dx, dy);
  lv_obj_t *lbl = lv_label_create(btn);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, accent, 0);
  lv_label_set_text(lbl, label);
  lv_obj_center(lbl);
  return btn;
}

lv_obj_t *makeNavButtonImg(lv_obj_t *parent, const lv_image_dsc_t *img, lv_color_t accent, uint8_t targetScreen, int dx, int dy) {
  lv_obj_t *btn = makeNavButtonShell(parent, accent, targetScreen, dx, dy);
  lv_obj_t *icon = lv_image_create(btn);
  lv_image_set_src(icon, img);
  // Artwork is stored white; recolor paints it this badge's accent.
  lv_obj_set_style_image_recolor(icon, accent, 0);
  lv_obj_set_style_image_recolor_opa(icon, LV_OPA_COVER, 0);
  lv_obj_center(icon);
  return btn;
}

// Back-to-clock button, one per non-STAT screen. With swipe navigation
// gone, this is the *only* way back, so it sits in exactly the same spot
// on every screen — bottom centre — rather than being tucked wherever each
// screen happened to have room. The status cluster moves to the top on
// these screens to clear that space (STAT keeps its cluster at the bottom;
// it has no back button, being the place you go back *to*).
void addBackNav(lv_obj_t *parent, uint8_t screenIdx, lv_color_t accent) {
  if (clusterLabel[screenIdx]) lv_obj_align(clusterLabel[screenIdx], LV_ALIGN_TOP_MID, 0, 14);

  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, 44, 30);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, accent, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_bg_color(btn, accent, LV_STATE_PRESSED);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -6);
  lv_obj_add_event_cb(btn, onNavButtonClicked, LV_EVENT_CLICKED, (void *)(uintptr_t)SCREEN_STAT);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, accent, 0);
  lv_label_set_text(lbl, LV_SYMBOL_LEFT);
  lv_obj_center(lbl);
}

lv_obj_t *statTime, *statDate, *statIcon, *statTemp;
lv_obj_t *navNotifBtn; // kept so it can show an unread-notification state — see updateStatScreen()

void buildStatScreen() {
  lv_obj_t *s = screens[SCREEN_STAT] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0x55FF6E); // COL_GREEN

  // Nav ring spread across a much wider arc — ±100° from top rather than
  // the ±68° it was crammed into, which left the six badges bunched in a
  // narrow band across the top. Going wider means the outermost two drop
  // to abs y≈136, just past the panel's midline, which is fine at the far
  // left and right: neither the date row (x≈65-175) nor the weather
  // icon/temperature row below it (x≈66-170) reaches that far out, so
  // there's nothing down there to collide with.
  //
  // Spacing is set by geometry, not guesswork: 40px badges need ≥42px
  // between centres, and a 200° span at r=92 puts them 40° (≈63px) apart —
  // comfortably clear. Offsets are r·sin(θ) / −r·cos(θ). Still worth an
  // eyeball pass on the real panel, same as every other pixel-exact spot
  // in this file.
  initNavIcons();
  makeNavButton(s, "SW", lv_color_hex(0xFF554B), SCREEN_STOPWATCH, -91, 16);            // COL_RED, -100°
  makeNavButton(s, LV_SYMBOL_AUDIO, lv_color_hex(0xAF6EFF), SCREEN_MUSIC, -80, -46);    // COL_PURPLE, -60°
  navNotifBtn = makeNavButton(s, LV_SYMBOL_BELL, lv_color_hex(0x5AAFFF), SCREEN_NOTIF, -31, -86); // COL_BLUE, -20°
  makeNavButtonImg(s, &navWalkImgDsc, lv_color_hex(0x55FF6E), SCREEN_DATA, 31, -86);    // COL_GREEN, +20°
  makeNavButton(s, "7D", lv_color_hex(0xFF9632), SCREEN_HISTORY, 80, -46);              // COL_ORANGE, +60°
  makeNavButtonImg(s, &navSunImgDsc, lv_color_hex(0xFFDC46), SCREEN_WEATHER, 91, 16);   // COL_YELLOW, +100°

  // The notification banner that used to sit at the top of this screen is
  // gone — the nav ring now occupies that band, and it was the only row
  // with space for it. Nothing is actually lost: a new notification still
  // pops the NOTIF screen up on arrival (see handleGadgetbridgeLine()), the
  // full list is a tap away on the bell, and the bell itself now fills in
  // when anything is waiting.
  statTime = lv_label_create(s);
  lv_obj_set_style_text_font(statTime, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(statTime, accent, 0); // green, matching the rest of the phosphor theme
  lv_label_set_text(statTime, "--:--");
  lv_obj_align(statTime, LV_ALIGN_CENTER, 0, -18); // +8: at 10:00 the leading "1" was catching the music badge at dx=-80,dy=-46

  // Date/icon/temp row spacing below is deliberately generous — the
  // original values (date at dy=24, icon at dy=56) put the 64px icon's
  // top edge right at the date label's bottom edge, and since the icon has
  // no alpha channel (plain RGB565, opaque square, not just the drawn
  // cloud shape), that overlap showed up as the icon's black background
  // square blanking out the date text. Confirmed from a live report, not
  // just reasoned about — see WEATHER_ICON_PX's comment for the matching
  // size reduction.
  statDate = lv_label_create(s);
  lv_obj_set_style_text_font(statDate, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(statDate, accent, 0);
  lv_label_set_text(statDate, "-- NO SIGNAL --");
  lv_obj_align(statDate, LV_ALIGN_CENTER, 0, 22); // +8, keeping the stack's spacing intact

  statIcon = lv_image_create(s);
  lv_obj_align(statIcon, LV_ALIGN_CENTER, -30, 62); // +8

  statTemp = lv_label_create(s);
  lv_obj_set_style_text_font(statTemp, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(statTemp, lv_color_hex(0xFFDC46), 0); // COL_YELLOW
  lv_label_set_text(statTemp, "");
  lv_obj_align(statTemp, LV_ALIGN_CENTER, 22, 62); // +8

  clusterLabel[SCREEN_STAT] = buildStatusCluster(s, accent);
}

void updateStatScreen(struct tm &t, int battPct) {
  // Bell badge fills in when anything is waiting — this is what replaced
  // the old text banner across the top of this screen.
  if (navNotifBtn) {
    lv_obj_set_style_bg_color(navNotifBtn, notifCount ? lv_color_hex(0x5AAFFF) : lv_color_black(), 0);
  }

  char timebuf[16];
  if (cfgTime24h) {
    snprintf(timebuf, sizeof(timebuf), "%02d:%02d", t.tm_hour, t.tm_min);
  } else {
    int hour12 = t.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;
    snprintf(timebuf, sizeof(timebuf), "%d:%02d", hour12, t.tm_min);
  }
  lv_label_set_text(statTime, timebuf);

  static const char *wd[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  static const char *mo[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  if (timeSynced) {
    char datebuf[32];
    snprintf(datebuf, sizeof(datebuf), "%s %s %d", wd[t.tm_wday], mo[t.tm_mon], t.tm_mday);
    lv_label_set_text(statDate, datebuf);
  } else {
    lv_label_set_text(statDate, "-- NO SIGNAL --");
  }

  if (hasWeather && !weatherTooStale()) {
    lv_obj_clear_flag(statIcon, LV_OBJ_FLAG_HIDDEN);
    renderWeatherIconSprite(weatherIconFor(weatherCode, weatherTxt), COL_YELLOW);
    lv_image_set_src(statIcon, &weatherIconImgDsc);
    char tbuf[8];
    snprintf(tbuf, sizeof(tbuf), "%dC", weatherTempC); // no degree glyph in LVGL's default font subset, same constraint pipboy_watch has
    lv_label_set_text(statTemp, tbuf);
  } else {
    lv_obj_add_flag(statIcon, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(statTemp, "");
  }

  updateStatusCluster(SCREEN_STAT, battPct);
}

// ---------- screen: DATA ----------
lv_obj_t *dataTitle, *dataCount, *dataSub;

void buildDataScreen() {
  lv_obj_t *s = screens[SCREEN_DATA] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0x55FF6E); // COL_GREEN — steps moved off orange; HISTORY keeps orange so the two stay tellable apart

  dataTitle = lv_label_create(s);
  lv_obj_set_style_text_font(dataTitle, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(dataTitle, accent, 0);
  lv_label_set_text(dataTitle, "Steps");
  lv_obj_align(dataTitle, LV_ALIGN_TOP_MID, 0, 44);

  dataCount = lv_label_create(s);
  lv_obj_set_style_text_font(dataCount, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(dataCount, accent, 0);
  lv_label_set_text(dataCount, "0");
  lv_obj_align(dataCount, LV_ALIGN_CENTER, 0, 0);

  dataSub = lv_label_create(s);
  lv_obj_set_style_text_font(dataSub, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(dataSub, accent, 0);
  lv_label_set_text(dataSub, "tap 4x to reset");
  lv_obj_align(dataSub, LV_ALIGN_CENTER, 0, 44);

  clusterLabel[SCREEN_DATA] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_DATA, accent);
}

void updateDataScreen(int steps, int battPct) {
  char stepbuf[12];
  snprintf(stepbuf, sizeof(stepbuf), "%lu", (unsigned long)steps);
  lv_label_set_text(dataCount, stepbuf);

  if (stepsResetArmedMs && millis() - stepsResetArmedMs < STEPS_RESET_ARM_MS) {
    char prompt[28];
    snprintf(prompt, sizeof(prompt), "RESET? TAP %u MORE", STEPS_RESET_TAPS - stepsResetTaps);
    lv_label_set_text(dataSub, prompt);
  } else {
    float km = steps * 0.00075f; // ~0.75m average stride — same as pipboy_watch
    char distbuf[24];
    snprintf(distbuf, sizeof(distbuf), "~%.2f KM", km);
    lv_label_set_text(dataSub, distbuf);
  }

  updateStatusCluster(SCREEN_DATA, battPct);
}

// ---------- screen: HISTORY ----------
// Native lv_bar widgets instead of pipboy_watch's hand-drawn fillRect bars
// (see drawHistoryScreen() in pipboy_watch.ino) — same 7-day-window idea
// (stepHist[0..5] plus today, read live from stepCounter), LVGL just draws
// the bars itself.
lv_obj_t *histTitle, *histBars[7], *histLabels[7];

void buildHistoryScreen() {
  lv_obj_t *s = screens[SCREEN_HISTORY] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0xFF9632); // COL_ORANGE

  histTitle = lv_label_create(s);
  lv_obj_set_style_text_font(histTitle, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(histTitle, accent, 0);
  lv_label_set_text(histTitle, "7-Day Steps");
  lv_obj_align(histTitle, LV_ALIGN_TOP_MID, 0, 42); // 42 not 30: the status cluster now sits at the top on this screen

  const int barW = 16, gap = 6, totalW = 7 * barW + 6 * gap;
  int x0 = -totalW / 2 + barW / 2;
  for (int i = 0; i < 7; i++) {
    histBars[i] = lv_bar_create(s);
    lv_obj_set_size(histBars[i], barW, 90);
    lv_bar_set_range(histBars[i], 0, 100);
    lv_obj_set_style_bg_color(histBars[i], lv_color_hex(0x193B2A), LV_PART_MAIN); // COL_GREEN_DIM-ish track
    lv_obj_set_style_bg_color(histBars[i], (i == 6) ? accent : lv_color_hex(0x196428), LV_PART_INDICATOR);
    lv_obj_align(histBars[i], LV_ALIGN_CENTER, x0 + i * (barW + gap), 10);

    histLabels[i] = lv_label_create(s);
    lv_obj_set_style_text_font(histLabels[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(histLabels[i], (i == 6) ? accent : lv_color_hex(0x196428), 0);
    lv_label_set_text(histLabels[i], "-");
    lv_obj_align_to(histLabels[i], histBars[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 6);
  }

  clusterLabel[SCREEN_HISTORY] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_HISTORY, accent);
}

void updateHistoryScreen(int battPct) {
  uint32_t vals[7];
  for (uint8_t i = 0; i < 6; i++) vals[i] = stepHist[i];
  vals[6] = stepCounter.count();

  uint32_t maxVal = 1;
  for (uint8_t i = 0; i < 7; i++) if (vals[i] > maxVal) maxVal = vals[i];

  static const char *dowInitial[7] = {"S", "M", "T", "W", "T", "F", "S"};
  time_t nowEpoch = time(nullptr);
  struct tm lt;
  localtime_r(&nowEpoch, &lt);

  for (uint8_t i = 0; i < 7; i++) {
    int pct = (int)((uint64_t)vals[i] * 100 / maxVal);
    lv_bar_set_value(histBars[i], pct, LV_ANIM_OFF);
    int wday = ((lt.tm_wday - (6 - i)) % 7 + 7) % 7;
    lv_label_set_text(histLabels[i], dowInitial[wday]);
  }

  updateStatusCluster(SCREEN_HISTORY, battPct);
}

// ---------- screen: WEATHER ----------
lv_obj_t *wxTitle, *wxIcon, *wxTemp, *wxCond, *wxHi, *wxLo, *wxNoData1, *wxNoData2;

void buildWeatherScreen() {
  lv_obj_t *s = screens[SCREEN_WEATHER] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0xFFDC46); // COL_YELLOW

  wxTitle = lv_label_create(s);
  lv_obj_set_style_text_font(wxTitle, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(wxTitle, accent, 0);
  lv_label_set_text(wxTitle, "Weather");
  lv_obj_align(wxTitle, LV_ALIGN_TOP_MID, 0, 42); // 42 not 30: the status cluster now sits at the top on this screen

  wxIcon = lv_image_create(s);
  lv_obj_align(wxIcon, LV_ALIGN_CENTER, -34, -20);

  wxTemp = lv_label_create(s);
  lv_obj_set_style_text_font(wxTemp, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(wxTemp, accent, 0);
  lv_obj_align(wxTemp, LV_ALIGN_CENTER, 26, -20);

  wxCond = lv_label_create(s);
  lv_obj_set_style_text_font(wxCond, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(wxCond, accent, 0);
  lv_obj_align(wxCond, LV_ALIGN_CENTER, 0, 26);

  wxHi = lv_label_create(s);
  lv_obj_set_style_text_font(wxHi, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(wxHi, accent, 0);
  lv_obj_align(wxHi, LV_ALIGN_CENTER, -30, 54);

  wxLo = lv_label_create(s);
  lv_obj_set_style_text_font(wxLo, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(wxLo, accent, 0);
  lv_obj_align(wxLo, LV_ALIGN_CENTER, 30, 54);

  wxNoData1 = lv_label_create(s);
  lv_obj_set_style_text_font(wxNoData1, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(wxNoData1, accent, 0);
  lv_label_set_text(wxNoData1, "-- NO DATA --");
  lv_obj_align(wxNoData1, LV_ALIGN_CENTER, 0, -10);

  wxNoData2 = lv_label_create(s);
  lv_obj_set_style_text_font(wxNoData2, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(wxNoData2, accent, 0);
  lv_label_set_long_mode(wxNoData2, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(wxNoData2, 190);
  lv_label_set_text(wxNoData2, "needs a weather app feeding Gadgetbridge, e.g. Breezy Weather");
  lv_obj_set_style_text_align(wxNoData2, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(wxNoData2, LV_ALIGN_CENTER, 0, 40);

  clusterLabel[SCREEN_WEATHER] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_WEATHER, accent);
}

void updateWeatherScreen(int battPct) {
  bool showWeather = hasWeather && !weatherTooStale();
  lv_obj_t *liveWidgets[] = {wxIcon, wxTemp, wxCond, wxHi, wxLo};
  lv_obj_t *noDataWidgets[] = {wxNoData1, wxNoData2};
  for (lv_obj_t *o : liveWidgets) (showWeather ? lv_obj_clear_flag : lv_obj_add_flag)(o, LV_OBJ_FLAG_HIDDEN);
  for (lv_obj_t *o : noDataWidgets) (!showWeather ? lv_obj_clear_flag : lv_obj_add_flag)(o, LV_OBJ_FLAG_HIDDEN);

  if (showWeather) {
    renderWeatherIconSprite(weatherIconFor(weatherCode, weatherTxt), COL_YELLOW);
    lv_image_set_src(wxIcon, &weatherIconImgDsc);

    char tbuf[8];
    snprintf(tbuf, sizeof(tbuf), "%dC", weatherTempC);
    lv_label_set_text(wxTemp, tbuf);

    long age = weatherAgeSec();
    String line = weatherTxt;
    if (age > WEATHER_FRESH_SEC) {
      char ageBuf[16];
      if (age < 3600) snprintf(ageBuf, sizeof(ageBuf), " (%ldm old)", age / 60);
      else snprintf(ageBuf, sizeof(ageBuf), " (%ldh old)", age / 3600);
      line += ageBuf;
    }
    lv_label_set_text(wxCond, line.c_str());

    char hiBuf[16], loBuf[16];
    snprintf(hiBuf, sizeof(hiBuf), "H:%d", weatherHiC);
    snprintf(loBuf, sizeof(loBuf), "L:%d", weatherLoC);
    lv_label_set_text(wxHi, hiBuf);
    lv_label_set_text(wxLo, loBuf);
  }

  updateStatusCluster(SCREEN_WEATHER, battPct);
}

// ---------- screen: NOTIF ----------
// A vertically-scrollable, snap-per-item list instead of pipboy_watch's
// manual "notifIndex + up/down swipe" paging (drawNotifScreen() there) —
// LVGL's native scroll-snap does the paging for free. MAX_NOTIFS cards are
// built once and just shown/hidden/retextedto match notifs[]/notifCount,
// rather than created and destroyed dynamically, to keep memory use fixed
// and predictable.
lv_obj_t *notifHeader, *notifList, *notifCards[MAX_NOTIFS];
lv_obj_t *notifCardSrc[MAX_NOTIFS], *notifCardTitle[MAX_NOTIFS], *notifCardBody[MAX_NOTIFS];
lv_obj_t *notifNone1, *notifNone2;

// Dismiss one notification from the watch. Tells the phone first — format
// per Espruino's Gadgetbridge protocol docs, {"t":"notify","id":N,
// "n":"DISMISS"} — then clears it locally rather than waiting for the
// phone to echo a notify- back, so the card disappears the moment it's
// tapped.
void onNotifDismiss(lv_event_t *e) {
  noteTouchActivity();
  uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (idx >= notifCount) return;

  long id = notifs[idx].id;
  if (id != 0) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"t\":\"notify\",\"id\":%ld,\"n\":\"DISMISS\"}", id);
    sendGadgetbridgeLine(buf);
  }

  // Removed by index rather than through notifRemove(id): an id of 0 means
  // the sender never gave us a usable one, and notifRemove treats that as
  // "drop the newest", which would clear the wrong card here.
  for (uint8_t j = idx; j + 1 < notifCount; j++) notifs[j] = notifs[j + 1];
  notifCount--;
  notifs[notifCount] = Notif(); // release the vacated slot's String memory
  if (notifIndex >= notifCount) notifIndex = notifCount ? notifCount - 1 : 0;
  forceRedraw = true;
}

void buildNotifScreen() {
  lv_obj_t *s = screens[SCREEN_NOTIF] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0x5AAFFF); // COL_BLUE

  notifHeader = lv_label_create(s);
  lv_obj_set_style_text_font(notifHeader, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(notifHeader, accent, 0);
  lv_label_set_text(notifHeader, "NOTIFICATIONS");
  lv_obj_align(notifHeader, LV_ALIGN_TOP_MID, 0, 42); // 42 not 30: the status cluster now sits at the top on this screen

  notifList = lv_obj_create(s);
  lv_obj_remove_style_all(notifList);
  lv_obj_set_size(notifList, 200, 140); // 140 not 150, and nudged down: has to clear the lowered header above and the back button below
  lv_obj_align(notifList, LV_ALIGN_CENTER, 0, 6);
  lv_obj_set_flex_flow(notifList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(notifList, LV_DIR_VER);
  lv_obj_set_scroll_snap_y(notifList, LV_SCROLL_SNAP_CENTER);
  lv_obj_set_style_pad_row(notifList, 10, 0);

  for (uint8_t i = 0; i < MAX_NOTIFS; i++) {
    lv_obj_t *card = notifCards[i] = lv_obj_create(notifList);
    lv_obj_set_size(card, 200, 130);
    lv_obj_set_style_bg_color(card, lv_color_black(), 0);
    lv_obj_set_style_border_color(card, accent, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_pad_all(card, 8, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);

    notifCardSrc[i] = lv_label_create(card);
    lv_obj_set_style_text_font(notifCardSrc[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(notifCardSrc[i], accent, 0);

    notifCardTitle[i] = lv_label_create(card);
    lv_obj_set_style_text_font(notifCardTitle[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(notifCardTitle[i], lv_color_white(), 0);
    lv_label_set_long_mode(notifCardTitle[i], LV_LABEL_LONG_WRAP);
    lv_obj_set_width(notifCardTitle[i], 180);

    notifCardBody[i] = lv_label_create(card);
    lv_obj_set_style_text_font(notifCardBody[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(notifCardBody[i], lv_color_hex(0xAAAAAA), 0);
    lv_label_set_long_mode(notifCardBody[i], LV_LABEL_LONG_WRAP);
    lv_obj_set_width(notifCardBody[i], 180);

    // Dismiss button, pinned to the card's top-right. IGNORE_LAYOUT keeps
    // the card's flex column from treating it as another stacked row.
    lv_obj_t *dismiss = lv_button_create(card);
    lv_obj_add_flag(dismiss, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_size(dismiss, 24, 24);
    lv_obj_set_style_radius(dismiss, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dismiss, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(dismiss, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(dismiss, accent, 0);
    lv_obj_set_style_border_width(dismiss, 1, 0);
    lv_obj_set_style_shadow_width(dismiss, 0, 0);
    lv_obj_set_style_bg_color(dismiss, accent, LV_STATE_PRESSED);
    lv_obj_align(dismiss, LV_ALIGN_TOP_RIGHT, 4, -4);
    lv_obj_add_event_cb(dismiss, onNotifDismiss, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_t *dl = lv_label_create(dismiss);
    lv_obj_set_style_text_font(dl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(dl, accent, 0);
    lv_label_set_text(dl, LV_SYMBOL_CLOSE);
    lv_obj_center(dl);

    lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);
  }

  notifNone1 = lv_label_create(s);
  lv_obj_set_style_text_font(notifNone1, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(notifNone1, accent, 0);
  lv_label_set_text(notifNone1, "-- NONE YET --");
  lv_obj_align(notifNone1, LV_ALIGN_CENTER, 0, -10);

  notifNone2 = lv_label_create(s);
  lv_obj_set_style_text_font(notifNone2, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(notifNone2, accent, 0);
  lv_label_set_text(notifNone2, "pair Gadgetbridge to start receiving these");
  lv_label_set_long_mode(notifNone2, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(notifNone2, 190);
  lv_obj_set_style_text_align(notifNone2, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(notifNone2, LV_ALIGN_CENTER, 0, 30);

  clusterLabel[SCREEN_NOTIF] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_NOTIF, accent);
}

void updateNotifScreen(int battPct) {
  bool any = notifCount > 0;
  (any ? lv_obj_add_flag : lv_obj_clear_flag)(notifNone1, LV_OBJ_FLAG_HIDDEN);
  (any ? lv_obj_add_flag : lv_obj_clear_flag)(notifNone2, LV_OBJ_FLAG_HIDDEN);
  // bleConnected hides the "pair Gadgetbridge" hint once there's nothing
  // left to explain — same as pipboy_watch's drawNotifScreen().
  if (bleConnected) lv_obj_add_flag(notifNone2, LV_OBJ_FLAG_HIDDEN);

  char hdr[24];
  if (notifCount > 1) snprintf(hdr, sizeof(hdr), "NOTIFICATIONS (%u)", notifCount);
  else snprintf(hdr, sizeof(hdr), "NOTIFICATIONS");
  lv_label_set_text(notifHeader, hdr);

  for (uint8_t i = 0; i < MAX_NOTIFS; i++) {
    if (i < notifCount) {
      lv_obj_clear_flag(notifCards[i], LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text(notifCardSrc[i], notifs[i].src.length() ? notifs[i].src.c_str() : "NOTIFICATION");
      lv_label_set_text(notifCardTitle[i], notifs[i].title.c_str());
      lv_label_set_text(notifCardBody[i], notifs[i].body.c_str());
    } else {
      lv_obj_add_flag(notifCards[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  updateStatusCluster(SCREEN_NOTIF, battPct);
}

// ---------- screen: MUSIC ----------
// Deliberately not styled like the other 8 screens (centered stack of
// labels) — this one leans on widgets none of the others use, both because
// a media player genuinely wants them and as a look at what else LVGL
// offers over pipboy_watch's hand-drawn approach:
//   - a progress *ring* (lv_arc) instead of a hand-measured bar
//   - real per-button hit targets (lv_button) for prev/play/next instead
//     of one whole-screen tap standing in for "play/pause" with no way to
//     skip tracks at all
//   - marquee-scrolling text (LV_LABEL_LONG_SCROLL_CIRCULAR) for a track
//     title too long to fit, instead of wrapping into a multi-line block
// "Now Playing" as a header label is gone too — the ring, the track name,
// and the transport buttons already say what this screen is without
// spending a whole row on a title.
lv_obj_t *musicRing;
lv_obj_t *musicTrackLbl, *musicArtistLbl, *musicTimeLbl;
lv_obj_t *musicPrevBtn, *musicPlayBtn, *musicNextBtn, *musicPlayIcon;
lv_obj_t *musicNoneLbl;

void onMusicPrev(lv_event_t *e) { noteTouchActivity(); sendMusicControl("previous"); }
void onMusicNext(lv_event_t *e) { noteTouchActivity(); sendMusicControl("next"); }
void onMusicPlayPause(lv_event_t *e) {
  noteTouchActivity();
  sendMusicControl(musicPlaying ? "pause" : "play");
  musicPlaying = !musicPlaying; // optimistic flip — see sendMusicControl()'s comment
  forceRedraw = true;
}

// A flat, outlined circular icon button — LVGL's default button look is a
// filled 3D-ish rectangle, which doesn't match this app's flat CRT
// aesthetic, but the button widget itself still earns its keep: real press
// feedback and hit-testing for free, which the old whole-screen-tap design
// didn't have at all (there was no way to tell where to tap, or to skip a
// track, only "tap anywhere to toggle play/pause").
lv_obj_t *makeMusicButton(lv_obj_t *parent, const char *symbol, int diameter, lv_color_t accent) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, diameter, diameter);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, accent, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_bg_color(btn, accent, LV_STATE_PRESSED); // visible feedback on press
  lv_obj_t *icon = lv_label_create(btn);
  lv_obj_set_style_text_font(icon, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(icon, accent, 0);
  lv_label_set_text(icon, symbol);
  lv_obj_center(icon);
  return btn;
}

void formatMinSec(long totalSec, char *buf, size_t bufSize) {
  if (totalSec < 0) totalSec = 0;
  snprintf(buf, bufSize, "%ld:%02ld", totalSec / 60, totalSec % 60);
}

void buildMusicScreen() {
  lv_obj_t *s = screens[SCREEN_MUSIC] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0xAF6EFF); // COL_PURPLE — moved off blue so it no longer shares NOTIF's colour

  // Progress ring — purely informational, not draggable: real position
  // comes from Gadgetbridge, not from someone dragging the arc, so the
  // interactive knob/click behaviour lv_arc normally has is stripped.
  musicRing = lv_arc_create(s);
  lv_obj_set_size(musicRing, 224, 224);
  lv_obj_center(musicRing);
  lv_arc_set_rotation(musicRing, 270);
  lv_arc_set_bg_angles(musicRing, 0, 360);
  lv_arc_set_value(musicRing, 0);
  lv_obj_remove_style(musicRing, NULL, LV_PART_KNOB);
  lv_obj_remove_flag(musicRing, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(musicRing, lv_color_hex(0x193B2A), LV_PART_MAIN);
  lv_obj_set_style_arc_width(musicRing, 5, LV_PART_MAIN);
  lv_obj_set_style_arc_color(musicRing, accent, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(musicRing, 5, LV_PART_INDICATOR);

  musicTrackLbl = lv_label_create(s);
  lv_obj_set_style_text_font(musicTrackLbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(musicTrackLbl, lv_color_white(), 0);
  lv_label_set_long_mode(musicTrackLbl, LV_LABEL_LONG_SCROLL_CIRCULAR); // marquee, not a multi-line wrap
  lv_obj_set_width(musicTrackLbl, 160);
  lv_obj_set_style_text_align(musicTrackLbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(musicTrackLbl, LV_ALIGN_CENTER, 0, -42);

  musicArtistLbl = lv_label_create(s);
  lv_obj_set_style_text_font(musicArtistLbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(musicArtistLbl, accent, 0);
  lv_label_set_long_mode(musicArtistLbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_obj_set_width(musicArtistLbl, 160);
  lv_obj_set_style_text_align(musicArtistLbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(musicArtistLbl, LV_ALIGN_CENTER, 0, -18);

  musicTimeLbl = lv_label_create(s);
  lv_obj_set_style_text_font(musicTimeLbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(musicTimeLbl, lv_color_hex(0x196428), 0); // COL_GREEN_DIM
  lv_label_set_text(musicTimeLbl, "");
  lv_obj_align(musicTimeLbl, LV_ALIGN_CENTER, 0, 4);

  musicPrevBtn = makeMusicButton(s, LV_SYMBOL_PREV, 40, accent);
  lv_obj_align(musicPrevBtn, LV_ALIGN_CENTER, -54, 54);
  lv_obj_add_event_cb(musicPrevBtn, onMusicPrev, LV_EVENT_CLICKED, NULL);

  musicNextBtn = makeMusicButton(s, LV_SYMBOL_NEXT, 40, accent);
  lv_obj_align(musicNextBtn, LV_ALIGN_CENTER, 54, 54);
  lv_obj_add_event_cb(musicNextBtn, onMusicNext, LV_EVENT_CLICKED, NULL);

  // Play/pause stays its own filled circle rather than reusing
  // makeMusicButton — it's the primary action, so it's built to stand out
  // (filled, larger) instead of matching prev/next's plain outline.
  musicPlayBtn = lv_button_create(s);
  lv_obj_set_size(musicPlayBtn, 52, 52);
  lv_obj_set_style_radius(musicPlayBtn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(musicPlayBtn, accent, 0);
  lv_obj_set_style_bg_opa(musicPlayBtn, LV_OPA_COVER, 0);
  lv_obj_set_style_shadow_width(musicPlayBtn, 0, 0);
  lv_obj_set_style_bg_color(musicPlayBtn, lv_color_white(), LV_STATE_PRESSED);
  lv_obj_align(musicPlayBtn, LV_ALIGN_CENTER, 0, 54);
  lv_obj_add_event_cb(musicPlayBtn, onMusicPlayPause, LV_EVENT_CLICKED, NULL);
  musicPlayIcon = lv_label_create(musicPlayBtn);
  lv_obj_set_style_text_font(musicPlayIcon, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(musicPlayIcon, lv_color_black(), 0);
  lv_label_set_text(musicPlayIcon, LV_SYMBOL_PLAY);
  lv_obj_center(musicPlayIcon);

  musicNoneLbl = lv_label_create(s);
  lv_obj_set_style_text_font(musicNoneLbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(musicNoneLbl, accent, 0);
  lv_label_set_long_mode(musicNoneLbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(musicNoneLbl, 170);
  lv_obj_set_style_text_align(musicNoneLbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(musicNoneLbl, "Nothing playing");
  lv_obj_center(musicNoneLbl);

  // Moved to the top on this screen only — the button row owns the bottom
  // here, unlike every other screen's fixed bottom cluster. Still the same
  // shared clusterLabel[]/updateStatusCluster() plumbing underneath, just
  // repositioned.
  clusterLabel[SCREEN_MUSIC] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_MUSIC, accent); // also moves the cluster to the top, which this screen wanted anyway
}

void updateMusicScreen(int battPct) {
  lv_obj_t *liveWidgets[] = {musicRing, musicTrackLbl, musicArtistLbl, musicTimeLbl,
                              musicPrevBtn, musicPlayBtn, musicNextBtn};
  for (lv_obj_t *o : liveWidgets) (hasMusic ? lv_obj_clear_flag : lv_obj_add_flag)(o, LV_OBJ_FLAG_HIDDEN);
  (hasMusic ? lv_obj_add_flag : lv_obj_clear_flag)(musicNoneLbl, LV_OBJ_FLAG_HIDDEN);

  if (hasMusic) {
    lv_label_set_text(musicTrackLbl, musicTrack.length() ? musicTrack.c_str() : "(no track info)");
    lv_label_set_text(musicArtistLbl, musicArtist.c_str());
    lv_label_set_text(musicPlayIcon, musicPlaying ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);

    // Position is extrapolated forward from the last known value while
    // playing — Gadgetbridge only pushes it occasionally, not every
    // second — same "derive from a timestamp" approach the stopwatch and
    // timer already use.
    long posSec = musicPositionSec;
    if (musicPlaying) posSec += (millis() - musicPositionStampMs) / 1000;
    if (musicDurSec > 0 && posSec > musicDurSec) posSec = musicDurSec;

    if (musicDurSec > 0) {
      char posBuf[8], durBuf[8], timeBuf[20];
      formatMinSec(posSec, posBuf, sizeof(posBuf));
      formatMinSec(musicDurSec, durBuf, sizeof(durBuf));
      snprintf(timeBuf, sizeof(timeBuf), "%s / %s", posBuf, durBuf);
      lv_label_set_text(musicTimeLbl, timeBuf);
      lv_arc_set_value(musicRing, (int)(posSec * 100 / musicDurSec));
    } else {
      // No duration known yet (e.g. only a bare musicstate has arrived) —
      // an empty ring reads as "no data" rather than a guess.
      lv_label_set_text(musicTimeLbl, "");
      lv_arc_set_value(musicRing, 0);
    }
  }

  updateStatusCluster(SCREEN_MUSIC, battPct);
}

// ---------- screen: STOPWATCH ----------
lv_obj_t *swElapsed, *swHint;

uint32_t stopwatchElapsedMs() {
  return swAccumMs + (swRunning ? (millis() - swStartMs) : 0);
}

void buildStopwatchScreen() {
  lv_obj_t *s = screens[SCREEN_STOPWATCH] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_color_t accent = lv_color_hex(0xFF554B); // COL_RED

  lv_obj_t *title = lv_label_create(s);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(title, accent, 0);
  lv_label_set_text(title, "Stopwatch");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 42); // 42 not 30: the status cluster now sits at the top on this screen

  swElapsed = lv_label_create(s);
  lv_obj_set_style_text_font(swElapsed, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(swElapsed, accent, 0);
  lv_label_set_text(swElapsed, "00:00.0");
  lv_obj_align(swElapsed, LV_ALIGN_CENTER, 0, 0);

  swHint = lv_label_create(s);
  lv_obj_set_style_text_font(swHint, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(swHint, accent, 0);
  lv_label_set_text(swHint, "tap to start");
  lv_obj_align(swHint, LV_ALIGN_CENTER, 0, 44);

  clusterLabel[SCREEN_STOPWATCH] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_STOPWATCH, accent);
}

void updateStopwatchScreen(int battPct) {
  uint32_t ms = stopwatchElapsedMs();
  uint32_t totalSec = ms / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu.%lu", (unsigned long)(totalSec / 60), (unsigned long)(totalSec % 60),
           (unsigned long)((ms % 1000) / 100));
  lv_label_set_text(swElapsed, buf);
  lv_label_set_text(swHint, swRunning ? "tap to stop" : (ms > 0 ? "tap to start / swipe to clear" : "tap to start"));
  updateStatusCluster(SCREEN_STOPWATCH, battPct);
}


// ---------- screen: SETTINGS ----------
// Reached by swiping down on the clock face rather than taking a seventh
// slot in the nav ring — the ring's spacing was settled and adding another
// badge would have meant shrinking or crowding all of them.
//
// Every value here was a compile-time constant before this screen existed;
// each writes through to NVS on change (saveSettings()) so it survives a
// reboot.
lv_obj_t *setBrightVal, *setDimVal, *setScanSw, *set24hSw;

static const uint32_t DIM_PRESETS_MS[] = {10000, 20000, 40000, 60000, 0}; // 0 = never dim
static const uint8_t DIM_PRESET_COUNT = sizeof(DIM_PRESETS_MS) / sizeof(DIM_PRESETS_MS[0]);

void settingsRefreshValues() {
  char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", (int)((cfgBrightness * 100 + 127) / 255));
  lv_label_set_text(setBrightVal, buf);

  if (cfgDimTimeoutMs == 0) snprintf(buf, sizeof(buf), "never");
  else snprintf(buf, sizeof(buf), "%lus", (unsigned long)(cfgDimTimeoutMs / 1000));
  lv_label_set_text(setDimVal, buf);
}

void onBrightAdjust(lv_event_t *e) {
  noteTouchActivity();
  int delta = (int)(intptr_t)lv_event_get_user_data(e);
  int v = (int)cfgBrightness + delta;
  if (v < 20) v = 20;    // floor: below this the panel is effectively unreadable
  if (v > 255) v = 255;
  cfgBrightness = (uint8_t)v;
  display.setBrightness(cfgBrightness); // apply immediately so the change is visible while adjusting
  settingsRefreshValues();
  saveSettings();
}

void onDimCycle(lv_event_t *e) {
  noteTouchActivity();
  uint8_t idx = 0;
  for (uint8_t i = 0; i < DIM_PRESET_COUNT; i++) {
    if (DIM_PRESETS_MS[i] == cfgDimTimeoutMs) { idx = i; break; }
  }
  cfgDimTimeoutMs = DIM_PRESETS_MS[(idx + 1) % DIM_PRESET_COUNT];
  settingsRefreshValues();
  saveSettings();
}

void onScanToggle(lv_event_t *e) {
  noteTouchActivity();
  cfgScanlines = lv_obj_has_state(setScanSw, LV_STATE_CHECKED);
  lv_obj_set_style_bg_image_opa(lv_layer_top(), cfgScanlines ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
  saveSettings();
}

void on24hToggle(lv_event_t *e) {
  noteTouchActivity();
  cfgTime24h = lv_obj_has_state(set24hSw, LV_STATE_CHECKED);
  forceRedraw = true; // clock re-renders on the next refresh pass
  saveSettings();
}

// One row of the settings list: a label on the left, caller's control on
// the right. Returns the row so the caller can parent its control to it.
lv_obj_t *makeSettingsRow(lv_obj_t *parent, const char *label, lv_color_t accent) {
  lv_obj_t *row = lv_obj_create(parent);
  lv_obj_remove_style_all(row);
  lv_obj_set_size(row, 186, 34);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  lv_obj_t *lbl = lv_label_create(row);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, accent, 0);
  lv_label_set_text(lbl, label);
  return row;
}

lv_obj_t *makeSettingsButton(lv_obj_t *parent, const char *text, lv_color_t accent,
                             lv_event_cb_t cb, void *userData) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, 30, 26);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btn, accent, 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_style_bg_color(btn, accent, LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);
  lv_obj_t *l = lv_label_create(btn);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(l, accent, 0);
  lv_label_set_text(l, text);
  lv_obj_center(l);
  return btn;
}

void buildSettingsScreen() {
  lv_obj_t *s = screens[SCREEN_SETTINGS] = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s, lv_color_black(), 0);
  lv_obj_set_scrollbar_mode(s, LV_SCROLLBAR_MODE_OFF);
  lv_color_t accent = lv_color_hex(0x55FF6E); // COL_GREEN

  lv_obj_t *title = lv_label_create(s);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(title, accent, 0);
  lv_label_set_text(title, "Settings");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 42);

  // Scrollable so the list isn't limited by how much fits between the
  // title and the back button on a round panel.
  lv_obj_t *list = lv_obj_create(s);
  lv_obj_remove_style_all(list);
  lv_obj_set_size(list, 200, 118);
  lv_obj_align(list, LV_ALIGN_CENTER, 0, 8);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_scroll_dir(list, LV_DIR_VER);
  lv_obj_set_style_pad_row(list, 4, 0);

  lv_obj_t *row = makeSettingsRow(list, "Brightness", accent);
  lv_obj_t *brightBox = lv_obj_create(row);
  lv_obj_remove_style_all(brightBox);
  lv_obj_set_size(brightBox, 104, 28);
  lv_obj_set_flex_flow(brightBox, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(brightBox, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(brightBox, 4, 0);
  makeSettingsButton(brightBox, "-", accent, onBrightAdjust, (void *)(intptr_t)-25);
  setBrightVal = lv_label_create(brightBox);
  lv_obj_set_style_text_font(setBrightVal, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(setBrightVal, lv_color_white(), 0);
  makeSettingsButton(brightBox, "+", accent, onBrightAdjust, (void *)(intptr_t)25);

  row = makeSettingsRow(list, "Dim after", accent);
  lv_obj_t *dimBtn = lv_button_create(row);
  lv_obj_set_size(dimBtn, 74, 26);
  lv_obj_set_style_radius(dimBtn, 13, 0);
  lv_obj_set_style_bg_color(dimBtn, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(dimBtn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(dimBtn, accent, 0);
  lv_obj_set_style_border_width(dimBtn, 1, 0);
  lv_obj_set_style_shadow_width(dimBtn, 0, 0);
  lv_obj_set_style_bg_color(dimBtn, accent, LV_STATE_PRESSED);
  lv_obj_add_event_cb(dimBtn, onDimCycle, LV_EVENT_CLICKED, NULL);
  setDimVal = lv_label_create(dimBtn);
  lv_obj_set_style_text_font(setDimVal, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(setDimVal, lv_color_white(), 0);
  lv_obj_center(setDimVal);

  row = makeSettingsRow(list, "Scanlines", accent);
  setScanSw = lv_switch_create(row);
  lv_obj_set_size(setScanSw, 44, 22);
  lv_obj_set_style_bg_color(setScanSw, accent, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(setScanSw, onScanToggle, LV_EVENT_VALUE_CHANGED, NULL);
  if (cfgScanlines) lv_obj_add_state(setScanSw, LV_STATE_CHECKED);

  row = makeSettingsRow(list, "24-hour time", accent);
  set24hSw = lv_switch_create(row);
  lv_obj_set_size(set24hSw, 44, 22);
  lv_obj_set_style_bg_color(set24hSw, accent, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(set24hSw, on24hToggle, LV_EVENT_VALUE_CHANGED, NULL);
  if (cfgTime24h) lv_obj_add_state(set24hSw, LV_STATE_CHECKED);

  settingsRefreshValues();

  clusterLabel[SCREEN_SETTINGS] = buildStatusCluster(s, accent);
  addBackNav(s, SCREEN_SETTINGS, accent);
}

// ---------- refresh ----------
// Updates every screen's dynamic content each tick (steps, time, battery,
// etc.) rather than only the currently-visible one — the LVGL-retained-mode
// equivalent of pipboy_watch's render(), just without needing to track
// "which screen is dirty": relabeling an off-screen widget is cheap (no
// panel traffic happens until that screen is actually shown), so there's
// nothing to gain from skipping the others.
void refreshAllScreens() {
  time_t nowEpoch = time(nullptr);
  struct tm t;
  localtime_r(&nowEpoch, &t);
  int steps = stepCounter.count();
  int battPct = readBatteryPercent();

  updateStatScreen(t, battPct);
  updateDataScreen(steps, battPct);
  updateHistoryScreen(battPct);
  updateWeatherScreen(battPct);
  updateNotifScreen(battPct);
  updateMusicScreen(battPct);
  updateStopwatchScreen(battPct);
}

// ---------- Vault Boy / boot screen ----------
// A real LVGL screen now, not raw LovyanGFX draws — which means LVGL has
// to be up and running *before* the rest of boot runs, not after (see the
// note above lv_init() in setup() for why that's still safe: the
// WiFi-setup-portal and sleep/critical-battery screens keep drawing
// straight to `display`, and that stays conflict-free as long as
// lv_timer_handler() is never pumped during those specific windows, which
// it isn't — LVGL only repaints when explicitly told to, not on a
// background timer).
//
// Two phases, matching pipboy_watch's original two-phase feel (a terminal
// log, then a separate Vault Boy moment) rather than cramming both into
// one screen at once: a RobCo-style loading screen with a status line and
// a segmented "terminal loading bar" (the blocky, discrete-segment look
// Fallout's own terminals use, not a smooth gradient fill) while boot is
// actually doing things, then those widgets hide and Vault Boy gets the
// screen to himself for the power-on flicker reveal.
lv_obj_t *bootScreen;
lv_obj_t *bootHeader, *bootStatus, *bootSegments[10];
lv_obj_t *bootVaultImg;
static const uint8_t BOOT_SEGMENT_COUNT = 10;
static const uint8_t BOOT_TOTAL_STEPS = 6; // display, peripherals, touch, motion, BLE, network

// Vault Boy, reused via the same sprite-bridge trick as the weather icons:
// pipboy_watch's exact 1bpp artwork (traced from a real reference photo,
// see vaultboy_bitmap.h) rendered into an off-screen LovyanGFX sprite, then
// handed to LVGL as a plain image — re-rendered at each flicker brightness
// step the same way renderWeatherIconSprite() re-renders per condition.
static LGFX_Sprite vaultBoySprite(&display);
static lv_image_dsc_t vaultBoyImgDsc;
bool vaultBoySpriteReady = false;

void drawVaultBoy(lgfx::LGFXBase &gfx, int x, int y, uint8_t brightness) {
  uint16_t flickerGreen = lgfx::color565((uint8_t)(85 * brightness / 255), (uint8_t)(255 * brightness / 255), (uint8_t)(110 * brightness / 255));
  gfx.drawBitmap(x, y, VAULTBOY_BITMAP, VAULTBOY_W, VAULTBOY_H, flickerGreen, COL_BG);
}

void renderVaultBoySprite(uint8_t brightness) {
  if (!vaultBoySpriteReady) {
    vaultBoySprite.setColorDepth(16);
    vaultBoySprite.createSprite(VAULTBOY_W, VAULTBOY_H);
    vaultBoySpriteReady = true;
  }
  vaultBoySprite.fillSprite(COL_BG);
  drawVaultBoy(vaultBoySprite, 0, 0, brightness);
  vaultBoyImgDsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  vaultBoyImgDsc.header.w = VAULTBOY_W;
  vaultBoyImgDsc.header.h = VAULTBOY_H;
  vaultBoyImgDsc.header.cf = LV_COLOR_FORMAT_RGB565_SWAPPED; // same LovyanGFX/LVGL byte-order fix as the weather icons
  vaultBoyImgDsc.header.stride = VAULTBOY_W * 2;
  vaultBoyImgDsc.data_size = (uint32_t)VAULTBOY_W * VAULTBOY_H * 2;
  vaultBoyImgDsc.data = (const uint8_t *)vaultBoySprite.getBuffer();
}

void buildBootScreen() {
  bootScreen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(bootScreen, lv_color_black(), 0);
  // LVGL's default LV_SCROLLBAR_MODE_AUTO shows a thin gray scrollbar
  // track the instant it judges any content even marginally close to the
  // object's bounds — nothing here actually scrolls, so that showed up as
  // a stray gray line with no purpose. Off entirely.
  lv_obj_set_scrollbar_mode(bootScreen, LV_SCROLLBAR_MODE_OFF);
  lv_screen_load(bootScreen);

  bootHeader = lv_label_create(bootScreen);
  lv_obj_set_style_text_font(bootHeader, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bootHeader, lv_color_hex(0x55FF6E), 0); // COL_GREEN
  lv_label_set_text(bootHeader, "COGENT TECH (TM)\nTERMLINK PROTOCOL");
  lv_obj_set_style_text_align(bootHeader, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(bootHeader, LV_ALIGN_TOP_MID, 0, 46);

  bootStatus = lv_label_create(bootScreen);
  lv_obj_set_style_text_font(bootStatus, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bootStatus, lv_color_hex(0x55FF6E), 0);
  // Width-capped with CLIP (truncate, no wrap) as a hard backstop — the
  // status strings are kept short enough in practice to never need it, but
  // an unconstrained label can run right off the round panel's edge with
  // no warning, which is exactly what happened before this was added.
  lv_obj_set_width(bootStatus, 200);
  lv_label_set_long_mode(bootStatus, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_align(bootStatus, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(bootStatus, "INITIALIZING...");
  lv_obj_align(bootStatus, LV_ALIGN_CENTER, 0, 10);

  int segW = 14, gap = 3, totalW = BOOT_SEGMENT_COUNT * segW + (BOOT_SEGMENT_COUNT - 1) * gap;
  int x0 = -totalW / 2 + segW / 2;
  for (uint8_t i = 0; i < BOOT_SEGMENT_COUNT; i++) {
    bootSegments[i] = lv_obj_create(bootScreen);
    lv_obj_remove_style_all(bootSegments[i]); // drop the default border/padding/scrollbar styling
    lv_obj_set_size(bootSegments[i], segW, 10);
    lv_obj_set_style_bg_color(bootSegments[i], lv_color_hex(0x193B2A), 0); // unlit — COL_GREEN_DIM-ish
    lv_obj_set_style_bg_opa(bootSegments[i], LV_OPA_COVER, 0);
    lv_obj_align(bootSegments[i], LV_ALIGN_CENTER, x0 + i * (segW + gap), 50);
  }

  bootVaultImg = lv_image_create(bootScreen);
  lv_obj_add_flag(bootVaultImg, LV_OBJ_FLAG_HIDDEN); // shown only for playBootAnimation()'s reveal
  lv_obj_center(bootVaultImg);

  lv_timer_handler(); // pump once so this initial paint actually reaches the panel
}

// Advances the status line and lights up progress segments proportional to
// `step` out of BOOT_TOTAL_STEPS. Not called at all on a sleep-wake (see
// setup()'s wokeFromSleep guards, same spirit as the old bootLog() ones) —
// that path is meant to feel instant, not narrate what it's doing.
void updateBootStatus(const char *status, uint8_t step) {
  Serial.printf("[pipboy] %s\n", status);
  lv_label_set_text(bootStatus, status);
  uint8_t lit = (uint8_t)((uint32_t)step * BOOT_SEGMENT_COUNT / BOOT_TOTAL_STEPS);
  if (lit > BOOT_SEGMENT_COUNT) lit = BOOT_SEGMENT_COUNT;
  for (uint8_t i = 0; i < BOOT_SEGMENT_COUNT; i++) {
    lv_obj_set_style_bg_color(bootSegments[i], i < lit ? lv_color_hex(0x55FF6E) : lv_color_hex(0x193B2A), 0);
  }
  lv_timer_handler();
  delay(110); // same "long enough to actually read as it goes by" pacing bootLog() used
}

// Vault Boy's power-on flicker reveal — same flicker character as before
// (a couple of quick dim/bright pulses before it settles, not a smooth
// fade — "instead of a fabricated second 'eyes open' frame", per
// pipboy_watch's original reasoning), just driven through the sprite
// bridge and lv_timer_handler() instead of raw draws.
void playBootAnimation() {
  for (uint8_t i = 0; i < BOOT_SEGMENT_COUNT; i++) lv_obj_add_flag(bootSegments[i], LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(bootHeader, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(bootStatus, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(bootVaultImg, LV_OBJ_FLAG_HIDDEN);

  uint8_t flickerSteps[] = {60, 20, 140, 60, 255};
  for (uint8_t b : flickerSteps) {
    renderVaultBoySprite(b);
    lv_image_set_src(bootVaultImg, &vaultBoyImgDsc);
    lv_timer_handler();
    delay(90);
  }
  delay(400);
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[pipboy] boot");

  bool wokeFromSleep = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);
  if (wokeFromSleep) Serial.println("[pipboy] waking from deep sleep (touch)...");

  setenv("TZ", POSIX_TZ, 1);
  tzset();

  setCpuFrequencyMhz(80);
  setupPowerManagement();

  Serial.println("[pipboy] display.init()...");
  bool dispOk = display.init();
  Serial.printf("[pipboy] display.init() -> %d\n", dispOk);
  display.setRotation(1); // 90° clockwise — see the touch-remap comment up top for what this costs on the touch side
  display.setBrightness(cfgBrightness);
  lastActivityMs = millis();
  nextFlickerMs = lastActivityMs + 4000 + random(4000);

  display.startWrite();
  display.fillScreen(COL_BG);
  display.endWrite();

  // LVGL starts here now, before the rest of boot, instead of after it —
  // needed so the boot screen itself (status line, progress bar, Vault Boy
  // reveal) can be built from real widgets. Still safe for the same reason
  // it always was: the WiFi-setup-portal and sleep/critical-battery
  // screens keep drawing straight to `display`, and that stays
  // conflict-free as long as lv_timer_handler() is never pumped during
  // those specific windows — which it isn't; LVGL only repaints when
  // explicitly told to; there's no background timer that could sneak a
  // repaint in over a raw draw.
  lv_init();
  lv_tick_set_cb(millis);
  lvDisplay = lv_display_create(240, 240);
  lv_display_set_color_format(lvDisplay, LV_COLOR_FORMAT_RGB565);
  static uint8_t lvBuf1[240 * 40 * 2];
  static uint8_t lvBuf2[240 * 40 * 2];
  lv_display_set_buffers(lvDisplay, lvBuf1, lvBuf2, sizeof(lvBuf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(lvDisplay, lvglFlush);
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvglTouchRead);

  buildBootScreen();
  if (!wokeFromSleep) updateBootStatus("INITIALIZING...", 1);
  else lv_timer_handler(); // still pump once so the boot screen's first paint actually reaches the panel

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  if (!wokeFromSleep) updateBootStatus("SCANNING BUS...", 2);
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[pipboy]   found device at 0x%02X\n", addr);
    }
  }

  if (!wokeFromSleep) updateBootStatus("TOUCH CALIBRATION...", 3);
  for (int attempt = 1; attempt <= 4 && !touchOk; attempt++) {
    touchOk = touch.begin(Wire, PIN_TOUCH_RST, PIN_TOUCH_INT);
    Serial.printf("[pipboy] touch.begin() attempt %d -> %d\n", attempt, touchOk);
    if (!touchOk) delay(150);
  }

  if (!wokeFromSleep) updateBootStatus("MOTION CALIBRATION...", 4);
  Serial.println("[pipboy] probing IMU...");
  imuOk = imu.begin(Wire, QMI8658_ADDRESS_HIGH) || imu.begin(Wire, QMI8658_ADDRESS_LOW);
  Serial.printf("[pipboy] imu.begin() -> %d\n", imuOk);
  if (imuOk) {
    imu.enableAccel(true);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_4G);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setAccelUnit_mps2(true);
    if (GYRO_ENABLED) {
      imu.enableGyro(true);
      imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
      imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
      imu.setGyroUnit_dps(true);
    }
  }

  loadState();

  int battMv = readBatteryMilliVolts();
  if (battMv >= BATT_MIN_PLAUSIBLE_MV && battMv <= BATT_CRITICAL_MV) {
    enterCriticalBatterySleep(battMv); // never returns
  }

  if (!wokeFromSleep) updateBootStatus("BLE UPLINK...", 5);
  setupBLE();

  bool wifiOk;
  if (wokeFromSleep) {
    timeSynced = (time(nullptr) > 1700000000);
    if (wifiResyncDue(WAKE_RESYNC_INTERVAL_SEC)) {
      Serial.println("[pipboy] wake: resync due, trying a quick WiFi reconnect...");
      rtcLastSyncAttemptEpoch = time(nullptr);
      bool resynced = syncTimeViaWifi(8000);
      wifiOk = resynced || timeSynced;
    } else {
      wifiOk = timeSynced;
      Serial.println("[pipboy] wake: recently synced, skipping WiFi.");
    }
    playBootAnimation();
  } else {
    updateBootStatus("TIME SYNC...", 6);
    wifiOk = wifiConnectOrSetup();
    rtcLastSyncAttemptEpoch = time(nullptr);
    updateBootStatus(wifiOk ? "UPLINK ESTABLISHED" : "OFFLINE MODE", 6);
    delay(350);
    playBootAnimation();
  }

  // Boot screen's done its job — free it (and the sprite buffer behind
  // it, ~73KB, worth reclaiming since nothing needs Vault Boy again until
  // the next boot) before building the real 9-screen UI.
  lv_obj_delete(bootScreen);
  if (vaultBoySpriteReady) {
    vaultBoySprite.deleteSprite();
    vaultBoySpriteReady = false;
  }

  buildStatScreen();
  buildDataScreen();
  buildHistoryScreen();
  buildWeatherScreen();
  buildNotifScreen();
  buildMusicScreen();
  buildStopwatchScreen();
  buildSettingsScreen();
  buildScanlineOverlay();
  lv_obj_set_style_bg_image_opa(lv_layer_top(), cfgScanlines ? LV_OPA_COVER : LV_OPA_TRANSP, 0);

  for (uint8_t i = 0; i < NUM_SCREENS; i++) {
    lv_obj_add_event_cb(screens[i], onScreenPressed, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(screens[i], onScreenPressing, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(screens[i], onScreenReleased, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(screens[i], onScreenGesture, LV_EVENT_GESTURE, NULL);
  }

  refreshAllScreens();
  lv_screen_load(screens[SCREEN_STAT]);
  currentScreen = SCREEN_STAT;

  if (!wokeFromSleep) delay(200);
  Serial.println("[pipboy] PIP-BOY 3000 online (LVGL).");
}

void loop() {
  uint32_t now = millis();

  // Poll the IMU at ~50Hz and feed the pedometer (and, while dimmed,
  // raise-to-wake). Unchanged from pipboy_watch.ino.
  if (imuOk && now - lastImuPollMs >= 20) {
    lastImuPollMs = now;
    float ax, ay, az;
    bool accelOk = imu.readAccelMPS2(ax, ay, az);
    if (accelOk) {
      stepCounter.update(ax, ay, az);
    }
    if (GYRO_ENABLED) {
      imu.readGyro(gyroX, gyroY, gyroZ);
      if (backlightDimmed) updateRaiseToWake(accelOk, ax, ay, az, now);
      if (IMU_TRACE && now - lastImuTraceMs >= 1000) {
        lastImuTraceMs = now;
        Serial.printf("[pipboy] imu a=(%.1f,%.1f,%.1f) g=(%.0f,%.0f,%.0f) dim=%d\n",
                      ax, ay, az, gyroX, gyroY, gyroZ, backlightDimmed);
      }
    }
  }

  // Day rollover — unchanged from pipboy_watch.ino.
  if (timeSynced) {
    time_t nowEpoch = time(nullptr);
    struct tm lt;
    localtime_r(&nowEpoch, &lt);
    uint32_t today = (uint32_t)((lt.tm_year + 1900) * 10000u + (lt.tm_mon + 1) * 100u + lt.tm_mday);
    if (lastDayMarker != 0 && today != lastDayMarker) {
      uint32_t finished = stepCounter.count();
      for (uint8_t i = 0; i + 1 < 6; i++) stepHist[i] = stepHist[i + 1];
      stepHist[5] = finished;
      Serial.printf("[pipboy] new day (%lu -> %lu), resetting steps (yesterday: %lu)\n",
                    (unsigned long)lastDayMarker, (unsigned long)today, (unsigned long)finished);
      stepCounter.set(0);
      lastDayMarker = today;
      saveState();
    }
    lastDayMarker = today;
  }

  // Touch input, screen navigation, and every gesture/tap action now flow
  // through LVGL's own input device + the onScreen* event callbacks
  // registered in setup() — nothing left to poll here.
  lv_timer_handler();

  // Re-snapshot now: lv_timer_handler() may just have run onScreenPressed()/
  // onScreenPressing(), which stamp lastActivityMs with their own fresh
  // millis() read. That read happens *after* the `now` captured at the top
  // of this function, so on a touch, lastActivityMs can end up larger than
  // the stale `now` — and since both are unsigned, `now - lastActivityMs`
  // below would underflow to a huge number and immediately trip the idle
  // timeout. Confirmed live (not theoretical): a serial capture showed
  // "idle timeout -> deep sleep" firing within ~8 real seconds of a touch,
  // nowhere near the real 10-minute threshold. Re-reading here guarantees
  // now >= lastActivityMs for every check below that depends on it.
  now = millis();

  // A notification that popped itself up automatically reverts to the
  // clock face after a few seconds — unchanged behaviour, LVGL-native
  // navigation.
  if (notifAutoShown && now > notifAutoRevertMs) {
    notifAutoShown = false;
    goToScreen(SCREEN_STAT);
  }

  // Idle-dim, CRT flicker, and auto-sleep — unchanged from pipboy_watch.ino;
  // none of this is LVGL's concern, it's backlight/power management
  // sitting below the UI layer either way.
  if (touchOk && !backlightDimmed && cfgDimTimeoutMs && now - lastActivityMs > cfgDimTimeoutMs) {
    display.setBrightness(BRIGHT_DIM);
    backlightDimmed = true;
  }

  if (!backlightDimmed && now >= nextFlickerMs) {
    display.setBrightness(cfgBrightness > 40 ? cfgBrightness - 40 : cfgBrightness / 2);
    delay(30);
    display.setBrightness(cfgBrightness);
    nextFlickerMs = now + 4000 + random(4000);
  }

  if (touchOk && AUTO_SLEEP_TIMEOUT_MS && !swRunning &&
      now - lastActivityMs > AUTO_SLEEP_TIMEOUT_MS) {
    Serial.println("[pipboy] idle timeout -> deep sleep");
    enterDeepSleep(); // never returns
  }

  // Keep the BLE "steps" characteristic fresh — unchanged.
  static uint32_t lastStepsPublish = 0;
  if (now - lastStepsPublish > 2000) {
    lastStepsPublish = now;
    if (stepsChar) {
      char buf[12];
      snprintf(buf, sizeof(buf), "%lu", (unsigned long)stepCounter.count());
      stepsChar->setValue((uint8_t *)buf, strlen(buf));
    }
  }

  // Report activity to Gadgetbridge so steps land in its own charts, not
  // just on the watch. Format per Espruino's Gadgetbridge protocol docs:
  //   {"t":"act","ts":<ms since epoch>,"stp":<steps>,"hrm":0,"mov":0,"rt":0}
  // `stp` is sent as the count accrued *since the last report*, not the
  // running daily total — Gadgetbridge stores these as per-interval
  // activity samples, so a cumulative value would read as a huge spike
  // every minute. (That reading of the field is from the docs' wording
  // rather than something confirmed against Gadgetbridge's charts, so if
  // the totals there look wrong, this is the line to revisit.)
  //
  // Only sent once the clock is trustworthy: a sample timestamped from an
  // unsynced clock would be filed under 1970 and pollute the history.
  static uint32_t lastActReportMs = 0;
  static uint32_t lastActSteps = 0;
  if (bleConnected && timeSynced && now - lastActReportMs > 60000) {
    uint32_t stepsNow = stepCounter.count();
    // A day rollover (or a manual reset) takes the count backwards; report
    // nothing for that interval rather than a negative-turned-huge delta.
    uint32_t delta = (stepsNow >= lastActSteps) ? (stepsNow - lastActSteps) : 0;
    lastActReportMs = now;
    lastActSteps = stepsNow;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"t\":\"act\",\"ts\":%llu000,\"stp\":%lu,\"hrm\":0,\"mov\":0,\"rt\":0}",
             (unsigned long long)time(nullptr), (unsigned long)delta);
    sendGadgetbridgeLine(buf);
  }

  // Refresh every screen's dynamic content once/second, or immediately when
  // forceRedraw asks for it, or faster while the stopwatch is running and
  // visible — same cadence pipboy_watch's render()/forceRedraw used, just
  // driving refreshAllScreens() instead of a full immediate-mode repaint.
  time_t nowSec = time(nullptr);
  struct tm t;
  localtime_r(&nowSec, &t);
  bool fastTick = (currentScreen == SCREEN_STOPWATCH && swRunning && now - lastFastDrawMs >= 100);
  if (forceRedraw || t.tm_sec != lastRenderedSecond || fastTick) {
    refreshAllScreens();
    lastRenderedSecond = t.tm_sec;
    lastFastDrawMs = now;
    forceRedraw = false;
  }

  // Armed step-reset prompt expiry — unchanged.
  if (stepsResetArmedMs && now - stepsResetArmedMs >= STEPS_RESET_ARM_MS) {
    stepsResetArmedMs = 0;
    stepsResetTaps = 0;
    forceRedraw = true;
  }

  // Persist steps every 30s — unchanged.
  if (now - lastSaveMs > 30000) {
    lastSaveMs = now;
    saveState();
  }

  // Periodic WiFi resync — unchanged.
  if (wifiResyncDue()) {
    rtcLastSyncAttemptEpoch = time(nullptr);
    syncTimeViaWifi(10000);
  }

  delay(5);
}
