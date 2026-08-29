// PIP-BOY 3000 watch face for the Waveshare ESP32-S3-Touch-LCD-1.28
//
// - Round GC9A01 display, green-phosphor CRT look, four screens (tap to
//   cycle): STAT (clock + weather one-liner), DATA (steps), WEATHER (full
//   temp/hi-lo/condition), NOTIF (latest phone notification).
// - Step counting from the onboard QMI8658 accelerometer; the gyro is read
//   too (gyroX/Y/Z) but not yet consumed by anything.
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
//   Touch CST816S : INT5 RST4  addr 0x15
//   IMU   QMI8658  : INT13     addr 0x6A/0x6B
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

#include "LGFX_Config.hpp"
#include "Touch_CST816S.h"
#include "StepCounter.h"
#include "vaultboy_bitmap.h"

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
static const uint32_t LONG_PRESS_MS = 3000; // hold this long, finger still, to trigger either one
// POSIX TZ string for America/Toronto — handles EST/EDT DST automatically.
static const char *POSIX_TZ = "EST5EDT,M3.2.0,M11.1.0/2";
// Tracked in seconds of real wall-clock time (not millis()) because millis()
// resets to 0 on every deep-sleep wake — see rtcLastSyncAttemptEpoch below.
static const time_t WIFI_RESYNC_INTERVAL_SEC = 4 * 60 * 60; // 4h

// ---------- power saving ----------
static const uint8_t BRIGHT_ACTIVE = 180;
static const uint8_t BRIGHT_DIM = 12;
static const uint32_t IDLE_DIM_TIMEOUT_MS = 8000; // dim backlight after 8s of no touch

// ---------- swipe navigation ----------
// The display is rotated 90° in software (see setRotation(1) in setup()),
// but the touch chip reports raw, unrotated panel coordinates — so the
// raw axis that corresponds to "visual left/right" is the *other* raw
// axis from what you'd naively expect, and possibly flipped in sign.
// These two knobs are the one-line fix if a swipe feels backwards or
// diagonal once tested on the real hardware:
static const bool SWIPE_AXIS_IS_RAW_Y = true;  // true: raw Y delta = visual horizontal swipe
static const bool SWIPE_INVERT = false;        // flip if left/right come out reversed
static const int16_t SWIPE_MIN_DIST = 35;      // px of raw movement to count as a swipe, not a tap

// ---------- pins ----------
static const uint8_t PIN_I2C_SDA   = 6;
static const uint8_t PIN_I2C_SCL   = 7;
static const uint8_t PIN_TOUCH_RST = 4;
static const uint8_t PIN_TOUCH_INT = 5;
static const uint8_t PIN_IMU_INT   = 13;
static const uint8_t PIN_BATT_ADC  = 1;

// ---------- Pip-Boy palette ----------
static const uint16_t COL_BG        = lgfx::color565(0, 0, 0);
static const uint16_t COL_GREEN     = lgfx::color565(85, 255, 110);   // bright phosphor
static const uint16_t COL_GREEN_DIM = lgfx::color565(25, 95, 40);     // secondary text
static const uint16_t COL_AMBER     = lgfx::color565(255, 170, 60);   // fault/status accent
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
uint8_t currentScreen = 0; // 0 = STAT (clock), 1 = DATA (steps), 2 = WEATHER, 3 = NOTIF
const uint8_t NUM_SCREENS = 4;
const uint8_t SCREEN_WEATHER = 2;
const uint8_t SCREEN_NOTIF = 3;
bool prevTouchDown = false;   // debounced/stable state used for edge detection
uint8_t touchDownStreak = 0;  // consecutive raw "down" reads in a row
uint32_t lastTouchPollMs = 0;
int16_t touchStartX = 0, touchStartY = 0; // where the current gesture began
int16_t touchLastX = 0, touchLastY = 0;   // most recent sample of the current gesture
bool touchWasDimWake = false; // this gesture's first touch woke a dimmed screen — don't also navigate
uint32_t touchDownStartMs = 0; // when the current gesture's stable "down" began, for long-press timing
bool longPressConsumed = false; // this gesture already fired its long-press action — skip the swipe check on lift
bool forceRedraw = true;
uint32_t notifAutoRevertMs = 0; // when a popped-up notification should hide itself
bool notifAutoShown = false;

// ---------- notification + weather state (from Gadgetbridge) ----------
String notifSrc = "";
String notifTitle = "";
String notifBody = "";
bool hasNotif = false;

bool hasWeather = false;
int weatherTempC = 0;
int weatherHiC = 0;
int weatherLoC = 0;
String weatherTxt = "";

// Latest gyro reading (deg/sec), updated every IMU poll alongside the accel
// feed into StepCounter. Not consumed by anything yet — available for
// whatever reads it next.
float gyroX = 0, gyroY = 0, gyroZ = 0;

uint32_t lastImuPollMs = 0;
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
int lastRenderedSecond = -1;

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

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConnected = true; forceRedraw = true; }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    forceRedraw = true;
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
    notifSrc = String((const char *)(doc["src"] | ""));
    notifTitle = String((const char *)(doc["title"] | ""));
    notifBody = String((const char *)(doc["body"] | ""));
    hasNotif = true;
    Serial.printf("[pipboy] notify: %s / %s\n", notifSrc.c_str(), notifTitle.c_str());

    // Pop the notification screen up for a few seconds, like a real watch.
    currentScreen = SCREEN_NOTIF;
    notifAutoShown = true;
    notifAutoRevertMs = millis() + 8000;
    display.setBrightness(BRIGHT_ACTIVE);
    backlightDimmed = false;
    lastActivityMs = millis();
    forceRedraw = true;

  } else if (strcmp(type, "weather") == 0) {
    float kelvin = doc["temp"] | 0.0f;
    weatherTempC = (int)roundf(kelvin - 273.15f);
    weatherHiC = (int)roundf((float)(doc["hi"] | kelvin) - 273.15f);
    weatherLoC = (int)roundf((float)(doc["lo"] | kelvin) - 273.15f);
    weatherTxt = String((const char *)(doc["txt"] | ""));
    hasWeather = true;
    forceRedraw = true;
    Serial.printf("[pipboy] weather: %dC (%s)\n", weatherTempC, weatherTxt.c_str());
  }
  // Other message types (notify-, music, findPhone, etc.) are ignored for now.
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
  BLECharacteristic *txChar = nus->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
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

// ---------- WiFi / NTP time sync ----------
// True if it's been over WIFI_RESYNC_INTERVAL_SEC of real wall-clock time
// since the last resync *attempt* (successful or not — see
// rtcLastSyncAttemptEpoch), or if the clock isn't valid yet at all. Callers
// should stamp rtcLastSyncAttemptEpoch = time(nullptr) right before they
// actually attempt a sync, win or lose, so a stretch with no WiFi in range
// retries at a steady interval instead of every single wake.
bool wifiResyncDue() {
  if (!timeSynced) return true; // no valid clock yet — always worth trying
  return (time(nullptr) - rtcLastSyncAttemptEpoch) > WIFI_RESYNC_INTERVAL_SEC;
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

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[pipboy] WiFi connect failed/timed out.");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println("[pipboy] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);

  WiFi.disconnect(true, true);
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
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString("join hotspot:", 120, 112);
  display.setTextColor(COL_GREEN, COL_BG);
  display.drawString(WIFI_SETUP_AP_NAME, 120, 134);
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString("on your phone, then", 120, 160);
  display.drawString("follow the prompt to", 120, 180);
  display.drawString("pick your WiFi", 120, 200);
  display.endWrite();
}

// First-boot (or nothing valid saved) path: tries saved credentials first —
// fast, no AP — and only pops the setup hotspot if that fails. Call once,
// at boot.
bool wifiConnectOrSetup() {
  WiFiManager wm;
  wm.setConnectTimeout(15);       // seconds to try saved creds before giving up
  wm.setConfigPortalTimeout(180); // don't let the portal hang boot forever if no one responds
  wm.setAPCallback([](WiFiManager *) { drawWifiSetupScreen(); });

  bool connected = wm.autoConnect(WIFI_SETUP_AP_NAME);
  if (!connected) {
    Serial.println("[pipboy] WiFi setup portal timed out with no connection.");
    forceRedraw = true;
    return false;
  }

  Serial.println("[pipboy] WiFi connected, fetching NTP time...");
  bool gotTime = fetchNtpTime(8000);
  WiFi.disconnect(true, true);
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

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  bool connected = wm.startConfigPortal(WIFI_SETUP_AP_NAME);

  bool gotTime = false;
  if (connected) {
    Serial.println("[pipboy] WiFi connected, fetching NTP time...");
    gotTime = fetchNtpTime(8000);
    WiFi.disconnect(true, true);
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
int readBatteryPercent() {
  static int cachedPct = -1;
  static uint32_t lastReadMs = 0;
  uint32_t now = millis();
  if (cachedPct < 0 || now - lastReadMs > 30000) {
    int mv = analogReadMilliVolts(PIN_BATT_ADC) * 3; // 200k/100k divider
    float pct = (mv - 3300) / (4200.0f - 3300.0f) * 100.0f; // rough LiPo curve
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    cachedPct = (int)(pct + 0.5f);
    lastReadMs = now;
  }
  return cachedPct;
}

// ---------- persistence ----------
void loadState() {
  prefs.begin("pipboy", false);
  stepCounter.begin(prefs.getUInt("steps", 0));
  lastDayMarker = prefs.getUInt("dayMarker", 0);
  prefs.end();
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
  prefs.end();
  lastSavedSteps = steps;
  lastSavedDayMarker = lastDayMarker;
}

// ---------- power ----------
// This board has no physical power switch, so this is the closest
// equivalent: a real ESP32 deep sleep (CPU, display, radios all powered
// down — this is what actually saves battery, unlike the backlight-only
// dim the idle timeout does) that wakes back up on the next touch. Deep
// sleep is a full reset — everything except the RTC timer and flash-backed
// state loses power, so the only way back is through setup() again from the
// top. See the wake-from-sleep fast path near the top of setup().
void enterDeepSleep() {
  Serial.println("[pipboy] long-press on DATA screen -> entering deep sleep...");
  saveState(); // catch any steps counted since the last periodic save

  display.startWrite();
  display.fillScreen(COL_BG);
  display.endWrite();
  display.setBrightness(0);
  delay(50); // let the backlight PWM actually settle to off before power drops

  // CST816S holds its INT pin high and pulses it low on a touch event —
  // ext0 wakes the chip the moment that pin goes low. GPIO5 is within the
  // S3's low-power IO domain, which ext0 requires.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TOUCH_INT, 0);
  esp_deep_sleep_start();
  // Never reaches here — the reset vector (setup()) is the only way out.
}

// ---------- drawing helpers ----------

// Sharp bold monospace face for everything except the time (which stays on
// Font7 for the 7-segment digital-clock look) — the old bitmap fonts read
// too soft/rounded up close. Always set font+size together so there's no
// risk of a leftover scale from one screen leaking into another.
void useSharpSmallFont() { display.setFont(&fonts::FreeMonoBold9pt7b); display.setTextSize(0.75f); }
void useSharpMedFont()   { display.setFont(&fonts::FreeMonoBold12pt7b); display.setTextSize(1.0f); }

// Bluetooth rune, traced from the real logo (spine + one zigzag flag) and
// drawn with anti-aliased lines instead of the old plain on/off dot.
void drawBluetoothIcon(int cx, int cy, bool connected) {
  const float w = 8, h = 14;
  float x0 = cx - w / 2, y0 = cy - h / 2;
  uint16_t col = connected ? COL_GREEN : COL_GREEN_DIM;
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
void drawBatteryIcon(int x, int y, int pct) {
  const int BW = 20, BH = 11, RAD = 3, NUBW = 2, NUBH = 5, BORDER = 2, INSET = 2;
  display.fillSmoothRoundRect(x, y, BW, BH, RAD, COL_GREEN_DIM);
  display.fillSmoothRoundRect(x + BORDER, y + BORDER, BW - 2 * BORDER, BH - 2 * BORDER, RAD - BORDER, COL_BG);
  display.fillRect(x + BW, y + (BH - NUBH) / 2, NUBW, NUBH, COL_GREEN_DIM);

  int innerW = BW - 2 * BORDER - 2 * INSET;
  int fw = innerW * pct / 100;
  if (fw < 0) fw = 0;
  if (fw > innerW) fw = innerW;
  if (fw > 0) {
    uint16_t col = pct > 20 ? COL_GREEN : COL_AMBER;
    display.fillRect(x + BORDER + INSET, y + BORDER + INSET, fw, BH - 2 * BORDER - 2 * INSET, col);
  }
}

// Bluetooth + battery + percentage, centered together as one group — the
// same exact position/style on every screen. Width measured at runtime so
// it stays centered regardless of whether the percentage is 1, 2, or 3
// digits.
void drawBleBattCluster(int battPct) {
  display.setTextDatum(middle_center);
  useSharpMedFont();
  char pctbuf[8];
  snprintf(pctbuf, sizeof(pctbuf), "%d%%", battPct);
  int pctW = display.textWidth(pctbuf);
  const int BT_W = 8, GAP1 = 10, BATT_W = 22, GAP2 = 5;
  int totalW = BT_W + GAP1 + BATT_W + GAP2 + pctW;
  int startX = 120 - totalW / 2;
  int rowY = 213;

  drawBluetoothIcon(startX + BT_W / 2, rowY, bleConnected);
  int battX = startX + BT_W + GAP1;
  drawBatteryIcon(battX, rowY - 5, battPct);
  display.setTextDatum(middle_left);
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString(pctbuf, battX + BATT_W + GAP2, rowY);
  display.setTextDatum(middle_center);
}

void drawStatScreen(struct tm &t, int steps, int battPct) {
  display.setTextDatum(middle_center);

  // Time — centered in the middle of the screen, everything else stacks
  // below it.
  int hour12 = t.tm_hour % 12;
  if (hour12 == 0) hour12 = 12;
  char timebuf[16];
  snprintf(timebuf, sizeof(timebuf), "%d:%02d:%02d", hour12, t.tm_min, t.tm_sec);
  display.setTextColor(COL_GREEN, COL_BG);
  display.setFont(&fonts::Font7);
  display.setTextSize(0.85f);
  display.drawString(timebuf, 120, 118);
  display.setTextSize(1.0f);

  // Everything below uses a sharp bold monospace face instead of Font7
  // (the time's font is 7-segment-style and can't render letters at all).
  display.setTextColor(COL_GREEN_DIM, COL_BG);

  char datebuf[32];
  static const char *wd[7] = {"Sun.","Mon.","Tue.","Wed.","Thu.","Fri.","Sat."};
  static const char *mo[12] = {"January","February","March","April","May","June","July","August","September","October","November","December"};
  if (timeSynced) {
    snprintf(datebuf, sizeof(datebuf), "%s %s %d, %d", wd[t.tm_wday], mo[t.tm_mon], t.tm_mday, t.tm_year + 1900);
  } else {
    snprintf(datebuf, sizeof(datebuf), "-- NO SIGNAL --");
  }
  useSharpSmallFont();
  display.drawString(datebuf, 120, 168);

  char weatherbuf[28];
  if (hasWeather) {
    snprintf(weatherbuf, sizeof(weatherbuf), "%dC  HI%d LO%d  %s", weatherTempC, weatherHiC, weatherLoC, weatherTxt.c_str());
  } else {
    snprintf(weatherbuf, sizeof(weatherbuf), "                    "); // blank until Gadgetbridge sends one
  }
  display.drawString(weatherbuf, 120, 192);

  drawBleBattCluster(battPct);
}

void drawDataScreen(int steps, int battPct) {
  display.setTextDatum(middle_center);

  useSharpMedFont();
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString("Steps", 120, 80);

  char stepbuf[12];
  snprintf(stepbuf, sizeof(stepbuf), "%lu", (unsigned long)steps);
  display.setTextColor(COL_GREEN, COL_BG);
  display.setFont(&fonts::Font7);
  display.drawString(stepbuf, 120, 120);

  float km = steps * 0.00075f; // ~0.75m average stride
  char distbuf[24];
  snprintf(distbuf, sizeof(distbuf), "~%.2f KM", km);
  useSharpSmallFont();
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString(distbuf, 120, 168);

  drawBleBattCluster(battPct);
}

// Fuller weather view than the compact line embedded in STAT — same data
// (hasWeather/weatherTempC/etc., fed over BLE by Gadgetbridge, see
// handleGadgetbridgeLine()), laid out like DATA's big-number-plus-detail
// style instead of squeezed into one line.
void drawWeatherScreen(int battPct) {
  display.setTextDatum(middle_center);

  useSharpMedFont();
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString("Weather", 120, 80);

  if (!hasWeather) {
    display.setTextColor(COL_GREEN_DIM, COL_BG);
    display.drawString("-- NO DATA --", 120, 120);
    useSharpSmallFont();
    display.drawString("pair Gadgetbridge to", 120, 148);
    display.drawString("enable weather sync", 120, 168);
  } else {
    char tempbuf[12];
    snprintf(tempbuf, sizeof(tempbuf), "%dC", weatherTempC);
    display.setTextColor(COL_GREEN, COL_BG);
    display.setFont(&fonts::Font7);
    display.setTextSize(0.85f);
    display.drawString(tempbuf, 120, 120);
    display.setTextSize(1.0f);

    useSharpSmallFont();
    display.setTextColor(COL_GREEN_DIM, COL_BG);
    char hiloBuf[24];
    snprintf(hiloBuf, sizeof(hiloBuf), "HI %d   LO %d", weatherHiC, weatherLoC);
    display.drawString(hiloBuf, 120, 168);

    if (weatherTxt.length()) {
      display.setTextColor(COL_GREEN, COL_BG);
      display.drawString(weatherTxt, 120, 190);
    }
  }

  drawBleBattCluster(battPct);
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

void drawNotifScreen(int battPct) {
  display.setTextDatum(middle_center);
  useSharpSmallFont();
  display.setTextColor(COL_GREEN_DIM, COL_BG);
  display.drawString("NOTIFICATION", 120, 46);

  if (!hasNotif) {
    display.setTextColor(COL_GREEN_DIM, COL_BG);
    display.drawString("-- NONE YET --", 120, 120);
    display.drawString("pair Gadgetbridge to", 120, 142);
    display.drawString("start receiving these", 120, 162);
  } else {
    display.setTextColor(COL_GREEN, COL_BG);
    display.drawString(notifSrc.length() ? notifSrc : "NOTIFICATION", 120, 70);

    useSharpSmallFont();
    display.setTextColor(COL_GREEN, COL_BG);
    drawWrapped(notifTitle, 92, 190, 16, 2);

    display.setTextColor(COL_GREEN_DIM, COL_BG);
    drawWrapped(notifBody, 134, 190, 16, 4);
  }

  drawBleBattCluster(battPct);
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

// Small progress bar below the mascot, replacing the old status text.
// `ok=false` renders it amber instead of green (used for the final state
// if WiFi/NTP sync failed).
void drawLoadingBar(int pct, bool ok = true) {
  const int BW = 120, BH = 8, BORDER = 2;
  int x = 120 - BW / 2, y = 206;
  display.fillSmoothRoundRect(x, y, BW, BH, 3, COL_GREEN_DIM);
  display.fillSmoothRoundRect(x + BORDER, y + BORDER, BW - 2 * BORDER, BH - 2 * BORDER, 1, COL_BG);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  int fw = (BW - 2 * BORDER) * pct / 100;
  if (fw > 0) {
    display.fillRect(x + BORDER, y + BORDER, fw, BH - 2 * BORDER, ok ? COL_GREEN : COL_AMBER);
  }
}

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
  delay(250);

  drawLoadingBar(10);
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

  if (currentScreen == 0) drawStatScreen(t, steps, battPct);
  else if (currentScreen == 1) drawDataScreen(steps, battPct);
  else if (currentScreen == SCREEN_WEATHER) drawWeatherScreen(battPct);
  else drawNotifScreen(battPct);
  display.endWrite();
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[pipboy] boot");

  // Deep sleep (enterDeepSleep(), triggered by a long-press on the DATA
  // screen — this board's only "power button") is a full reset: everything
  // except the RTC timer and flash-backed state loses power, and setup()
  // is the only way back. Detect that case so wake can skip the slow parts
  // (flicker animation, WiFi+NTP round trip) instead of paying full boot
  // cost every time you just want to glance at the time.
  bool wokeFromSleep = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);
  if (wokeFromSleep) Serial.println("[pipboy] waking from deep sleep (touch)...");

  // Load the DST-aware timezone rule unconditionally, independent of
  // WiFi — BLE time sync (from Gadgetbridge, see handleGadgetbridgeLine)
  // can now set the clock without ever touching WiFi, and it needs this
  // rule in place to convert the UTC it receives into correct local time.
  setenv("TZ", POSIX_TZ, 1);
  tzset();

  // This app is nowhere near CPU-bound (a clock face + light I2C polling),
  // so run at the lowest clock WiFi still tolerates reliably instead of
  // the default 240MHz — meaningfully less power for identical behavior.
  setCpuFrequencyMhz(80);

  Serial.println("[pipboy] display.init()...");
  bool dispOk = display.init();
  Serial.printf("[pipboy] display.init() -> %d\n", dispOk);
  display.setRotation(1); // 90° clockwise
  display.setBrightness(BRIGHT_ACTIVE);
  lastActivityMs = millis();

  // Boot splash drawn immediately, independent of everything else below, so
  // the panel/SPI path is proven working before we touch I2C at all — but
  // skip the multi-second flicker-in on a sleep wake, where it'd just be an
  // annoying delay between "touched the watch" and "screen shows something".
  if (wokeFromSleep) {
    display.startWrite();
    display.fillScreen(COL_BG);
    display.endWrite();
  } else {
    playBootAnimation();
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  // (WiFi/NTP sync happens further down, after touch+IMU are probed, so
  //  the ~15s WiFi wait doesn't hide sensor problems behind a blank pause.)
  Wire.setClock(400000);
  Wire.setTimeOut(50); // ms — never let a stuck bus hang setup()

  Serial.println("[pipboy] scanning I2C bus...");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[pipboy]   found device at 0x%02X\n", addr);
    }
  }
  Serial.println("[pipboy] scan done.");
  drawLoadingBar(25);

  Serial.println("[pipboy] probing touch...");
  for (int attempt = 1; attempt <= 4 && !touchOk; attempt++) {
    touchOk = touch.begin(Wire, PIN_TOUCH_RST, PIN_TOUCH_INT);
    Serial.printf("[pipboy] touch.begin() attempt %d -> %d\n", attempt, touchOk);
    if (!touchOk) delay(150);
  }
  drawLoadingBar(35);

  Serial.println("[pipboy] probing IMU...");
  imuOk = imu.begin(Wire, QMI8658_ADDRESS_HIGH) || imu.begin(Wire, QMI8658_ADDRESS_LOW);
  Serial.printf("[pipboy] imu.begin() -> %d\n", imuOk);
  if (imuOk) {
    imu.enableAccel(true);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_4G);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setAccelUnit_mps2(true);

    // Gyro is read but not yet acted on — plumbed in for future gesture/
    // orientation features (e.g. rejecting wrist-twist from step counts,
    // raise-to-wake, auto-flip for the other wrist). 512dps covers a sharp
    // wrist flick with headroom; drop to 256dps if finer resolution is
    // wanted later and violent motion isn't a concern.
    imu.enableGyro(true);
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.setGyroUnit_dps(true);
  }
  drawLoadingBar(45);

  loadState();
  Serial.println("[pipboy] starting BLE...");
  setupBLE();
  drawLoadingBar(60);

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
    if (wifiResyncDue()) {
      Serial.println("[pipboy] wake: resync due, trying a quick WiFi reconnect...");
      rtcLastSyncAttemptEpoch = time(nullptr); // stamp before attempting — throttles retries even on failure
      bool resynced = syncTimeViaWifi(8000);   // saved creds only, no portal — same as loop()'s periodic resync
      wifiOk = resynced || timeSynced;         // a failed attempt doesn't invalidate a clock that was already fine
    } else {
      wifiOk = timeSynced;
      Serial.println("[pipboy] wake: recently synced, skipping WiFi.");
    }
  } else {
    wifiOk = wifiConnectOrSetup();
    rtcLastSyncAttemptEpoch = time(nullptr);

    // wifiConnectOrSetup() may have taken over the whole screen for the
    // setup portal — redraw the boot background underneath the final
    // loading-bar state so that screen doesn't show through.
    display.startWrite();
    display.fillScreen(COL_BG);
    drawVaultBoy((240 - VAULTBOY_W) / 2, 6, 255);
    display.endWrite();
    drawLoadingBar(100, wifiOk);
  }

  if (!dispOk) {
    useSharpSmallFont();
    display.setTextColor(COL_AMBER, COL_BG);
    display.drawString("DISPLAY INIT FAILED", 120, 222);
  }
  if (!touchOk) {
    useSharpSmallFont();
    display.setTextColor(COL_AMBER, COL_BG);
    display.drawString("NO TOUCH", 80, 222);
  }
  if (!imuOk) {
    useSharpSmallFont();
    display.setTextColor(COL_AMBER, COL_BG);
    display.drawString("NO IMU", 170, 222);
  }

  // Skip the settle delay on a sleep wake — the whole point of that path is
  // getting back to something useful on screen as fast as possible.
  if (!wokeFromSleep) delay(600);
  Serial.println("[pipboy] PIP-BOY 3000 online.");
}

void loop() {
  uint32_t now = millis();

  // Poll the IMU at ~50Hz and feed the pedometer.
  if (imuOk && now - lastImuPollMs >= 20) {
    lastImuPollMs = now;
    float ax, ay, az;
    if (imu.readAccelMPS2(ax, ay, az)) {
      stepCounter.update(ax, ay, az);
    }
    imu.readGyro(gyroX, gyroY, gyroZ); // dps, per setGyroUnit_dps() above
  }

  // Day rollover: once we know real wall-clock time, reset the step
  // count the first time we notice the calendar day has changed.
  if (timeSynced) {
    uint32_t today = (uint32_t)(time(nullptr) / 86400);
    if (lastDayMarker != 0 && today != lastDayMarker) {
      stepCounter.set(0);
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

    if (stableDown && !prevTouchDown) {
      // Gesture just started.
      touchStartX = touchLastX = tx;
      touchStartY = touchLastY = ty;
      touchWasDimWake = backlightDimmed;
      touchDownStartMs = now;
      longPressConsumed = false;
      if (backlightDimmed) {
        display.setBrightness(BRIGHT_ACTIVE);
        backlightDimmed = false;
        forceRedraw = true;
      }
      lastActivityMs = now;
    } else if (stableDown && prevTouchDown) {
      // Gesture in progress — track the latest position for when it ends.
      touchLastX = tx;
      touchLastY = ty;
      lastActivityMs = now;

      // Long-press-and-hold (finger essentially stationary) does one of two
      // screen-dependent things — both deliberately disruptive/hard to
      // trigger by accident, so they're gated behind a hold, not a tap:
      //   STAT (0) -> reopen the WiFi setup portal
      //   DATA (1) -> deep sleep (this board's only "power button")
      // Every other screen: long-press does nothing.
      if (!touchWasDimWake && !longPressConsumed && (currentScreen == 0 || currentScreen == 1)) {
        int16_t driftX = touchLastX - touchStartX;
        int16_t driftY = touchLastY - touchStartY;
        bool heldStill = abs(driftX) < 15 && abs(driftY) < 15;
        if (heldStill && now - touchDownStartMs >= LONG_PRESS_MS) {
          longPressConsumed = true;
          if (currentScreen == 1) {
            enterDeepSleep(); // never returns — next code that runs is setup()
          }
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
    } else if (!stableDown && prevTouchDown) {
      // Finger lifted — decide if this was a swipe (unless the hold above
      // already consumed this gesture as a long-press).
      if (!touchWasDimWake && !longPressConsumed) {
        int16_t rawDx = touchLastX - touchStartX;
        int16_t rawDy = touchLastY - touchStartY;
        int16_t visualSwipe = SWIPE_AXIS_IS_RAW_Y ? rawDy : rawDx;
        if (SWIPE_INVERT) visualSwipe = -visualSwipe;

        if (abs(visualSwipe) >= SWIPE_MIN_DIST) {
          if (visualSwipe > 0) currentScreen = (currentScreen + 1) % NUM_SCREENS;
          else currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
          notifAutoShown = false; // user's driving now, stop the auto-revert timer
          forceRedraw = true;
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

  // Dim the backlight after a few seconds of no touch — it's the single
  // biggest power draw on this board, and a watch face doesn't need to
  // stay at full brightness when nobody's looking at it.
  if (touchOk && !backlightDimmed && now - lastActivityMs > IDLE_DIM_TIMEOUT_MS) {
    display.setBrightness(BRIGHT_DIM);
    backlightDimmed = true;
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
  // keeps things flicker-free without needing a second framebuffer. And
  // skip it entirely while the backlight is dimmed to near-invisible: a
  // redraw nobody can see burns CPU/SPI time for zero benefit, ~86,400
  // times a day whenever the watch sits idle. Nothing is lost by
  // deferring — waking always sets forceRedraw=true, so the very next
  // frame after a touch is already guaranteed correct.
  time_t nowSec = time(nullptr);
  struct tm t;
  localtime_r(&nowSec, &t);
  if (!backlightDimmed && (forceRedraw || t.tm_sec != lastRenderedSecond)) {
    render(forceRedraw);
    lastRenderedSecond = t.tm_sec;
    forceRedraw = false;
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
