# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Arduino (ESP32-S3) firmware for a Waveshare **ESP32-S3-Touch-LCD-1.28** (round 240x240 GC9A01, CST816S touch,
QMI8658 IMU, no RTC, no power button) styled after Fallout's green terminal look. Repo: github.com/Cogential/ESP32-S3-Touch-LCD-1.28-watch (remote `origin`).
`README.md` is the detailed design writeup (features, power/battery behaviour, open questions); read the section
for a sketch before changing it. Project memory holds the hardware-confirmed gotchas for `esp32-watch-lvgl/`.

## The three sketches

Each folder is a self-contained Arduino sketch (one big `.ino` plus headers). There is no shared library code:
`LGFX_Config.hpp`, `Touch_CST816S.h`, `StepCounter.h`, `vaultboy_bitmap.h` and `lv_conf.h` are **copied** per sketch,
so a fix in one copy does not reach the others.

- `esp32-watch-original/`: the original firmware, hand-drawn on raw LovyanGFX. The user treats it as "keep as-is"; don't
  change it unless asked. It still has TIMER, FLASHLIGHT and swipe navigation.
- `esp32-watch-lvgl/`: the active firmware. Same back end (WiFi/NTP + WiFiManager portal, BLE Gadgetbridge parser, battery
  curve, NVS, steps, raise-to-wake, deep sleep) reused nearly verbatim from `esp32-watch-original.ino`, with a new LVGL v9.5
  screen layer. It has diverged on purpose (curved nav bands on STAT, SETTINGS screen, no timer/flashlight), so don't assume
  the two match.
- `lvgl_watch/`: minimal LVGL proof of concept (arc + counting label, no real time). Reference only.

`esp32-watch-lvgl.ino` is organised in `// ---------- <section> ----------` blocks (BLE, WiFi/NTP, battery, power, LVGL
bridge, melt, gestures, one block per `screen: X`, setup/loop); grep those headers to navigate. Points that span
sections:
- LVGL owns the panel only while `lv_timer_handler()` is pumped. The WiFi-portal, sleep and critical-battery screens
  draw straight to `display` with LovyanGFX, which is safe only because LVGL isn't pumped during those windows.
- `lv_init()` is at the top of `setup()` because boot is itself an LVGL screen, and `initMelt()` runs alongside the
  display setup so the boot screen can be snapshotted before Vault Boy's sprite is freed.
- Gadgetbridge sees the watch as a Bangle.js: the BLE name `"Bangle.js esp32-watch"` must stay matching its
  allowlist, and messages are Bangle.js/Nordic-UART JSON (`{"t":...}`).

## Build / flash

`arduino-cli` is the project-local `./bin/arduino-cli` (1.1.1, gitignored). Platform and library versions are pinned
in each sketch's `sketch.yaml` (esp32 core 3.3.11, LovyanGFX 1.2.27, LVGL 9.5.0, ArduinoJson, QMI8658, WiFiManager).
There are no tests or linters: verification means compiling, then flashing and checking on the board.

```
# esp32-watch-lvgl needs PSRAM=enabled (the melt transition's snapshots live in PSRAM)
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M,PSRAM=enabled esp32-watch-lvgl
./bin/arduino-cli board list    # port is usually /dev/ttyACM0
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M,PSRAM=enabled --upload -p /dev/ttyACM0 esp32-watch-lvgl

# esp32-watch-original / lvgl_watch: same, without PSRAM=enabled
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M esp32-watch-original
```

- The `sketch.yaml` `fqbn:` line is the source of truth. `esp32-watch-lvgl` needs `PSRAM=enabled`; without it the melt
  can't allocate and silently falls back to LVGL slides.
- `FlashSize=16M` must stay with the `app3M_fat9M_16MB` partition scheme, or the board boot-loops.
- Serial logs: `arduino-cli monitor` exits right away in this non-interactive shell. Use
  `stty -F /dev/ttyACM0 115200 raw -echo` then `timeout N cat /dev/ttyACM0`.
- Wi-Fi credentials come from the WiFiManager captive portal ("esp32-watch-setup"); there is no secrets.h.

## Rules from the 2026-10-20 review fixes (esp32-watch-lvgl)

- **BLE callbacks never parse or touch UI state.** `NUSRxCallbacks` only appends to `nusRxBuffer` under
  `nusRxMutex`; `drainGadgetbridgeRx()` in `loop()` parses. Anything that changes `notifs[]`, music/weather
  strings or the screen must stay on the loop task.
- **Change screens only with `goToScreen()`**, never by setting `currentScreen` alone. It skips a melt when the
  target is already showing.
- **In `meltPushFrame`, clamp `colY` to >= 0** before indexing `meltOldPx`; waiting strips have negative offsets.
- Review items still open (see /data/cogent/reviews/2026-10-20/review-smartwatch.md): #5 esp32-watch-original music
  command format, #6 slide fallback is a no-op when the new snapshot fails (restore the previous screen first,
  but only when `captureOld`, because the boot screen is already deleted), #7 scanlines missing during the melt,
  #9 extra final melt frame, #10 melt memory/`waitDMA` cleanup. (#8, the step-history rollover, went away with
  the steps screens.)

## esp32-watch-lvgl changes on 2026-10-21

- **Steps, 7-day history and the stopwatch were removed by request** (screens, pedometer, BLE steps
  characteristic, Gadgetbridge `act` reports, NVS step keys, which `loadState()` deletes). Don't bring them
  back unasked. Screens are now STAT, WEATHER, NOTIF, MUSIC, SETTINGS; STAT's nav is three curved bands
  around the edge (`makeNavBand()`: music left, notifications top, weather right). The IMU is only polled while dimmed (raise-to-wake is its only user).
- **Battery gauge** (`// ---------- battery gauge ----------`): filtered and rate-limited, with the full point
  learned from the highest filtered reading (NVS `battMax`, clamped 4050-4200mV) because this board reads a
  full cell low. The critical-battery check in `setup()` deliberately still uses the raw reading. The serial
  log line `battery: raw .. filt .. full .. -> X% (showing Y%)` is what to capture if the gauge misbehaves.
  There's no charge detection; the only real fix for that is wiring the charger's status signal to a spare
  GPIO (15/16/17/18/21).
