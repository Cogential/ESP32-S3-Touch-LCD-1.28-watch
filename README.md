# smartwatch

Firmware for a Waveshare **ESP32-S3-Touch-LCD-1.28** (round 240×240 GC9A01
panel, CST816S capacitive touch, QMI8658 accel+gyro, no RTC, no physical
power button) turned into a Fallout Pip-Boy 3000-style wrist watch.

## Projects in this repo

- **`pipboy_watch/`** — the working Pip-Boy watch face firmware. Hand-drawn
  monochrome CRT-green UI, built directly on LovyanGFX with no GUI toolkit.
  This is the "keep as-is" project — see below.
- **`lvgl_watch/`** — a separate, minimal LVGL v9 proof-of-concept on the
  same hardware. Not a second watch app (no WiFi/BLE/steps/sleep) — just
  enough to prove the render + touch pipeline works and show what a real
  widget toolkit looks like on this panel. Confirmed running on the actual
  board.
- **`pipboy_lvgl/`** — a full port of every `pipboy_watch/` feature onto
  LVGL widgets. See [below](#pipboy_lvgl--full-lvgl-port) for what's reused
  vs. rebuilt and what's still unverified. Compiles and flashes; not yet
  confirmed correct on real hardware.
- See [Exploring a richer UI](#exploring-a-richer-ui) below for the
  reasoning behind both. Nothing here has touched `pipboy_watch/` — it's
  unchanged.

## `pipboy_watch/` — feature summary

- **Screens** (swipe to cycle): STAT (clock + date + weather one-liner +
  notification banner), DATA (today's steps), HISTORY (7-day step bar
  chart), WEATHER (full conditions), NOTIF (notification list), MUSIC
  (now-playing from the phone), STOPWATCH, TIMER, FLASHLIGHT (solid white,
  max brightness).
- **Time**: WiFi + NTP on first boot (WiFiManager captive portal —
  "PipBoy-Setup" — if no saved network), then WiFi is switched off. Periodic
  resync while awake; a lighter, shorter-threshold resync on waking from
  deep sleep (see "Power" below). Also settable over BLE — Gadgetbridge
  pushes real time on every connect, so the clock stays correct even with
  no WiFi in range at all.
- **Steps**: accelerometer-magnitude pedometer (`StepCounter.h`). Resets at
  local midnight; the day that just ended is pushed into a 6-day rolling
  history (persisted in NVS) for the HISTORY screen's bar chart.
- **Raise-to-wake**: gyro+accel heuristic, calibrated against logged
  readings from the real board (see the comments above
  `updateRaiseToWake()` in `pipboy_watch.ino` — two earlier calibration
  attempts got the axis/orientation wrong before this one was measured
  correctly).
- **Bluetooth**: BLE peripheral named "Bangle.js Pip-Boy 3000" — speaks the
  open Bangle.js/Nordic-UART JSON protocol that Gadgetbridge (free,
  open-source Android app) already knows how to talk to. Add it in
  Gadgetbridge as a Bangle.js watch to get notifications, weather, and
  now-playing music forwarded with no custom phone app. Also exposes a
  small custom "Pip-Boy Time Service" for a companion web page to set the
  clock directly.
- **Music control**: MUSIC screen shows track/artist from Gadgetbridge's
  `musicinfo`/`musicstate` messages; tapping sends a play/pause command
  back over BLE. The *receive* side (showing what's playing) is
  well-grounded — same message format as weather/notifications. The
  *send* side (`sendMusicControl()`) is **not verified against a real
  Gadgetbridge session** — flagged in its own comment in the code. Worth
  testing with a phone actually playing music before trusting it.
- **Power**:
  - No physical power switch: a 3-6s still long-press on any screen puts
    the board into a real ESP32 deep sleep (`enterDeepSleep()`), waking on
    the next touch. A 6-12s hold instead reopens the WiFi setup portal.
  - Backlight dims after 8s idle; auto-sleep after 10 minutes idle (held
    off while a stopwatch/timer is running, or the flashlight screen is
    up).
  - **Critical-battery auto-shutdown**: on every boot, before spending any
    current on WiFi/BLE/the wake animation, the firmware takes one battery
    reading. At or below 3350mV it shows a brief "BATTERY LOW" message and
    goes straight back to sleep. Added after the watch once needed a
    physical battery-connector reconnect to come back from a full deep
    discharge — the goal is to never let the cell get that low again. See
    `BATT_CRITICAL_MV` / `enterCriticalBatterySleep()`.
  - Vault Boy's power-on flicker animation plays on *every* wake now, not
    just cold boot — a deliberate cost against the "fast wake" design (adds
    ~0.85s to every wake), traded for wake feeling like the watch turning
    on rather than the screen just un-dimming.
- **Battery gauge**: ADC on GPIO1 through a 200k/100k divider, averaged over
  8 samples, mapped through a non-linear LiPo discharge curve (not a
  straight line — most of a cell's usable life sits in the 3.7-4.0V
  plateau, which a linear map compresses badly). Reports "--" rather than a
  fabricated percentage when the reading isn't plausible as a real cell
  (e.g. no battery connected at all). These are readings taken *under
  load*, not true resting voltage — see the comment above
  `readBatteryMilliVolts()` for why no compensation is applied.
- **CRT flicker**: a faint, random backlight brightness dip every 4-8s
  while the screen is active — the scanline overlay that used to go with
  it was removed (didn't look good on the real hardware).

### Known pending items

- **Physical power button**: not yet wired. Rear header exposes
  GPIO15/16/17/18/21 (not GPIO33 — not RTC-capable, can't wake from deep
  sleep). Needs `esp_sleep_enable_ext1_wakeup()` with `ESP_EXT1_WAKEUP_ANY_LOW`
  once the button exists — see the TODO comment above
  `powerDownAndDeepSleep()`.
- **Battery charging ceiling**: the displayed percentage has been observed
  capping in the high 80s while charging. Two live hypotheses, not yet
  distinguished: (a) normal voltage-under-charging-current effect (would
  mean the cell is actually fine), or (b) a systematic offset in the
  ADC/divider-to-millivolt calculation, since the code assumes an exact
  200k/100k divider and a perfectly-calibrated ADC with no correction for
  either. Needs an independent multimeter reading compared against the
  Serial-logged `battery: XXXXmV` value to tell which.
- R15 charge-current resistor swap (160K → ~330K, so the charger targets
  ~500mA into a cell rated for that instead of ~1A) has been done by the
  user on the physical board. Not independently verified by inspection —
  taken as reported.

## Build / flash

```
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M pipboy_watch
./bin/arduino-cli board list                      # find the port, e.g. /dev/ttyACM0
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M --upload -p /dev/ttyACM0 pipboy_watch
```

Library/platform versions are pinned in `pipboy_watch/sketch.yaml`.
`FlashSize=16M` must stay paired with the `PartitionScheme` — the
bootloader's own flash-size field doesn't otherwise pick up the board's
real 16MB chip, and the mismatch boot-loops it.

## Exploring a richer UI

The stock example software this board ships with (Waveshare's
`LVGL_Arduino` demo) uses **LVGL v8.3.10** on top of **TFT_eSPI v2.5.34** to
get a benchmark-style widget UI with real animations, anti-aliased fonts,
and image support — a different tier of UI than `pipboy_watch/`'s
hand-drawn screens.

Recommended path for a second, separate project in this repo:

- **Keep LovyanGFX as the panel driver**, don't switch to TFT_eSPI. The
  pin configuration in `pipboy_watch/LGFX_Config.hpp` is already verified
  against real hardware (schematic-checked, not guessed) — re-deriving it
  under a different driver library would be repeating exactly the kind of
  hardware guess that's bitten this project before. LVGL doesn't care which
  low-level driver feeds it; it only needs a flush callback that pushes a
  rendered buffer to the panel, which LovyanGFX (or a small sprite/DMA
  wrapper around it) can supply directly.
- **LVGL v8.3.x, not v9.x**, unless a v9-only feature is actually needed —
  v8.3 is what most tutorials, and Waveshare's own demo, target; v9
  changed enough of the API that a lot of existing examples don't carry
  over directly.
- A lighter-weight alternative to adopting LVGL at all: LovyanGFX itself
  can decode and draw PNG/JPG/BMP images (`drawPng()`/`drawJpg()`) and
  supports sprite-based off-screen buffers for tear-free animation
  (`LGFX_Sprite`) — enough to add real images and smoother motion to a
  hand-coded UI without bringing in a full widget toolkit. Less capable
  than LVGL (no built-in widgets, theming, or layout engine — still
  hand-laid-out screens), but a smaller, better-understood dependency
  surface if the goal is mainly "images and smooth animation" rather than
  a full app-style UI.

### `lvgl_watch/` — what's actually there

Went with full LVGL. **Compiles clean** (23% flash, 39% RAM — LVGL's
render buffers account for the jump from `pipboy_watch`'s 15% RAM, still
comfortable headroom either way) and **confirmed running on the real
board** — this proof-of-concept was the basis for going ahead with the
full `pipboy_lvgl/` port below.

- LVGL **v9.5.0** (the version Arduino's Library Manager actually serves
  now — see the note in `lvgl_watch/sketch.yaml` for why v9 was chosen
  over v8.3, which most existing tutorials still target).
- `LGFX_Config.hpp` and `Touch_CST816S.h` are copied unchanged from
  `pipboy_watch/` — same already-verified pin wiring, not re-derived.
- `lv_conf.h` is LVGL's own stock template with exactly two changes: the
  top `#if 0` flipped to `#if 1` to enable it, and the Montserrat 32/48
  fonts turned on for a legible clock face. Everything else is default.
- Demo content: an anti-aliased progress arc (tap it — it swaps color,
  which proves touch is actually hitting the widget under your finger, not
  just registering *somewhere* on the panel) and a big proportional-font
  clock label. The clock is **not wired to real time** — no WiFi/NTP/BLE
  in this sketch — it just counts up from `millis()` to prove live label
  updates render cleanly. A real integration would reuse pipboy_watch's
  own time-sync code rather than reinvent it.
- **Known limitation, not yet solved**: the panel is left unrotated
  (`display.setRotation(1)` from `pipboy_watch` is *not* called here).
  `Touch_CST816S.h` reports raw, unrotated panel coordinates, and getting
  those to line up with a rotated display needs a coordinate remap that
  pipboy_watch never had to solve (it only reads swipe deltas and tap/hold
  duration, never an absolute tap position). Rotating this to match
  pipboy_watch's upright orientation is follow-up work.

Build/flash the same way as `pipboy_watch/`, substituting the folder name:

```
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M lvgl_watch
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M --upload -p <port> lvgl_watch
```

### `pipboy_lvgl/` — full LVGL port

Started as a port of every `pipboy_watch/` screen onto LVGL widgets, and
has since diverged into its own design. **Confirmed working on the real
board** — touch, navigation, every screen, and the Gadgetbridge
integration have all been tested live, not just compiled. See
[[pipboy-lvgl-port]] in project memory for the real, hardware-confirmed
bugs found and fixed along the way (an integer-underflow sleep-on-touch
bug, an LVGL/LovyanGFX RGB565 byte-order mismatch, a touch-coordinate
Y-inversion that made every button unclickable, and the verified
Gadgetbridge music protocol).

**Seven screens**: STAT (clock), DATA (steps), HISTORY (7-day chart),
WEATHER, NOTIF, MUSIC, STOPWATCH. TIMER and FLASHLIGHT were removed by
request — `pipboy_watch/` still has both if either needs reviving.

**Navigation is entirely button-driven** — swipe-to-cycle is gone. STAT
carries a ring of six round nav badges on a ±100° arc across the top
(spacing derived from the geometry: 40px badges need ≥42px between
centres, so a 200° span at r=92 puts them ~63px apart), and every other
screen has an identical back button at bottom centre. Swiping up on the
clock face opens SETTINGS, which is deliberately not in the nav ring.
Screens change with the **melt transition** described below. The only other
surviving gesture is the stopwatch's swipe-to-clear, which was never
screen-to-screen navigation.

**Screen melt transition** (`playMelt()`): LVGL ships eight slide variants
and a fade, with no hook for anything per-column, so this is hand-rolled —
the 1993 Doom screen melt, where the outgoing screen sags away in ragged
vertical strips to reveal the next one behind it. Doom's column-seeding is
reproduced exactly (each strip's start offset is a random *walk* from its
neighbour's, which is what makes the edge read as sagging wax instead of
noise); the fall ramp is steepened, since Doom's pacing assumed a 35fps
software renderer and would put a transition near a full second here.

The implementation deliberately avoids LVGL: doing it as 20 animated image
widgets would dirty the whole screen every frame and drag the entire
compositor — scanline overlay included — through a full recomposite. Both
screens are instead snapshotted to PSRAM once (`lv_snapshot_take_to_draw_buf`,
115KB each), and frames are composed and pushed straight at the panel,
bypassing LVGL for the ~0.35s it runs. Two consequences worth knowing:

- **PSRAM is now enabled** (`PSRAM=enabled` in the FQBN). The ESP32-S3R2
  here has 2MB of embedded quad PSRAM — confirmed with `esptool chip-id`,
  not assumed. It claims GPIO 35-37, which nothing on this board uses.
  If allocation ever fails, `meltReady` stays false and `goToScreen()`
  falls back to the stock slides, which still encode direction (sections
  drop from the top, the clock slides back up, settings rises from the
  bottom to match its swipe).
- **SPI was raised 40MHz → 80MHz.** The melt is bandwidth-bound: one
  115KB frame is 23ms at 40MHz and ~11ms at 80MHz. Measured on hardware,
  this took a transition from ~600ms to ~350ms (≈19 frames at ~20ms).
  Every other redraw got cheaper too. **If the panel ever shows tearing,
  speckle or shifted pixels, `freq_write` in `LGFX_Config.hpp` is the
  first thing to put back.**

The boot screen melts into the watch face for the same reason — it needs
the snapshot taken before Vault Boy's 73KB sprite is freed, which is why
`initMelt()` runs up with the LVGL display setup rather than beside the
other screen builders.

**Per-section colours**: clock/steps green, history orange, weather
yellow, notifications blue, music purple, stopwatch red. Each section's
accent carries through its whole screen, its nav badge, and its back
button. The weather badge draws an actual sun (via the icon sprite
bridge below) rather than a text label.

**Boot** is its own LVGL screen now, not the old scrolling terminal log: a
"COGENT TECH (TM) / TERMLINK PROTOCOL" header, a status line, a segmented
Fallout-terminal-style progress bar that fills as each real boot step
completes, then the header/bar hide and Vault Boy gets the screen to
himself for the power-on flicker reveal. This is why `lv_init()` runs at
the *top* of `setup()` here rather than after the boot sequence — see the
comment there for why that stays conflict-free with the raw-LovyanGFX
WiFi-portal and sleep screens.

**Reused wholesale, unchanged**, from `pipboy_watch.ino`: WiFi/NTP sync and
the WiFiManager captive portal, BLE setup and the entire Gadgetbridge
message parser (notify/weather/musicinfo/musicstate/setTime), the battery
curve and critical-battery shutdown, NVS persistence, the step-history
rollover, raise-to-wake's IMU math, and deep sleep. The WiFi-setup-portal
and sleep/critical-battery screens still draw straight to `display` as
plain LovyanGFX calls; that stays conflict-free with LVGL because
`lv_timer_handler()` is never pumped during those windows, and LVGL only
repaints when explicitly told to.

**Reused with light adaptation**: the hand-drawn weather icon shapes
(`drawIconSun`/`Cloud`/`RainStreaks`/`drawWeatherIcon`) — same pixel math,
just parameterized to draw onto an off-screen `LGFX_Sprite` instead of the
global `display`, which then gets handed to LVGL as a plain image (once the
RGB565 byte-order mismatch between the two libraries was found and fixed —
see project memory). The gesture *decision logic* (tap-duration thresholds,
the 4-tap step reset, which duration means sleep vs. WiFi-portal) is also
reused essentially verbatim, just triggered from LVGL's press/release/
gesture events instead of a hand-rolled polling loop.

**New**: the whole screen layer, built as LVGL widgets — labels, an
`lv_bar`-based history chart, a scrollable snap-list for notifications
(replacing pipboy_watch's index-paged list), LVGL's built-in symbol glyphs
(`LV_SYMBOL_BATTERY_*`, `LV_SYMBOL_BLUETOOTH`, `LV_SYMBOL_PLAY`/`PAUSE`) in
place of the hand-drawn battery/Bluetooth icons, and a tiled,
alpha-blended CRT scanline texture on `lv_layer_top()` so it covers every
screen without duplicating per-screen. (A second overlay — a bright line
sweeping top-to-bottom on a loop — was tried and removed by request.)

The **MUSIC screen** is deliberately unlike the others, as a look at what
else LVGL offers: a progress *ring* (`lv_arc`) driven by playback position
extrapolated from the last known value, real prev/play/next transport
buttons with their own hit targets and press states, and marquee-scrolling
(`LV_LABEL_LONG_SCROLL_CIRCULAR`) track/artist text instead of wrapping
long titles into a block.

The **notification banner** that used to sit atop the clock screen is gone
— the nav ring occupies that band now. New notifications still pop the
NOTIF screen up on arrival, and the bell badge fills in while anything is
waiting.

**Outstanding, not a firmware issue**: track/artist names on the MUSIC
screen require Gadgetbridge to have Android's notification-access
permission granted so it can read Spotify's (or whatever player's) media
session — without it, Gadgetbridge sends play/pause state but never the
track info. Confirmed via live serial capture and Gadgetbridge's own
"Debug → Set Music Info" test round-tripping correctly. See project memory
for the exact permission path, including a known Android 13+/F-Droid
"restricted settings" gotcha.

Build/flash identically:

```
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M pipboy_lvgl
./bin/arduino-cli compile --fqbn esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M --upload -p <port> pipboy_lvgl
```
