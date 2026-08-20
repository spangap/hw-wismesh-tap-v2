# hw-wismesh-tap-v2 — INTERNALS

Maintainer reference for the RAK WisMesh TAP V2 board HAL. For what the straddle
is and how to build with it, read [README.md](README.md) first.

## 1. Bring-up: two rails, then the CS park, all in the `start:` band

`WismeshTapBoard::onStart` ([esp-idf/src/wismeshtap.cpp](esp-idf/src/wismeshtap.cpp))
runs **before** `spangapInit()`, because the first shared-SPI-bus access is
`fs_mount_sd()` *inside* `spangapInit()`, and three preconditions must hold by
then:

1. **The 3V3 peripheral rail (GPIO 14) is up.** The SD card (and the display,
   touch and GNSS) sits behind it; until it's HIGH the SD probe just times out
   (`ESP_ERR_TIMEOUT`).
2. **The LoRa rail (GPIO 4) is up.** The RAK3112's internal SX1262 has its own
   power gate, and iface-lora has no Kconfig symbol for a radio rail —
   `loraInit()` assumes a powered chip. It also runs far too late to do this
   itself relative to nothing in particular, but the rail costs nothing here
   and makes the following CS park meaningful (an unpowered chip has no
   drivers to park).
3. **Every shared-bus CS is parked HIGH.** The ST7789 shares the SD card's bus
   (host 3) with no driver owning its CS yet; parked LOW-ish it would drive
   MISO during the SD probe. The SX1262's CS is parked too — its bus (host 2)
   is private, but the chip is live from the rail-up and `loraInit()` claims
   the pin much later.

## 2. The input HAL: button only — touch is the lcd component's

The FT5x06 is **not brought up by this board**: it is spangap-lcd's own
`CONFIG_LCD_TOUCH_*` controller (`lcd_touch.cpp`), configured from
`straddle.yaml` the same way the panel is. That module owns the I2C0 bus, the
INT wiring, the standby gating of touch reads, and the `lcd.multi_touch`
request key (maps sets it around its pinch-zoom; the FT5x06's 2 points are
enough for a pinch). The component path fits this board because touch is the
port's **only** client; a bus shared with other board chips needs the
board-HAL `touch_read` fallback instead — the T-Deck's keyboard-entangled
GT911 (`tdeck_lcd.cpp`) is that case and the template for it.

What the board's input HAL contributes is the Home button alone:
`pointer_read` and `touch_read` are null, `lcdSetHasKeyboard()` is never
called (keeping the **on-screen keyboard** active), and `click_read` never
returns true — a Home-button press is navigation, not a cursor click.

## 3. The Home button / standby state machine

One button, four meanings, all timed here (`kStandbyHoldMs` 300 /
`kMulticlickMs` 250 — reflexes, not settings):

- hold → `sys.standby = 1`
- one click → `lcdGoHome()` (dispatched when the multi-click window closes)
- two clicks → `lcdShowRecents()` (dispatched on its own release, no window)
- press in standby → `sys.standby = 0`, press fully absorbed

The board only flips `sys.standby`; the subscription (`tapStandby`, lcd task)
does the display sleep/wake via `lcdScreenSleep/Wake`. Touch gating is not
here: spangap-lcd's controller module watches the same key and discards its
samples while it is set, so a resting finger can't hold the device awake.

The light-sleep wake path is ported from the T-Deck (same GPIO 0, same
semantics), and its three traps carry over verbatim:

- **LOW_LEVEL, not edge**: edges are invisible while the GPIO clock is gated in
  light sleep, so the wake source is level-triggered — and a level ISR re-fires
  for the whole press, so `tapStandbyBtnISR` silences the pin on first fire
  (LL register write; the driver call takes a non-ISR spinlock) and
  `tapClickRead` re-enables it after handling.
- **The entry press must lift before arming**: standby is usually entered by a
  press that is still down; `s_standbyLock` (a `PM_NO_LIGHT_SLEEP` lock) holds
  the CPU awake for just that window, released the moment the wake source arms.
- **Bounce backstop**: a press bounces, and the sleep-exit window can land in a
  bounce-high gap so the level ISR never fires. `tapSleepWake` (registered via
  `pmOnLightSleepWake`) re-checks GPIO 0 on every GPIO-caused wake and latches
  the press itself.

What was deliberately **not** ported from the T-Deck: the wake press opens no
click burst (it is absorbed entirely — the first touch of a sleeping device
should do nothing but wake it), and there is no three-click tier (two meanings
suffice when a single click is already navigation).

## 4. Everything else is Kconfig VALUES, not sources

A board straddle is **non-buildable**: a `sdkconfig.defaults` here would be
ignored under `--with`. The hardware profile lives entirely in `straddle.yaml`'s
`kconfig:` block — memory (16 MB flash, octal PSRAM, the 8 MB firmware floor),
the SD card (`CONFIG_SPANGAP_SDCARD_*`, spangap-core's), the display and touch
(`CONFIG_LCD_*` / `CONFIG_LCD_TOUCH_*`, spangap-lcd's), the radio
(`CONFIG_LORA*`, iface-lora's, `when:`-gated) and the GNSS UART
(`CONFIG_GPS_*`, gps's, `when:`-gated).
This board defines **no Kconfig symbols of its own**; its bespoke pins
(rails, touch, button, battery, buzzer, LEDs) are `BOARD_*` macros in
[esp-idf/include/wismeshtap.h](esp-idf/include/wismeshtap.h).

## 5. Battery: divider ratio and the OCV curve

GPIO 1 carries VBAT through a gate-less divider, so it needs no power-up step.
The 5/3 ratio is the board's stated 1.667 multiplier applied on top of a
curve-fit-**calibrated** ADC read — the same reasoning as the T-Deck, where
Meshtastic-style multipliers can bake in compensation for an *uncalibrated*
ADC. If a multimeter disagrees, trim `BAT_DIV_NUM/DEN`, don't touch the
calibration. Percent comes from the board's 11-point open-circuit-voltage
curve (100 %…0 % in decade steps), linearly interpolated, smoothed by 16-sample
averaging plus a light EMA.

## 6. Pitfalls

- **Source-of-truth gaps.** The pin map comes from the board's Meshtastic
  variant cross-checked against MeshCore's bare-RAK3112 variant, not a unit in
  hand. The four verify-first items (GNSS UART orientation, touch mirroring,
  panel invert, divider ratio) are listed in README.md.
- **GPIO 0 is the BOOT strap.** Holding the Home button through a reset enters
  the ROM downloader — that's the chip, not a bug here.
- **Two 16 MB touch boards.** The T-Deck and this board both pass the flash
  check in `detect_hw`; the anchor (FT5x06 at 0x38 on 9/40) and the radio pins
  are what separate them. Keep both probes honest when editing either — and
  keep flashmon's copies (`flashmon/esp-idf/main/detect.c`) in step.
