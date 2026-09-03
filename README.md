# hw-wismesh-tap-v2 — RAK WisMesh TAP V2 board HAL

**hw-wismesh-tap-v2** is the board-support straddle for the **RAK WisMesh TAP V2**
(RAK10710) — a handheld touch-screen LoRa mesh node built on the **RAK3312
WisBlock Core** (the RAK3112 module: ESP32-S3, 16 MB flash, 8 MB **octal**
PSRAM, with a Semtech **SX1262** inside the module on a dedicated SPI bus). The
case carries a 2.8" **ST7789** 320x240 panel with **FT5x06** capacitive touch,
an SD-card slot sharing the panel's SPI bus, a **RAK12501** GNSS receiver
(Quectel L76K), a Home button, a buzzer, two LEDs and a li-ion battery. Board
reference:
<https://store.rakwireless.com/products/meshtastic-touch-client-wismesh-tap-v2-sx1262>.

It is a **non-buildable** component — it decides nothing about what the device
*does*. A buildable assembler (`reticulous/reticulous`) adds it and inherits the
board: `spangap build reticulous/reticulous --with spangap/hw-wismesh-tap-v2`.
The mesh stack, the IP/web platform, `app_main`, the partition layout, the
update story and the browser SPA all come from the buildable and its other
straddles — not from here.

The board stages `spangap-lcd` (it physically has a screen; drop with
`--no-lcd`) and `gps` (the receiver is built in). There is **no
hardware keyboard**: the board never calls `lcdSetHasKeyboard()`, so
spangap-lcd's **on-screen keyboard** stays active for all text input. There is
no pointer device either — the touch screen is the pointer.

## ⚠️ Verify before trusting

The pin map below was assembled from the board's Meshtastic variant
(`variants/esp32s3/rak_wismesh_tap_v2`) cross-checked against MeshCore's
bare-RAK3112 variant, **not** from a board in hand. Confirm against your actual
unit before an RF or partition run:

- **GNSS UART orientation.** Meshtastic's device variant reads host RX = 44 /
  host TX = 43 (what's wired here); MeshCore's bare-module variant names the
  pair the other way round. If the receiver stays silent on real hardware, swap
  `CONFIG_GPS_RX_PIN` / `CONFIG_GPS_TX_PIN` in `straddle.yaml`.
- **Panel polarity.** `CONFIG_LCD_INVERT_COLOR=y` (the usual ST7789 IPS
  setting). If colours come out negative, flip it.
- **Battery divider.** The 5/3 ratio is the variant's stated 1.667 multiplier.
  Trim `BAT_DIV_NUM/DEN` in `esp-idf/src/wismeshtap.cpp` if a multimeter
  disagrees.

## What it does, and how it fits

The board contributes hooks that the buildable's generated init dispatcher
calls. There is nothing to call by hand: if the straddle is in the build, the
board comes up automatically.

| Hook | Band | Present when | Brings up |
|---|---|---|---|
| `WismeshTapBoard::onStart` | start | always | both power rails, shared-SPI CS park |
| `WismeshTapBoard::onInit` | init | always | publishes `sys.board` |
| `WismeshTapLcdInput::onStart` | start | spangap-lcd staged | Home-button input HAL (registered before `lcdInit`) |
| `WismeshTapBattery::onInit` | init | always | 1/min ADC sampling → `battery.millivolt` / `battery.percent` |

The FT5x06 touch is **not** a board hook: it is the lcd component's own
`CONFIG_LCD_TOUCH_*` controller ([spangap-lcd](../spangap-lcd)'s
`lcd_touch.cpp`), which this board configures from `straddle.yaml` the same way
it configures the panel — no touch C code here.

`WismeshTapBoard::onStart` runs **before** `spangapInit()`: it drives the 3V3
peripheral rail (GPIO 14, feeding display/touch/SD/GNSS) and the LoRa power
gate (GPIO 4) HIGH, then parks the panel's and radio's CS lines HIGH — the SD
card shares the panel's SPI bus, and `fs_mount_sd()` inside `spangapInit()` is
the first access to it.

The Home button's meanings (owned by the input HAL): hold 300 ms → standby,
one click → launcher (`lcdGoHome`), two clicks → app switcher
(`lcdShowRecents`), any press in standby → wake. In standby the display is off
and the button is armed as a light-sleep wake source.

The glass wakes it too: this is a handheld with its button round the back, so
`CONFIG_LCD_WAKE_ON_TOUCH_DEFAULT=y` and the `s.lcd.wake_on_touch` Display row
ships on (a pocket-carried deck ships it off). While it holds, the FT5x06's INT
is armed as a second light-sleep wake source and a finger on the dark screen
clears `sys.standby`; that finger is swallowed, so it wakes the device without
pressing what was left under it. Turn the row off and the Home button is the
only way back.

The LoRa radio engine, the SD/state filesystem, the GNSS task, the IP/web
platform and the mesh stack are owned by other straddles
([iface-lora](../iface-lora), [spangap-core](../spangap-core),
[gps](../gps), [spangap-net](../spangap-net), [rns](../rns));
this board supplies pins (below) and the bring-up glue.

## Board identity (`detect_hw`)

`esp-idf/src/detect.cpp` answers one question about this board: it returns
`"hw-wismesh-tap-v2"` when the hardware under the firmware is this board, and
NULL when it is not. What it asks:

16 MB flash, then — after driving both power rails — an FT5x06 ACK at 0x38 on
the board I2C (polled; the controller boots its own firmware off a cold rail),
confirmed by an SX1262 answering on the RAK3112's internal pins. The T-Deck is
the other 16 MB touch board and its radio sits on entirely different pins.

spangap-core calls it before the first `onStart()` — the last moment no bus is
claimed — and **halts the device awake** when the answer disagrees with the
board this image was built for. The confirmed answer is published as `sys.hw`
and announced on the console as `build: hw hw-wismesh-tap-v2`. flashmon's
standalone detector carries a hand-kept copy of the same function, renamed
`detect_hw_wismesh_tap_v2`; change one, change the other. See
[spangap-core/docs/init.md](../spangap-core/docs/init.md) and
[flashmon/docs/detect.md](../flashmon/docs/detect.md).

## Hardware & pin map

RAK3312 WisBlock Core (RAK3112 module: ESP32-S3, 16 MB flash, 8 MB **octal**
PSRAM, selected via `CONFIG_SPIRAM_MODE_OCT`).

### LoRa SX1262 (inside the RAK3112; owned by iface-lora, pins published here)

On its own SPI bus, **host 2** — separate from the display/SD bus.

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| NSS / CS | 7 | | RST | 8 |
| SCK | 5 | | BUSY | 48 |
| MOSI | 6 | | DIO1 | 47 |
| MISO | 3 | | Power enable | 4 |

The SX1262 drives **DIO2** as its own RF antenna switch
(`CONFIG_LORA0_DIO2_RF_SWITCH=y`) and **DIO3** supplies the 1.8 V TCXO
(`CONFIG_LORA0_TCXO_MV=1800`). The module's supply is gated behind **GPIO 4**
(no iface-lora symbol exists for a radio rail, so `WismeshTapBoard::onStart`
owns it). One radio (`CONFIG_LORA_COUNT=1`), `CONFIG_LORA0_RADIO_SX1262=y`.

### Display + SD card (shared SPI bus, host 3)

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| SCK | 13 | | LCD DC | 42 |
| MOSI | 11 | | LCD backlight | 41 |
| MISO | 10 | | LCD CS | 12 |
| | | | SD CS | 2 |

ST7789, native 240x320 portrait, rotated 90° to landscape, no dedicated reset
(the panel resets with the 3V3 rail). spangap's `spi_helper` arbitrates the
shared bus; only the CS lines differ.

### Touch, button, battery, GNSS

| Peripheral | Pins |
|---|---|
| FT5x06 touch (I2C0, addr 0x38; `CONFIG_LCD_TOUCH_*`) | SDA 9, SCL 40, INT 39, no reset |
| ↳ glass laminated 180° to the panel | `CONFIG_LCD_TOUCH_MIRROR_X/_Y=y` |
| Home button | GPIO 0 (BOOT strap, pulled-up active-low) |
| Battery sense | GPIO 1 (ADC1), divider 5/3 |
| GNSS (RAK12501 / Quectel L76K, UART1) | host RX 44 ← GPS TX, host TX 43 → GPS RX |
| 3V3 peripheral rail enable | GPIO 14 (active high) |

### Present but unwired

The platform has no engine for these; they are listed so nobody hunts for
missing config:

| Peripheral | Pins |
|---|---|
| Buzzer | GPIO 38 |
| LEDs | green 46, blue 45 |

### Memory / flash (published from `kconfig:`)

A non-buildable straddle has no `sdkconfig.defaults` of its own — it would be
ignored under `--with` — so every value that describes this hardware is
published from `straddle.yaml`'s `kconfig:` block:

| Key | Value | Why |
|---|---|---|
| `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` | `y` | 16 MB flash (RAK3112) |
| `CONFIG_SPANGAP_MAX_FIRMWARE_KB` | `8192` | state floor at 8 MB: app+fixed (~3.7 MB on an LCD build) plus growth headroom below it, `/state` gets the remaining ~8 MB — the same split as the T-Deck, the other 16 MB LCD board. Changing it relocates `/state` → factory reset on next boot |
| `CONFIG_SPIRAM_MODE_OCT` | `y` | the RAK3112 carries 8 MB PSRAM in **octal** mode (`qio_opi`) |

## Storage variables

The board publishes the shared `battery.millivolt` / `battery.percent`
ephemerals. The touch controller found is published by spangap-lcd as
`lcd.touch` (shown in Settings → System → Hardware), and its detect threshold
is a Display slider on this board because the part is an FT5x06
(`s.lcd.touch_sens`). Runtime LoRa parameters
live at `s.lora.*` ([iface-lora](../iface-lora)); display settings at `s.lcd.*`
([spangap-lcd](../spangap-lcd)); GNSS at `s.gps.*` ([gps](../gps)).

## Dependencies

- [spangap-core](../spangap-core) — base runtime (storage, log, CLI, fs, ITS);
  also owns the SD card via `CONFIG_SPANGAP_SDCARD_*`.
- [spangap-lcd](../spangap-lcd) — staged by this board; owns the ST7789 panel
  via `CONFIG_LCD_*` **and** the FT5x06 touch via `CONFIG_LCD_TOUCH_*` (the
  driver components are pinned in its own manifest); consumes the button input
  HAL.
- [gps](../gps) — staged by this board; owns the GNSS task.
- [iface-lora](../iface-lora) — owns the SX1262 radio engine; this board gates
  its rail and supplies its pins via Kconfig.

## Read next

- [INTERNALS.md](INTERNALS.md) — the two-rail bring-up, the shared-bus CS park,
  the Home-button/standby state machine, and the board pitfalls.
