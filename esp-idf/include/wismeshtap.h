/**
 * wismeshtap.h — RAK WisMesh TAP V2 board support for reticulous.
 *
 * What this module provides:
 *   - Compile-time hardware constants for the board's bespoke peripherals: the
 *     two power rails, the Home button and the battery sense. Consumed by
 *     wismeshtap.cpp / conditional/spangap-lcd/wismeshtap_lcd.cpp.
 *   - The board bring-up Services (WismeshTapBoard / WismeshTapBattery).
 *
 * The display AND its FT5x06 touch are not wired here: both are owned by the
 * lcd component and configured through CONFIG_LCD_* / CONFIG_LCD_TOUCH_*
 * supplied by straddle.yaml's kconfig: block. The LoRa radio pins likewise
 * come from iface-lora's CONFIG_LORA*, the SD card's from spangap-core's
 * CONFIG_SPANGAP_SDCARD_*, and the GNSS receiver's from gps's
 * CONFIG_GPS_*. So this header carries only the board's own pins. Runtime LoRa
 * parameters (freq, BW, SF, ...) live in storage at s.lora.*.
 */
#pragma once

#include "sdkconfig.h"
#include "service.h"

/* The board's name as a person reads it — published to sys.board at init and
 * shown in the Hardware section of Settings, so the UI never spells a board
 * name of its own. */
#define BOARD_NAME              "RAK WisMesh TAP V2"

/* Peripheral power-enable pin: gates the +3.3 V rail to the display, touch,
 * SD card and GNSS receiver. Must be driven HIGH at boot or the SD probe inside
 * spangapInit() times out against an unpowered card. */
#define BOARD_POWER_EN_PIN      14
#define BOARD_POWER_EN_ACTIVE   1   /* 1 = active high */

/* LoRa radio power-enable pin: gates the RAK3112 module's SX1262 supply.
 * iface-lora has no rail symbol of its own, so the board drives it HIGH in
 * onStart alongside the peripheral rail — SPI traffic to the SX1262 before
 * that is just SPI traffic into a powered-down chip. */
#define BOARD_LORA_POWER_EN_PIN 4

/* Battery sense: VBAT through a resistor divider into GPIO 1 (ADC1). The
 * board's Meshtastic variant states the divider ratio as 1.667 (= 5/3); the
 * divider has no enable gate, so the pin always carries the divided VBAT and
 * needs no power-up step. Trim BAT_DIV_NUM/DEN in wismeshtap.cpp if a
 * multimeter disagrees. */
#define BOARD_BAT_ADC           1

/* Home button: GPIO 0 (the BOOT-strap pin), pulled-up active-low.
 * wismeshtap_lcd.cpp gives it four meanings: a 300 ms hold enters standby, one
 * click goes Home (lcdGoHome), two clicks raise the app switcher
 * (lcdShowRecents), and a press while in standby wakes the device. There is no
 * pointer device, so a press never synthesizes a cursor click — the touch
 * screen is the pointer. */
#define BOARD_HOME_BTN_PIN      0

/* Present on the board but unwired — the platform has no engine for either:
 * a magnetic buzzer on GPIO 38, and the case LEDs (green 46, blue 45). */
#define BOARD_BUZZER_PIN        38
#define BOARD_LED_GREEN_PIN     46
#define BOARD_LED_BLUE_PIN      45

/**
 * Board bring-up. onStart is the always-on hardware bring-up: it drives both
 * power rails HIGH and parks the shared-SPI CS lines (the first shared-bus
 * access is fs_mount_sd() *inside* spangapInit(), and the panel shares that
 * bus), so it runs in the start band, before spangapInit(). onInit publishes
 * sys.board once storage exists to say it into.
 *
 * The on-device-UI input HAL (the Home button; touch is the lcd component's
 * own CONFIG_LCD_TOUCH_* controller) lives in
 * conditional/spangap-lcd/src/wismeshtap_lcd.cpp behind a
 * when: spangap/spangap-lcd Service (WismeshTapLcdInput) — compiled and
 * registered only when spangap-lcd is staged, so no #if is needed anywhere.
 */
class WismeshTapBoard : public Service {
public:
    void onStart() override;   /* rails + CS park */
    void onInit()  override;   /* publishes sys.board */
};

/**
 * Battery monitor bring-up (onInit): configures the GPIO1 ADC, publishes an
 * initial battery.millivolt / battery.percent, and arms a once-a-minute
 * esp_timer to keep them fresh. init band (needs storage up). No task of its
 * own — the periodic timer callback does the sampling.
 */
class WismeshTapBattery : public Service {
public:
    void onInit() override;
};
