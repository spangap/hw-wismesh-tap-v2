/**
 * detect.cpp — is the hardware under this firmware a RAK WisMesh TAP V2?
 *
 * The board's one self-assertion. It answers about THIS board only: its own
 * name when the anchor peripheral and the radio both answer on TAP V2 pins, and
 * NULL otherwise. Nothing here enumerates other boards — that comparison belongs
 * to whoever calls it.
 *
 * Two callers, one body:
 *
 *   * spangap-core, at the top of spangapInit(), before any bus is claimed. A
 *     board straddle is staged because the image was built for that board, so a
 *     NULL here means the image is on the wrong hardware and the platform halts
 *     rather than driving someone else's pins for the rest of the boot.
 *   * flashmon's standalone detector, which carries a copy of this function
 *     renamed `detect_hw_wismesh_tap_v2` and calls it alongside every other
 *     board's, to identify a chip whose firmware is unknown.
 *
 * The copy is manual and deliberately so — see detect_probe.h. Change this,
 * change flashmon/esp-idf/main/detect.c.
 *
 * The rails: this drives both power enables, because the touch controller
 * cannot answer without the 3V3 rail and the SX1262 sits behind its own gate.
 * It releases them ONLY when the probe fails. On success the rails are wanted
 * either way round — the firmware is about to use them (and
 * WismeshTapBoard::onStart drove them already, so this is idempotent), and the
 * detector is reset into real firmware straight after.
 */
#include "detect_probe.h"
#include "wismeshtap.h"

/* LoRa pins (the RAK3112 module's internal SX1262). They live in straddle.yaml
 * as CONFIG_LORA0_* — but those symbols only exist when iface-lora is staged,
 * and this probe has to work in an image built without a radio stack at all,
 * so they are written out here. */
#define DETECT_LORA_SCK    5
#define DETECT_LORA_MOSI   6
#define DETECT_LORA_MISO   3
#define DETECT_LORA_CS     7
#define DETECT_LORA_RST    8
#define DETECT_LORA_BUSY  48

/* GNSS RX. Lives in straddle.yaml as CONFIG_GPS_RX_PIN for the same reason —
 * the symbol only exists when gps is staged. */
#define DETECT_GPS_RX     44

/* FT5x06 touch. Lives in straddle.yaml as CONFIG_LCD_TOUCH_* for the same
 * reason — those symbols only exist when spangap-lcd is staged, and this probe
 * has to work in a --no-lcd image too. 0x38 is the family's one fixed
 * address. */
#define DETECT_TOUCH_SDA   9
#define DETECT_TOUCH_SCL  40
#define DETECT_TOUCH_ADDR 0x38

extern "C" const char* detect_hw(void)
{
    /* 16 MB flash or it is not a TAP V2 — cheapest possible rejection, and it
     * touches no pin at all. */
    if (!detect_flash_mb(16)) return NULL;

    detect_rail_drive(BOARD_POWER_EN_PIN, BOARD_POWER_EN_ACTIVE ? 1 : 0);
    detect_rail_drive(BOARD_LORA_POWER_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(150));            /* 3.3 V rail settle */

    /* Anchor: the FT5x06 touch controller at its one fixed address on the board
     * I2C bus. A plain ACK is all it needs to give — its meaning comes from the
     * pins. POLLED, not probed once: the controller runs its own firmware off
     * this rail and can miss the first probe after a cold power-on. */
    bool tp = false;
    for (int i = 0; i < 6; i++) {
        if ((tp = detect_ack(DETECT_TOUCH_SDA, DETECT_TOUCH_SCL, DETECT_TOUCH_ADDR)))
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!tp) {
        detect_miss("no touch at 0x%02X — not a WisMesh TAP V2", DETECT_TOUCH_ADDR);
        detect_rail_release(BOARD_LORA_POWER_EN_PIN);
        detect_rail_release(BOARD_POWER_EN_PIN);
        return NULL;
    }
    /* Confirm with the radio: the RAK3112's internal SX1262 on its private bus.
     * The T-Deck is the other 16 MB touch board and its radio sits on entirely
     * different pins, so an SX1262 answering here settles it. */
    if (!detect_radio_is(DETECT_LORA_SCK, DETECT_LORA_MOSI, DETECT_LORA_MISO,
                         DETECT_LORA_CS, DETECT_LORA_RST, DETECT_LORA_BUSY, "sx1262")) {
        detect_miss("touch answered but no SX1262 — not a WisMesh TAP V2");
        detect_rail_release(BOARD_LORA_POWER_EN_PIN);
        detect_rail_release(BOARD_POWER_EN_PIN);
        return NULL;
    }

#if DETECT_EXTRAS
    /* Logged for the person reading rather than tested: the GNSS receiver
     * (RAK12501, Quectel L76K). Its autobaud listens 1.2 s per rate, which is
     * why this never runs in the firmware — see DETECT_EXTRAS. */
    detect_gps(DETECT_GPS_RX, NULL);
#endif

    detect_found("hw_wismesh_tap_v2");
    return "hw-wismesh-tap-v2";
}
