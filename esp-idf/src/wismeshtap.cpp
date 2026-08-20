/**
 * wismeshtap.cpp — RAK WisMesh TAP V2 board support, end to end.
 *
 * Single owner of all WisMesh TAP V2 hardware bring-up. See wismeshtap.h for the
 * API contract and README.md for the hardware reference. Layout:
 *
 *   1. Power rails + shared-SPI CS park.
 *      Always compiled — the SD card needs the +3.3 V rail (and an undriven
 *      panel CS parked) even with no on-device UI. Driven from
 *      WismeshTapBoard::onStart before spangapInit().
 *   2. Battery monitor (WismeshTapBattery).
 *
 * The Home-button input HAL lives in
 * conditional/spangap-lcd/src/wismeshtap_lcd.cpp — compiled only when
 * spangap-lcd is staged. No #if for it anywhere here. The FT5x06 touch (and
 * its I2C bus) is the lcd component's own CONFIG_LCD_TOUCH_* controller — no
 * board code at all.
 */
#include "wismeshtap.h"
#include "log.h"            /* warn (battery adc) */
#include "storage.h"        /* battery.* ephemerals, sys.board */

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

/* =========================================================================
 * 1. Power rails + shared-SPI CS park
 *
 * Board prerequisites for the very first shared-SPI-bus access — which is
 * fs_mount_sd() *inside* spangapInit(). All three MUST happen before that:
 *
 *  1. Peripheral power rail (GPIO 14). Gates the +3.3 V rail to the SD card,
 *     display, touch and GNSS. Until it's driven HIGH the SD card is
 *     unpowered, so esp_vfs_fat_sdspi_mount() just times out.
 *  2. LoRa power rail (GPIO 4). Gates the RAK3112 module's SX1262 supply.
 *     iface-lora has no rail symbol of its own, and loraInit() runs long
 *     after spangapInit() — drive it here so the radio is alive (and its CS
 *     parkable) from the start.
 *  3. Idle CS park. The ST7789 panel shares the SD card's bus with no driver
 *     owning its CS yet; park it HIGH so it doesn't drive MISO during the SD
 *     probe. The SX1262's CS is parked too — its bus is private, but the
 *     radio is live once its rail is up and loraInit() claims the pin much
 *     later.
 * ========================================================================= */

static void wismeshtapPowerInit(void)
{
    auto driveHigh = [](int pin) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode         = GPIO_MODE_OUTPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
        gpio_set_level((gpio_num_t)pin, 1);
    };
    driveHigh(BOARD_POWER_EN_PIN);
    driveHigh(BOARD_LORA_POWER_EN_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));   /* 3.3 V rail settle (matches lora.cpp) */

    /* Park every shared-bus device's CS HIGH (deselected) so none of them
     * drive MISO during the SD probe. */
    auto parkCsHigh = [&](int pin) {
        if (pin < 0) return;
        driveHigh(pin);
    };
    /* The LCD CS pin comes from the lcd component's Kconfig (CONFIG_LCD_CS_PIN);
     * park it HIGH before the SD probe (inside spangapInit, before lcdInit claims
     * the pin) so the panel doesn't drive MISO. Defined only on an LCD build. */
#if defined(CONFIG_LCD_CS_PIN)
    parkCsHigh(CONFIG_LCD_CS_PIN);
#endif
    /* LoRa radio CS pins come from iface-lora's Kconfig (CONFIG_LORA*). The
     * radio's bus is its own, but the chip is live from the rail-up above and
     * loraInit() only claims the pin much later — park it deselected. */
#if defined(CONFIG_LORA0_CS_PIN)
    parkCsHigh(CONFIG_LORA0_CS_PIN);
#endif
}

/* =========================================================================
 * 2. Battery monitor — VBAT via the GPIO1 divider (see BOARD_BAT_ADC).
 *
 * Always compiled (no UI dependency): a once-a-minute esp_timer samples the ADC
 * and publishes two ephemerals the rest of the system reacts to —
 *   battery.millivolt  — true VBAT in mV (pin reading × divider)
 *   battery.percent    — 0..100, via the open-circuit-voltage curve below
 * spangap-lcd's status bar subscribes to battery.percent for its icon; spangap-
 * core's `bat` CLI command prints both. No dedicated task — the periodic timer
 * callback does the read on the esp_timer task.
 * ========================================================================= */

namespace {

/* Divider ratio 5/3 (= the board's stated 1.667 multiplier), applied on top of
 * a curve-fit-calibrated ADC reading. Trim NUM/DEN if a multimeter disagrees. */
constexpr uint32_t BAT_DIV_NUM   = 5, BAT_DIV_DEN = 3;
constexpr int      BAT_SAMPLES   = 16;             /* averaged per read — kills ADC jitter */
constexpr int64_t  BAT_PERIOD_US = 60LL * 1000000; /* re-sample cadence: every minute */

/* Open-circuit voltage at 100 %, 90 %, … 0 % — the board's published discharge
 * curve. Linear interpolation between the decade points; input jitter is
 * smoothed by the per-read averaging + EMA below. */
const uint16_t s_ocvMv[11] = {
    4160, 4020, 3940, 3870, 3810, 3760, 3740, 3720, 3680, 3620, 2990,
};

adc_oneshot_unit_handle_t s_adc      = nullptr;
adc_cali_handle_t         s_adcCali  = nullptr;
adc_unit_t                s_adcUnit  = ADC_UNIT_1;
adc_channel_t             s_adcChan  = ADC_CHANNEL_0;   /* GPIO1; confirmed at init */
bool                      s_adcReady = false;
uint32_t                  s_mvEma    = 0;               /* smoothed VBAT, mV (0 = unset) */

uint8_t batteryPercent(uint16_t mv) {
    if (mv >= s_ocvMv[0])  return 100;
    if (mv <= s_ocvMv[10]) return 0;
    for (int i = 1; i <= 10; i++) {
        if (mv >= s_ocvMv[i]) {
            uint16_t hi = s_ocvMv[i - 1], lo = s_ocvMv[i];
            int pctLo = 100 - i * 10;
            return (uint8_t)(pctLo + (uint32_t)(mv - lo) * 10 / (hi - lo));
        }
    }
    return 0;
}

/* Sample, smooth, publish. Runs on the esp_timer task (and once at init). */
void batteryRead(void*) {
    if (!s_adcReady) return;
    int acc = 0, ok = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        int raw;
        if (adc_oneshot_read(s_adc, s_adcChan, &raw) == ESP_OK) { acc += raw; ok++; }
    }
    if (!ok) return;
    int raw = acc / ok;
    int pinMv;
    if (!(s_adcCali && adc_cali_raw_to_voltage(s_adcCali, raw, &pinMv) == ESP_OK))
        pinMv = (int)((int64_t)raw * 3100 / 4095);      /* nominal 12-bit @ 12 dB */
    uint32_t mv = (uint32_t)pinMv * BAT_DIV_NUM / BAT_DIV_DEN;
    /* Light EMA across reads (~3-4 min at the 1/min cadence) so the icon and
     * percent don't wobble on noise; first reading seeds it directly (no lag). */
    s_mvEma = s_mvEma ? (s_mvEma * 3 + mv) / 4 : mv;
    uint16_t outMv = (uint16_t)s_mvEma;
    storageBegin();                                     /* one commit -> subscribers see both */
    storageSet("battery.millivolt", (int)outMv);
    storageSet("battery.percent",   (int)batteryPercent(outMv));
    storageEnd();
}

}  // namespace

/* onInit — ADC bring-up, an initial reading, then the once-a-minute timer.
 * Runs after spangapInit() so storage is up for the ephemeral writes. */
void WismeshTapBattery::onInit() {
    if (adc_oneshot_io_to_channel(BOARD_BAT_ADC, &s_adcUnit, &s_adcChan) != ESP_OK) {
        warn("battery: GPIO%d is not an ADC pin\n", BOARD_BAT_ADC);
        return;
    }
    adc_oneshot_unit_init_cfg_t ucfg = {};
    ucfg.unit_id = s_adcUnit;
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) {
        warn("battery: adc unit init failed\n");
        return;
    }
    adc_oneshot_chan_cfg_t ccfg = {};
    ccfg.atten    = ADC_ATTEN_DB_12;        /* ~0..3.1 V pin range; VBAT×3/5 maxes ~2.5 V */
    ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(s_adc, s_adcChan, &ccfg) != ESP_OK) {
        warn("battery: adc channel config failed\n");
        return;
    }
    adc_cali_curve_fitting_config_t cal = {};
    cal.unit_id  = s_adcUnit;
    cal.chan     = s_adcChan;
    cal.atten    = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_adcCali) != ESP_OK) {
        s_adcCali = nullptr;                /* fall back to nominal raw->mV scaling */
        warn("battery: adc calibration unavailable, using nominal scale\n");
    }
    s_adcReady = true;

    batteryRead(nullptr);                   /* publish an initial reading now */

    const esp_timer_create_args_t targs = { .callback = batteryRead, .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK, .name = "battery", .skip_unhandled_events = true };
    esp_timer_handle_t th = nullptr;
    if (esp_timer_create(&targs, &th) == ESP_OK)
        esp_timer_start_periodic(th, BAT_PERIOD_US);
    else
        warn("battery: timer create failed\n");
}

void WismeshTapBoard::onStart() {
    wismeshtapPowerInit();                  /* both rails + shared-SPI CS park */
}

/* onInit — the board says what it is, once storage exists to say it into. Every
 * surface that names the hardware (the Hardware section of Settings, on both
 * the display and the browser) reads this key, so a board is identified in one
 * place rather than by each surface knowing which board it is running on. */
void WismeshTapBoard::onInit() {
    storageSet("sys.board", BOARD_NAME);
}
