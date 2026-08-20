/**
 * wismeshtap_lcd.cpp — WisMesh TAP V2 on-device-UI input HAL: the Home button.
 * Lives under esp-idf/conditional/spangap-lcd/, so it is compiled ONLY when
 * spangap-lcd is staged — no #if needed.
 *
 * Touch is NOT here: the FT5x06 is the lcd component's own CONFIG_LCD_TOUCH_*
 * controller (spangap-lcd's lcd_touch.cpp), which also gates its reads in
 * standby and serves the lcd.multi_touch request key. The board has no
 * hardware keyboard and no pointer device: the touch screen is the pointer,
 * and lcdSetHasKeyboard() is deliberately never called, so spangap-lcd's
 * on-screen keyboard stays active for all text input.
 *
 * The Home button (GPIO 0, active-low) carries the board's whole button
 * policy:
 *   - hold 300 ms                -> standby (set sys.standby)
 *   - one click                  -> the launcher (lcdGoHome)
 *   - two clicks                 -> the running-app switcher (lcdShowRecents)
 *   - any press while in standby -> wake (clear sys.standby), fully absorbed
 * There is no pointer to click with, so a single press is navigation rather
 * than a synthesized click. Clicks are counted while releases keep landing
 * within kMulticlickMs of each other; two is the maximum, so it dispatches on
 * its own release without waiting. Neither threshold is a setting: both are
 * reflexes, not preferences.
 */
#include "wismeshtap.h"
#include "wismeshtap_lcd.h"

#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lcd_input.h"
#include "lcd.h"
#include "storage.h"
#include "log.h"
#include "pm.h"

#include "esp_attr.h"
#include "esp_sleep.h"

#include <cstdint>
#include <cstdlib>
#include "lvgl.h"

/* Forward declarations so the HAL ops table + cross-calls resolve regardless
 * of definition order. */
static void tapInputInit(void);
static bool tapClickRead(void);
static void tapStandby(bool on);

/* ---- Home button (GPIO 0): navigation and standby ----
 * GPIO 0 is the BOOT-strap pin, read as a pulled-up active-low input after
 * boot. Meanings per the header note at the top of this file. */
static constexpr uint32_t kStandbyHoldMs = 300;   /* hold this long and the screen goes off */
static constexpr uint32_t kMulticlickMs  = 250;   /* a further click has to land within this */

static lv_timer_t*    s_standbyTimer = nullptr;   /* fires at kStandbyHoldMs -> standby */
static lv_timer_t*    s_clickTimer   = nullptr;   /* fires at kMulticlickMs -> dispatch the burst */
static int            s_clicks       = 0;         /* releases counted in the current burst */
static bool           s_btnHeld      = false;     /* a press is in progress */
static bool           s_btnConsumed  = false;     /* the hold fired -> release is not a click */
static bool           s_wakeAbsorb   = false;     /* swallow the rest of a transition press until release */
static volatile bool  s_standby      = false;     /* mirror of sys.standby (set on the lcd task) */
/* Standby wake: the button is re-armed as a genuine light-sleep wake source
 * (pmGpioWakeEnable — LOW_LEVEL, since edges are invisible while the GPIO clock
 * is gated, and sleep-isolation-exempt via gpio_sleep_sel_dis). Two wrinkles:
 *
 *  - A level-triggered ISR re-fires for as long as the line is held, so an
 *    unmitigated LOW_LEVEL ISR storms its core for the whole press.
 *    tapStandbyBtnISR therefore silences the pin on first fire; tapClickRead
 *    re-enables it after handling.
 *
 *  - Standby is usually *entered by a press* that is still down. The LOW_LEVEL
 *    wake can only be armed once that finger lifts, so s_standbyLock holds the
 *    CPU out of light sleep for just that entry-press window. */
static pm_lock_handle_t s_standbyLock = nullptr;
static bool             s_wakeArmed   = false;   /* LOW_LEVEL wake source live */
static volatile bool    s_wakePending = false;   /* the wake ISR fired: a press happened, even if the
                                                  * finger has since lifted (latched so a short press
                                                  * during the sleep-exit latency still wakes) */

/* First fire of the LOW_LEVEL wake press: silence the pin (see storm note
 * above), then the normal notify. LL register write, not gpio_intr_disable() —
 * the driver call takes a non-ISR spinlock. IRAM: the ISR service is installed
 * with ESP_INTR_FLAG_IRAM. */
static void IRAM_ATTR tapStandbyBtnISR(void* arg) {
    gpio_ll_intr_disable(&GPIO, (gpio_num_t)BOARD_HOME_BTN_PIN);
    s_wakePending = true;   /* a press occurred — latch it so a finger that lifts before
                             * the lcd task polls (sleep-exit latency) still wakes us */
    lcdInputISR(arg);
}

/* Arm the Home button as the light-sleep wake source (lcd task, in standby,
 * button up) and let the CPU sleep. Level semantics cover the handler-swap gap:
 * a press racing the swap still fires once LOW_LEVEL is set. */
static void tapWakeArm(void) {
    gpio_isr_handler_remove((gpio_num_t)BOARD_HOME_BTN_PIN);
    gpio_isr_handler_add((gpio_num_t)BOARD_HOME_BTN_PIN, tapStandbyBtnISR, nullptr);
    pmGpioWakeEnable(BOARD_HOME_BTN_PIN, GPIO_INTR_LOW_LEVEL);
    s_wakeArmed   = true;
    s_wakePending = false;   /* fresh arm: drop any stale latch */
    pmLockRelease(s_standbyLock);
}

/* Light-sleep wake backstop (IDLE-task context, on every light-sleep exit). A
 * press bounces, and the ~1 ms sleep-exit window can land in a bounce-high gap
 * so the level ISR never fires and the chip drops straight back to light sleep.
 * Re-check on the wake itself: if a GPIO wake finds us armed in standby with
 * GPIO0 held low, latch the press and notify the lcd task directly. A real
 * (held) press keeps GPIO0 low, so the chip re-wakes at once on each sleep
 * attempt and this catches it within a cycle; a DIO1 radio wake leaves GPIO0
 * high, ignored. */
static void tapSleepWake(int cause) {
    if (cause != ESP_SLEEP_WAKEUP_GPIO) return;
    if (!s_standby || !s_wakeArmed) return;
    if (gpio_get_level((gpio_num_t)BOARD_HOME_BTN_PIN) != 0) return;   /* GPIO0 high → not the button */
    s_wakePending = true;
    lcdInputSignal();
}

static void tapButtonInit(void) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BOARD_HOME_BTN_PIN;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_ANYEDGE;   /* wake on both press and release */
    gpio_config(&io);
    /* lcdInputISR wakes the lcd task on each edge; tapClickRead() (event mode)
     * runs the click/hold state machine from there. */
    gpio_isr_handler_add((gpio_num_t)BOARD_HOME_BTN_PIN, lcdInputISR, nullptr);
    pmOnLightSleepWake(tapSleepWake);      /* backstop the LOW_LEVEL standby wake */
}

static void cancelStandbyTimer(void) {
    if (s_standbyTimer) { lv_timer_delete(s_standbyTimer); s_standbyTimer = nullptr; }
}

static void cancelClickBurst(void) {
    if (s_clickTimer) { lv_timer_delete(s_clickTimer); s_clickTimer = nullptr; }
    s_clicks = 0;
}

/* The multi-click window closed with no further press (lcd task, via lv_timer):
 * a single click, which is Home. Two clicks dispatch on their own release. */
static void btnClickCb(lv_timer_t*) {
    s_clickTimer = nullptr;    /* the one-shot self-deleted after this fire */
    int n = s_clicks;
    s_clicks = 0;
    if (n == 1) lcdGoHome();
}

static void armClickWindow(void) {
    if (s_clickTimer) lv_timer_delete(s_clickTimer);
    s_clickTimer = lv_timer_create(btnClickCb, kMulticlickMs, nullptr);
    lv_timer_set_repeat_count(s_clickTimer, 1);
}

/* Held to kStandbyHoldMs: enter standby. Swallow the rest of this press so the
 * still-down finger can't immediately wake it again. */
static void btnStandbyCb(lv_timer_t*) {
    s_standbyTimer = nullptr;
    s_btnConsumed  = true;
    s_btnHeld      = false;    /* this press is done as far as the state machine */
    s_wakeAbsorb   = true;     /* ignore the held finger until it lifts */
    cancelClickBurst();        /* clicks before the hold are not a navigation burst */
    storageSet("sys.standby", 1);
}

/* lcd_input.h click_read (lcd task): the click-count / hold / standby state
 * machine. Never asserts a click — there is no pointer to click with, so a
 * press is navigation (dispatched from here / the window timer) and the return
 * value is always false. */
static bool tapClickRead(void) {
    bool down = gpio_get_level((gpio_num_t)BOARD_HOME_BTN_PIN) == 0;   /* active-low */

    /* Swallow the remainder of a press that already caused a transition (woke
     * us, or was held into standby) until the finger lifts. The lift of the
     * press that *entered* standby is also the moment the LOW_LEVEL wake source
     * can be armed (see tapWakeArm). */
    if (s_wakeAbsorb) {
        if (!down) {
            s_wakeAbsorb = false;
            if (s_standby && !s_wakeArmed) tapWakeArm();
        }
        return false;
    }
    /* In standby the only live input is this button: a press just wakes (clears
     * sys.standby) and is absorbed — never a click or a hold. */
    if (s_standby) {
        if (!s_wakeArmed) {
            /* Entry press still down (or a programmatic standby raced a press):
             * s_standbyLock is holding sleep off until we can arm on release. */
            if (!down) tapWakeArm();
            return false;
        }
        /* A press occurred — still down, OR the ISR latched one that has since
         * lifted during the sleep-exit latency. Either wakes. Clearing
         * sys.standby runs tapStandby(false), which restores the awake button
         * config. */
        if (down || s_wakePending) {
            s_wakePending = false;
            storageSet("sys.standby", 0);
            if (down) s_wakeAbsorb = true;   /* still down: swallow until it lifts */
        }
        /* Woken with nothing to act on (an already-consumed blip, or a
         * non-button wake): the ISR silenced the pin, so re-arm it or the
         * button goes deaf. */
        else gpio_intr_enable((gpio_num_t)BOARD_HOME_BTN_PIN);
        return false;
    }

    if (down) {
        if (!s_btnHeld) {
            s_btnHeld     = true;
            s_btnConsumed = false;
            /* The burst is still open — this press may be its second click, so
             * stop the window from closing under it. */
            if (s_clickTimer) { lv_timer_delete(s_clickTimer); s_clickTimer = nullptr; }
            s_standbyTimer = lv_timer_create(btnStandbyCb, kStandbyHoldMs, nullptr);
            lv_timer_set_repeat_count(s_standbyTimer, 1);
        }
        return false;                               /* never act while held */
    }

    cancelStandbyTimer();
    bool released = (s_btnHeld && !s_btnConsumed);  /* a press that wasn't held into standby */
    s_btnHeld = false;
    if (!released) return false;

    /* Two is as far as the counting goes, so it needs no window to close. */
    if (++s_clicks >= 2) { s_clicks = 0; cancelClickBurst(); lcdShowRecents(); return false; }
    armClickWindow();
    return false;
}

/* lcd_input.h init hook — runs on the lcd task once the panel and the shared
 * GPIO ISR service are up. Wire the board's input: the Home button. (Touch is
 * the lcd component's, brought up right after this hook.) */
static void tapInputInit(void) {
    tapButtonInit();

    /* Button + standby. The button (and the lcd inactivity timeout) only
     * set/clear the ephemeral sys.standby key; this subscription is what
     * actually sleeps/wakes the device — display off via lcdScreenSleep/Wake.
     * (The lcd component gates its own touch reads on the same key.) This init
     * runs on the lcd task, so the subscription dispatches there and
     * lcdScreenSleep/Wake are safe. */
    storageSubscribeChanges("sys.standby", ON_CHANGE { tapStandby(atoi(val) != 0); });
}

/* sys.standby subscription target (lcd task). The lcd component only flips the
 * key — on the inactivity timeout or our Home button; we decide what the device
 * actually does: display off, and the button becomes the wake source. */
static void tapStandby(bool on) {
    if (on == s_standby) return;
    s_standby = on;
    if (on) {
        cancelClickBurst();                            /* no burst survives into sleep */
        lcdScreenSleep();                              /* display off, backlight to 0 */
        /* Hold the CPU out of light sleep only until the LOW_LEVEL wake source
         * is armed — the entry press must lift first (see s_standbyLock). Armed
         * right away when standby came programmatically (cron/CLI, button up). */
        if (!s_standbyLock) pmLockCreate(PM_NO_LIGHT_SLEEP, "standby", &s_standbyLock);
        pmLockAcquire(s_standbyLock);
        if (gpio_get_level((gpio_num_t)BOARD_HOME_BTN_PIN) != 0) tapWakeArm();
        info("standby\n");
    } else {
        if (s_wakeArmed) {
            /* Back to the awake config: plain ANYEDGE ISR, no sleep wake. (The
             * lock was already dropped when the wake source was armed.) */
            s_wakeArmed = false;
            pmGpioWakeDisable(BOARD_HOME_BTN_PIN);
            gpio_set_intr_type((gpio_num_t)BOARD_HOME_BTN_PIN, GPIO_INTR_ANYEDGE);
            gpio_isr_handler_remove((gpio_num_t)BOARD_HOME_BTN_PIN);
            gpio_isr_handler_add((gpio_num_t)BOARD_HOME_BTN_PIN, lcdInputISR, nullptr);
        } else {
            pmLockRelease(s_standbyLock);              /* never armed — still held */
        }
        lcdScreenWake();                               /* display on, backlight fade-in */
        info("wake\n");
    }
}

/* onStart — register this board's input HAL with the lcd component, before
 * spangapInit()/lcdInit(). The display and its touch are the component's
 * (CONFIG_LCD_* / CONFIG_LCD_TOUCH_*); we supply only the button. No pointer
 * device: the touch screen is the pointer. lcdSetHasKeyboard() is deliberately
 * not called — the board has no hardware keyboard, so the on-screen keyboard
 * must stay active. */
void WismeshTapLcdInput::onStart() {
    static const lcd_input_t ops = {
        .init         = tapInputInit,
        .touch_read   = nullptr,
        .pointer_read = nullptr,
        .click_read   = tapClickRead,
    };
    lcdSetInput(&ops);
}
