#include "ui.h"

#include "board_config.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static uint8_t g_raw_pressed;
static uint8_t g_stable_pressed;
static uint8_t g_long_sent;
static uint32_t g_last_change_ms;
static uint32_t g_press_start_ms;
static uint32_t g_beep_until_ms;

static bool button_is_pressed_raw(void)
{
    return (DL_GPIO_readPins(GPIO_UI_START_KEY_PORT, GPIO_UI_START_KEY_PIN) == 0U);
}

void ui_init(void)
{
    g_raw_pressed = button_is_pressed_raw() ? 1U : 0U;
    g_stable_pressed = g_raw_pressed;
    g_long_sent = 0U;
    g_last_change_ms = millis();
    g_press_start_ms = millis();
    g_beep_until_ms = 0U;
    ui_leds(false, false);
    ui_beep(false);
}

ui_button_event_t ui_poll_button(void)
{
    uint32_t now = millis();
    uint8_t raw = button_is_pressed_raw() ? 1U : 0U;

    if (raw != g_raw_pressed) {
        g_raw_pressed = raw;
        g_last_change_ms = now;
    }

    if ((uint32_t)(now - g_last_change_ms) < BOARD_BUTTON_DEBOUNCE_MS) {
        return UI_BUTTON_NONE;
    }

    if (g_stable_pressed != raw) {
        g_stable_pressed = raw;
        if (g_stable_pressed) {
            g_press_start_ms = now;
            g_long_sent = 0U;
        } else {
            if (!g_long_sent) {
                return UI_BUTTON_SHORT;
            }
        }
    }

    if (g_stable_pressed && !g_long_sent &&
        (uint32_t)(now - g_press_start_ms) >= BOARD_BUTTON_LONG_MS) {
        g_long_sent = 1U;
        return UI_BUTTON_LONG;
    }

    return UI_BUTTON_NONE;
}

void ui_red(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_UI_LED_R_PORT, GPIO_UI_LED_R_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_UI_LED_R_PORT, GPIO_UI_LED_R_PIN);
    }
}

void ui_green(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_UI_LED_G_PORT, GPIO_UI_LED_G_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_UI_LED_G_PORT, GPIO_UI_LED_G_PIN);
    }
}

void ui_leds(bool red, bool green)
{
    ui_red(red);
    ui_green(green);
}

void ui_beep(bool on)
{
#if BOARD_BEEP_ACTIVE_HIGH
    if (on) {
        DL_GPIO_setPins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN);
    }
#else
    if (on) {
        DL_GPIO_clearPins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN);
    } else {
        DL_GPIO_setPins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN);
    }
#endif
}

void ui_beep_for(uint32_t ms)
{
    g_beep_until_ms = millis() + ms;
    ui_beep(true);
}

void ui_task(void)
{
    if (g_beep_until_ms != 0U && (int32_t)(millis() - g_beep_until_ms) >= 0) {
        g_beep_until_ms = 0U;
        ui_beep(false);
    }
}

void ui_signal_mode(uint8_t mode)
{
    for (uint8_t i = 0; i <= mode; i++) {
        ui_green(true);
        ui_beep(true);
        delay_ms(70U);
        ui_green(false);
        ui_beep(false);
        delay_ms(120U);
    }
}

void ui_signal_error(void)
{
    for (uint8_t i = 0; i < 3U; i++) {
        ui_leds(true, false);
        ui_beep(true);
        delay_ms(180U);
        ui_leds(false, false);
        ui_beep(false);
        delay_ms(120U);
    }
}

void ui_signal_keypoint(uint8_t index)
{
    for (uint8_t i = 0; i < index; i++) {
        ui_leds(false, true);
        ui_beep(true);
        delay_ms(BOARD_KEYPOINT_BEEP_MS);
        ui_leds(false, false);
        ui_beep(false);
        delay_ms(90U);
    }
}
