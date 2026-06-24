#include "ui.h"
#include "board_pins.h"

static uint16_t g_beep_remaining_ms;
static uint8_t g_button_history = 0x00U;
static bool g_button_pressed;
static bool g_button_short_event;
static bool g_button_long_event;
static bool g_button_long_reported;
static uint16_t g_button_hold_ms;
static bool g_button_release_event;
static uint16_t g_button_last_hold_ms;

static bool raw_button_pressed(void)
{
    return (DL_GPIO_readPins(BOARD_KEY_PORT, BOARD_KEY_PIN) == 0U);
}

void ui_init(void)
{
    DL_GPIO_initDigitalInputFeatures(BOARD_KEY_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BOARD_BEEP_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_LED_R_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_LED_G_IOMUX);

    DL_GPIO_clearPins(BOARD_BEEP_PORT, BOARD_BEEP_PIN);
    DL_GPIO_clearPins(BOARD_LED_R_PORT, BOARD_LED_R_PIN);
    DL_GPIO_clearPins(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
    DL_GPIO_enableOutput(BOARD_BEEP_PORT, BOARD_BEEP_PIN);
    DL_GPIO_enableOutput(BOARD_LED_R_PORT, BOARD_LED_R_PIN);
    DL_GPIO_enableOutput(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
}

void ui_update_10ms(void)
{
    g_button_history = (uint8_t) ((g_button_history << 1) |
                                  (raw_button_pressed() ? 1U : 0U));

    bool stable_pressed = ((g_button_history & 0x0FU) == 0x0FU);
    bool stable_released = ((g_button_history & 0x0FU) == 0x00U);

    if (stable_pressed) {
        if (!g_button_pressed) {
            g_button_pressed = true;
            g_button_hold_ms = 0U;
            g_button_long_reported = false;
        } else if (!g_button_long_reported) {
            g_button_hold_ms += BOARD_LOOP_PERIOD_MS;
            if (g_button_hold_ms >= BOARD_KEY_LONG_PRESS_MS) {
                g_button_long_event = true;
                g_button_long_reported = true;
            }
        } else if (g_button_hold_ms < 60000U) {
            g_button_hold_ms += BOARD_LOOP_PERIOD_MS;
        }
    } else if (stable_released && g_button_pressed) {
        g_button_last_hold_ms = g_button_hold_ms;
        g_button_release_event = true;
        if (!g_button_long_reported) {
            g_button_short_event = true;
        }
        g_button_pressed = false;
        g_button_hold_ms = 0U;
        g_button_long_reported = false;
    }

    if (g_beep_remaining_ms > 0U) {
        if (g_beep_remaining_ms <= BOARD_LOOP_PERIOD_MS) {
            g_beep_remaining_ms = 0U;
            DL_GPIO_clearPins(BOARD_BEEP_PORT, BOARD_BEEP_PIN);
        } else {
            g_beep_remaining_ms -= BOARD_LOOP_PERIOD_MS;
            DL_GPIO_setPins(BOARD_BEEP_PORT, BOARD_BEEP_PIN);
        }
    }
}

bool ui_start_button_pressed_edge(void)
{
    bool event = g_button_short_event;
    g_button_short_event = false;
    return event;
}

bool ui_start_button_long_press_event(void)
{
    bool event = g_button_long_event;
    g_button_long_event = false;
    return event;
}

bool ui_button_is_pressed(void)
{
    return g_button_pressed;
}

uint16_t ui_button_hold_ms(void)
{
    return g_button_hold_ms;
}

bool ui_button_release_event(uint16_t *held_ms)
{
    bool event = g_button_release_event;
    if (event && (held_ms != 0)) {
        *held_ms = g_button_last_hold_ms;
    }
    g_button_release_event = false;
    return event;
}

void ui_set_status_leds(bool red_on, bool green_on)
{
    if (red_on) {
        DL_GPIO_setPins(BOARD_LED_R_PORT, BOARD_LED_R_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_LED_R_PORT, BOARD_LED_R_PIN);
    }

    if (green_on) {
        DL_GPIO_setPins(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
    }
}

void ui_set_running(bool running)
{
    ui_set_status_leds(false, running);
}

void ui_set_error(bool error)
{
    ui_set_status_leds(error, false);
}

void ui_beep_ms(uint16_t duration_ms)
{
    g_beep_remaining_ms = duration_ms;
    if (duration_ms > 0U) {
        DL_GPIO_setPins(BOARD_BEEP_PORT, BOARD_BEEP_PIN);
    }
}

void ui_keypoint_signal(uint8_t index)
{
    (void) index;
    ui_beep_ms(120U);
    DL_GPIO_togglePins(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
}

void ui_finish_signal(void)
{
    ui_beep_ms(600U);
    DL_GPIO_setPins(BOARD_LED_G_PORT, BOARD_LED_G_PIN);
    DL_GPIO_setPins(BOARD_LED_R_PORT, BOARD_LED_R_PIN);
}
