#include "sound_light.h"

#include "board_config.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include "ui.h"

static uint8_t g_active;
static uint8_t g_blink;
static uint8_t g_output_on;
static uint32_t g_until_ms;
static uint32_t g_next_toggle_ms;
static uint32_t g_on_ms;
static uint32_t g_off_ms;

static int32_t time_diff(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target);
}

static void set_output(uint8_t on)
{
    g_output_on = on ? 1U : 0U;
    ui_beep(g_output_on != 0U);
    ui_leds(g_output_on != 0U, g_output_on != 0U);
}

static void start_solid(uint32_t duration_ms)
{
    g_active = 1U;
    g_blink = 0U;
    g_until_ms = millis() + duration_ms;
    set_output(1U);
}

static void start_blink(uint32_t total_ms, uint32_t on_ms, uint32_t off_ms)
{
    g_active = 1U;
    g_blink = 1U;
    g_on_ms = on_ms;
    g_off_ms = off_ms;
    g_until_ms = millis() + total_ms;
    g_next_toggle_ms = millis() + on_ms;
    set_output(1U);
}

void sound_light_init(void)
{
    g_active = 0U;
    g_blink = 0U;
    g_output_on = 0U;
    g_until_ms = 0U;
    g_next_toggle_ms = 0U;
    g_on_ms = 0U;
    g_off_ms = 0U;
}

void sound_light_task(void)
{
    uint32_t now = millis();

    if (!g_active) {
        return;
    }

    if (time_diff(now, g_until_ms) >= 0) {
        sound_light_cancel();
        return;
    }

    if (g_blink && time_diff(now, g_next_toggle_ms) >= 0) {
        set_output(g_output_on ? 0U : 1U);
        g_next_toggle_ms = now + (g_output_on ? g_on_ms : g_off_ms);
    }
}

void sound_light_cancel(void)
{
    g_active = 0U;
    g_blink = 0U;
    g_output_on = 0U;
    ui_beep(false);
    ui_leds(false, false);
}

bool sound_light_active(void)
{
    sound_light_task();
    return g_active != 0U;
}

void sound_light_keypoint(uint8_t point_index)
{
    (void)point_index;
    start_solid(BOARD_KEYPOINT_BEEP_MS);
}

void sound_light_stop(void)
{
    start_solid(1000U);
}

void sound_light_complete(void)
{
    start_blink(3000U, 250U, 250U);
}

void sound_light_error(void)
{
    start_blink(600U, 100U, 100U);
}

void sound_light_aiming_ok(void)
{
    start_blink(220U, 10U, 100U);
}

void beep_on(void)
{
    ui_beep(true);
}

void beep_off(void)
{
    ui_beep(false);
}

void beep_toggle(void)
{
    DL_GPIO_togglePins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN);
}

void led_on(void)
{
    ui_leds(true, true);
}

void led_off(void)
{
    ui_leds(false, false);
}

void led_toggle(void)
{
    DL_GPIO_togglePins(GPIO_UI_LED_R_PORT, GPIO_UI_LED_R_PIN);
    DL_GPIO_togglePins(GPIO_UI_LED_G_PORT, GPIO_UI_LED_G_PIN);
}

void signal_short(void)
{
    start_solid(5U);
}

void signal_keypoint(void)
{
    sound_light_keypoint(0U);
}

void signal_stop(void)
{
    sound_light_stop();
}

void signal_complete(void)
{
    sound_light_complete();
}

void signal_error(void)
{
    sound_light_error();
}

void signal_aiming_ok(void)
{
    sound_light_aiming_ok();
}

uint8_t is_beep_on(void)
{
    bool level_high = DL_GPIO_readPins(GPIO_UI_BEEP_PORT, GPIO_UI_BEEP_PIN) != 0U;
#if BOARD_BEEP_ACTIVE_HIGH
    return level_high ? 1U : 0U;
#else
    return level_high ? 0U : 1U;
#endif
}

uint8_t is_led_on(void)
{
    return (DL_GPIO_readPins(GPIO_UI_LED_R_PORT, GPIO_UI_LED_R_PIN) != 0U ||
        DL_GPIO_readPins(GPIO_UI_LED_G_PORT, GPIO_UI_LED_G_PIN) != 0U) ? 1U : 0U;
}
