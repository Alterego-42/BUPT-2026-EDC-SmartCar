#include "ti_msp_dl_config.h"
#include "board_pins.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "k230.h"
#include "motor.h"
#include "oled.h"
#include "pca9685.h"
#include "tasks.h"
#include "ui.h"

static uint32_t g_now_ms;
static uint32_t g_last_oled_ms;

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static uint8_t slot_from_hold(uint16_t held_ms)
{
    uint32_t slot = ((uint32_t) held_ms / BOARD_TASK_SLOT_MS) + 1U;
    if (slot > BOARD_TASK_MAX_SLOT) {
        slot = BOARD_TASK_MAX_SLOT;
    }
    return (uint8_t) slot;
}

static void show_selection(uint16_t held_ms)
{
    uint8_t slot = slot_from_hold(held_ms);
    app_task_t task = tasks_from_slot(slot);

    if ((g_now_ms - g_last_oled_ms) < BOARD_OLED_REFRESH_MS) {
        return;
    }
    g_last_oled_ms = g_now_ms;

    oled_clear();
    oled_print(0, 0, "HOLD SELECT");
    oled_print_i32(1, 0, "SLOT", slot);
    oled_print(2, 0, tasks_name(task));
    oled_print_i32(3, 0, "MS", held_ms);
}

static void show_status(void)
{
    task_snapshot_t snap;

    if ((g_now_ms - g_last_oled_ms) < BOARD_OLED_REFRESH_MS) {
        return;
    }
    g_last_oled_ms = g_now_ms;

    tasks_snapshot(g_now_ms, &snap);
    oled_show_status((uint8_t) snap.task, snap.state, snap.gray.position,
        snap.gray.mask, snap.yaw_x10, snap.distance_mm);
}

static void start_slot(uint8_t slot)
{
    app_task_t task = tasks_from_slot(slot);
    if (task == APP_TASK_NONE) {
        ui_beep_ms(500U);
        return;
    }
    tasks_start(task, g_now_ms);
    g_last_oled_ms = 0U;
}

int main(void)
{
    SYSCFG_DL_init();
    motor_init();
    grayscale_init();
    encoder_init();
    imu_init();
    ui_init();
    oled_init();
    pca9685_init();
    laser_init();
    k230_init();
    tasks_init();

    motor_disable();
    ui_beep_ms(80U);
    oled_clear();
    oled_print(0, 0, "BUPT EDC CAR");
    oled_print(1, 0, "READY");

    while (1) {
        uint16_t held_ms = 0U;

        delay_ms(BOARD_LOOP_PERIOD_MS);
        g_now_ms += BOARD_LOOP_PERIOD_MS;

        imu_poll(g_now_ms);
        k230_poll(g_now_ms);
        ui_update_10ms();

        if (ui_button_is_pressed() && !tasks_running()) {
            motor_disable();
            show_selection(ui_button_hold_ms());
        }

        if (ui_button_release_event(&held_ms)) {
            if (tasks_running()) {
                tasks_stop();
                ui_beep_ms(220U);
                g_last_oled_ms = 0U;
            } else {
                start_slot(slot_from_hold(held_ms));
            }
        }

        tasks_update(g_now_ms);

        if (!ui_button_is_pressed()) {
            show_status();
        }
    }
}
