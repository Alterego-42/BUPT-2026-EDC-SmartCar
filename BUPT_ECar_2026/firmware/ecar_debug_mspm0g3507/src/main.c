#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "line_follow.h"
#include "motor.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include "ui.h"
#include "util.h"

typedef enum {
    MODE_MOTOR_STEP = 0,
    MODE_ENCODER_CHECK,
    MODE_GRAY_MONITOR,
    MODE_LINE_FOLLOW,
    MODE_KEYPOINT_RUN,
    MODE_B_STOP_RUN,
} debug_mode_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_ACTIVE,
    RUN_FINISHED,
    RUN_ERROR,
} run_state_t;

static debug_mode_t g_mode = MODE_MOTOR_STEP;
static run_state_t g_state = RUN_IDLE;
static uint32_t g_step_start_ms;
static uint32_t g_last_blink_ms;
static uint8_t g_phase;
static uint8_t g_next_keypoint;
static uint32_t g_pause_until_ms;
static bool g_keypoint_line_known;
static bool g_keypoint_stable_line_seen;
static bool g_keypoint_candidate_active;
static bool g_keypoint_candidate_line_seen;
static uint32_t g_keypoint_candidate_since_ms;
static uint32_t g_keypoint_signal_until_ms;

static const int32_t k_keypoint_mm[4] = {
    BOARD_KEYPOINT_B_MM,
    BOARD_KEYPOINT_C_MM,
    BOARD_KEYPOINT_D_MM,
    BOARD_KEYPOINT_A_MM,
};

static const bool k_keypoint_line_seen_after[4] = {
    true, false, true, false,
};

static void all_stop(void)
{
    line_follow_stop();
    ui_leds(false, false);
}

static void reset_keypoint_detector(void)
{
    g_keypoint_line_known = false;
    g_keypoint_stable_line_seen = false;
    g_keypoint_candidate_active = false;
    g_keypoint_candidate_line_seen = false;
    g_keypoint_candidate_since_ms = 0;
    g_keypoint_signal_until_ms = 0;
}

static void enter_idle(void)
{
    all_stop();
    g_state = RUN_IDLE;
    g_phase = 0;
    reset_keypoint_detector();
}

static void finish_run(void)
{
    all_stop();
    ui_leds(false, true);
    ui_beep_for(600U);
    g_state = RUN_FINISHED;
}

static void error_stop(void)
{
    all_stop();
    ui_signal_error();
    g_state = RUN_ERROR;
}

static void start_mode(void)
{
    encoder_reset();
    g_step_start_ms = millis();
    g_phase = 0;
    g_next_keypoint = 0;
    g_pause_until_ms = 0;
    reset_keypoint_detector();
    line_follow_reset();
    g_state = RUN_ACTIVE;
    ui_leds(false, true);
    ui_beep_for(120U);
}

static void signal_keypoint(uint8_t point_index)
{
    (void)point_index;
    g_keypoint_signal_until_ms = millis() + BOARD_KEYPOINT_BEEP_MS;
    ui_leds(true, true);
    ui_beep_for(BOARD_KEYPOINT_BEEP_MS);
}

static void set_keypoint_run_led(void)
{
    if (g_keypoint_signal_until_ms != 0U &&
        (int32_t)(millis() - g_keypoint_signal_until_ms) < 0) {
        ui_leds(true, true);
    } else {
        g_keypoint_signal_until_ms = 0;
        ui_leds(false, true);
    }
}

static bool keypoint_transition_detected(bool line_seen, uint32_t now_ms, int32_t distance_mm)
{
    if (g_next_keypoint >= 4U) {
        return false;
    }

    if (!g_keypoint_line_known) {
        g_keypoint_line_known = true;
        g_keypoint_stable_line_seen = line_seen;
        return false;
    }

    if (line_seen == g_keypoint_stable_line_seen) {
        g_keypoint_candidate_active = false;
        return false;
    }

    if (!g_keypoint_candidate_active || g_keypoint_candidate_line_seen != line_seen) {
        g_keypoint_candidate_active = true;
        g_keypoint_candidate_line_seen = line_seen;
        g_keypoint_candidate_since_ms = now_ms;
        return false;
    }

    if ((uint32_t)(now_ms - g_keypoint_candidate_since_ms) < BOARD_KEYPOINT_TRANSITION_STABLE_MS) {
        return false;
    }

    bool expected_line_seen = k_keypoint_line_seen_after[g_next_keypoint];
    int32_t gate_mm = k_keypoint_mm[g_next_keypoint] - BOARD_KEYPOINT_GATE_BEFORE_MM;
    if (gate_mm < 0) {
        gate_mm = 0;
    }

    if (line_seen != expected_line_seen || distance_mm < gate_mm) {
        return false;
    }

    g_keypoint_stable_line_seen = line_seen;
    g_keypoint_candidate_active = false;
    return true;
}

static void switch_mode(void)
{
    g_mode = (debug_mode_t)(((uint8_t)g_mode + 1U) % BOARD_MODE_COUNT);
    enter_idle();
    ui_signal_mode((uint8_t)g_mode);
}

static void blink_idle(void)
{
    uint32_t now = millis();
    if ((uint32_t)(now - g_last_blink_ms) < 500U) {
        return;
    }
    g_last_blink_ms = now;

    bool red = ((now / 500U) & 0x01U) != 0U;
    bool green = (((uint8_t)g_mode + 1U) & 0x01U) != 0U;
    ui_leds(red, green && red);
}

static void task_motor_step(void)
{
    uint32_t elapsed = millis() - g_step_start_ms;
    uint32_t slot = elapsed % 4200U;

    if (slot < 900U) {
        motor_point_left(BOARD_MOTOR_TEST_PERCENT);
        ui_leds(false, true);
    } else if (slot < 1400U) {
        motor_stop();
        ui_leds(false, false);
    } else if (slot < 2300U) {
        motor_point_right(BOARD_MOTOR_TEST_PERCENT);
        ui_leds(true, false);
    } else if (slot < 2800U) {
        motor_stop();
        ui_leds(false, false);
    } else if (slot < 3500U) {
        motor_set_percent(BOARD_MOTOR_TEST_PERCENT, BOARD_MOTOR_TEST_PERCENT);
        ui_leds(true, true);
    } else {
        motor_stop();
        ui_leds(false, false);
    }
}

static void task_encoder_check(void)
{
    static int32_t last_left;
    static int32_t last_right;

    uint32_t elapsed = millis() - g_step_start_ms;
    if (g_phase == 0U) {
        encoder_reset();
        last_left = 0;
        last_right = 0;
        g_phase = 1U;
        g_step_start_ms = millis();
        motor_point_left(BOARD_MOTOR_TEST_PERCENT);
        return;
    }

    if (g_phase == 1U && elapsed >= 850U) {
        motor_stop();
        encoder_snapshot_t snap = encoder_get();
        last_left = snap.left_ticks;
        if (abs_i32(last_left) >= BOARD_ENCODER_SELFTEST_MIN_TICKS) {
            ui_beep_for(90U);
            ui_green(true);
        } else {
            ui_red(true);
            ui_beep_for(500U);
        }
        g_phase = 2U;
        g_step_start_ms = millis();
        return;
    }

    if (g_phase == 2U && elapsed >= 650U) {
        ui_leds(false, false);
        motor_point_right(BOARD_MOTOR_TEST_PERCENT);
        g_phase = 3U;
        g_step_start_ms = millis();
        return;
    }

    if (g_phase == 3U && elapsed >= 850U) {
        motor_stop();
        encoder_snapshot_t snap = encoder_get();
        last_right = snap.right_ticks;
        if (abs_i32(last_right) >= BOARD_ENCODER_SELFTEST_MIN_TICKS) {
            ui_beep_for(160U);
            ui_green(true);
        } else {
            ui_red(true);
            ui_beep_for(500U);
        }
        g_phase = 4U;
        g_step_start_ms = millis();
        return;
    }

    if (g_phase == 4U && elapsed >= 1200U) {
        ui_leds(false, false);
        encoder_reset();
        g_phase = 0U;
    }
}

static void task_gray_monitor(void)
{
    grayscale_sample_t gray = grayscale_read();
    if (gray.line_seen) {
        ui_green(true);
        ui_red(gray.error < -800 || gray.error > 800);
    } else {
        ui_green(false);
        ui_red(true);
    }
    motor_stop();
}

static void task_line_follow_only(void)
{
    encoder_snapshot_t enc = encoder_get();
    if ((uint32_t)(millis() - g_step_start_ms) < BOARD_LINE_START_DELAY_MS) {
        motor_stop();
        ui_green((millis() / 120U) & 0x01U);
        return;
    }

    line_follow_status_t status = line_follow_update(millis(), enc.distance_mm);
    if (status.fault) {
        error_stop();
        return;
    }

    ui_green(status.gray.line_seen);
    ui_red(status.lost);
}

static void handle_keypoints(bool stop_at_b)
{
    encoder_snapshot_t enc = encoder_get();
    if ((uint32_t)(millis() - g_step_start_ms) < BOARD_LINE_START_DELAY_MS) {
        motor_stop();
        ui_green((millis() / 120U) & 0x01U);
        return;
    }

    if (g_pause_until_ms != 0U) {
        motor_stop();
        if ((int32_t)(millis() - g_pause_until_ms) >= 0) {
            g_pause_until_ms = 0U;
            line_follow_reset_at(enc.distance_mm);
            reset_keypoint_detector();
            set_keypoint_run_led();
        }
        return;
    }

    line_follow_status_t status = line_follow_update_ex(millis(), enc.distance_mm, true);
    if (status.fault) {
        error_stop();
        return;
    }

    if (keypoint_transition_detected(status.gray.line_seen, millis(), enc.distance_mm)) {
        uint8_t point_index = (uint8_t)(g_next_keypoint + 1U);
        signal_keypoint(point_index);
        if (stop_at_b && g_next_keypoint == 0U) {
            g_pause_until_ms = millis() + BOARD_KEYPOINT_B_STOP_MS;
            motor_stop();
        }
        g_next_keypoint++;
        if (g_next_keypoint >= 4U) {
            finish_run();
            return;
        }
    }

    set_keypoint_run_led();
}

static void active_task(void)
{
    switch (g_mode) {
    case MODE_MOTOR_STEP:
        task_motor_step();
        break;
    case MODE_ENCODER_CHECK:
        task_encoder_check();
        break;
    case MODE_GRAY_MONITOR:
        task_gray_monitor();
        break;
    case MODE_LINE_FOLLOW:
        task_line_follow_only();
        break;
    case MODE_KEYPOINT_RUN:
        handle_keypoints(false);
        break;
    case MODE_B_STOP_RUN:
        handle_keypoints(true);
        break;
    default:
        error_stop();
        break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    timebase_init();
    ui_init();
    motor_init();
    grayscale_init();
    encoder_init();
    imu_init();

    ui_signal_mode((uint8_t)g_mode);

    while (1) {
        ui_task();

        ui_button_event_t button = ui_poll_button();
        if (button == UI_BUTTON_SHORT) {
            if (g_state == RUN_ACTIVE) {
                enter_idle();
            } else {
                start_mode();
            }
        } else if (button == UI_BUTTON_LONG) {
            if (g_state == RUN_ACTIVE) {
                enter_idle();
            } else {
                switch_mode();
            }
        }

        if (g_state == RUN_ACTIVE) {
            active_task();
        } else {
            blink_idle();
        }
    }
}
