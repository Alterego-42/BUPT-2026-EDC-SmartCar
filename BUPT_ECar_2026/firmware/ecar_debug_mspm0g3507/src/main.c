#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"
#include "encoder.h"
#include "gimbal.h"
#include "gimbal_route.h"
#include "grayscale.h"
#include "imu.h"
#include "k230_uart.h"
#include "line_follow.h"
#include "motor.h"
#include "sound_light.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include "ui.h"
#include "util.h"

typedef enum {
    MODE_ONE_LAP_RUN = 0,
    MODE_TARGET_POINT_RUN,
    MODE_LINE_GIMBAL_RUN,
    MODE_LINE_FOLLOW,
    MODE_KEYPOINT_RUN,
    MODE_GIMBAL_TEST,
    MODE_GIMBAL_SWEEP_TEST,
} debug_mode_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_ACTIVE,
    RUN_FINISHED,
    RUN_ERROR,
} run_state_t;

typedef enum {
    KEYPOINT_SURFACE_UNKNOWN = 0,
    KEYPOINT_SURFACE_WHITE,
    KEYPOINT_SURFACE_BLACK,
} keypoint_surface_t;

typedef enum {
    TARGET_PHASE_IDLE = 0,
    TARGET_PHASE_SCAN_CCW,
    TARGET_PHASE_SCAN_CW,
    TARGET_PHASE_AIM,
    TARGET_PHASE_WAIT_LOST,
    TARGET_PHASE_WAIT_REPLACE,
    TARGET_PHASE_RETURN_ZERO,
    TARGET_PHASE_SEARCH_RETRY,
} target_mode_phase_t;

static debug_mode_t g_mode = MODE_ONE_LAP_RUN;
static run_state_t g_state = RUN_IDLE;
static uint32_t g_step_start_ms;
static uint32_t g_last_blink_ms;
static uint8_t g_phase;
static uint8_t g_next_keypoint;
static uint32_t g_pause_until_ms;
static keypoint_surface_t g_keypoint_stable_surface;
static bool g_keypoint_candidate_active;
static keypoint_surface_t g_keypoint_candidate_surface;
static uint32_t g_keypoint_candidate_since_ms;
static uint32_t g_keypoint_last_signal_ms;
static target_mode_phase_t g_target_phase;
static target_mode_phase_t g_target_after_return_phase;
static uint8_t g_target_cycle;
static uint32_t g_target_phase_start_ms;
static bool g_target_candidate_active;
static uint32_t g_target_candidate_since_ms;
static uint32_t g_target_valid_frames;
static uint32_t g_target_last_valid_frame_count;
static uint32_t g_target_aim_stable_since_ms;
static uint32_t g_target_lost_since_ms;
static int32_t g_target_scan_start_yaw_steps;
static int16_t g_target_found_yaw_cdeg[BOARD_TARGET_MODE_CYCLES];
static int16_t g_target_found_pitch_cdeg[BOARD_TARGET_MODE_CYCLES];

static const keypoint_surface_t k_keypoint_surface_after[4] = {
    KEYPOINT_SURFACE_BLACK,
    KEYPOINT_SURFACE_WHITE,
    KEYPOINT_SURFACE_BLACK,
    KEYPOINT_SURFACE_WHITE,
};

static void handle_keypoints(bool stop_at_b, bool loop_forever);
static void target_mode_start(void);
static void task_target_point_run(void);

static bool mode_uses_gimbal(debug_mode_t mode)
{
    return mode == MODE_TARGET_POINT_RUN ||
        mode == MODE_LINE_GIMBAL_RUN ||
        mode == MODE_GIMBAL_TEST ||
        mode == MODE_GIMBAL_SWEEP_TEST;
}

static void all_stop(void)
{
    gimbal_route_enable(false);
    gimbal_stop();
    line_follow_stop();
    ui_leds(false, false);
}

static void reset_keypoint_detector(void)
{
    g_keypoint_stable_surface = KEYPOINT_SURFACE_UNKNOWN;
    g_keypoint_candidate_active = false;
    g_keypoint_candidate_surface = KEYPOINT_SURFACE_UNKNOWN;
    g_keypoint_candidate_since_ms = 0;
    g_keypoint_last_signal_ms = 0;
}

static void enter_idle(void)
{
    sound_light_cancel();
    all_stop();
    g_state = RUN_IDLE;
    g_phase = 0;
    reset_keypoint_detector();
}

static void finish_run(void)
{
    all_stop();
    sound_light_stop();
    g_state = RUN_FINISHED;
}

static void complete_run(void)
{
    all_stop();
    sound_light_complete();
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
    gimbal_route_reset();
    gimbal_route_enable(g_mode == MODE_LINE_GIMBAL_RUN);
    motor_stop();
    if (mode_uses_gimbal(g_mode)) {
        gimbal_start();
    }
    if (g_mode == MODE_TARGET_POINT_RUN) {
        target_mode_start();
    }
    g_state = RUN_ACTIVE;
    ui_leds(false, true);
    ui_beep_for(120U);
}

static void set_keypoint_run_led(void)
{
    if (sound_light_active()) {
        ui_leds(true, true);
    } else {
        ui_leds(false, true);
    }
}

static keypoint_surface_t keypoint_surface_from_gray(const grayscale_sample_t *gray)
{
    if (gray->black_count == 0U) {
        return KEYPOINT_SURFACE_WHITE;
    }

    return KEYPOINT_SURFACE_BLACK;
}

static bool keypoint_transition_detected(const grayscale_sample_t *gray, uint32_t now_ms,
    int32_t distance_mm)
{
    keypoint_surface_t surface = keypoint_surface_from_gray(gray);
    keypoint_surface_t expected_after;
    bool matched;

    (void)distance_mm;
    if (g_next_keypoint >= 4U) {
        return false;
    }

    expected_after = k_keypoint_surface_after[g_next_keypoint];

    if (g_keypoint_stable_surface == KEYPOINT_SURFACE_UNKNOWN) {
        g_keypoint_stable_surface = surface;
        g_keypoint_candidate_active = false;
        return false;
    }

    if (surface == g_keypoint_stable_surface) {
        g_keypoint_candidate_active = false;
        return false;
    }

    if (!g_keypoint_candidate_active || g_keypoint_candidate_surface != surface) {
        g_keypoint_candidate_active = true;
        g_keypoint_candidate_surface = surface;
        g_keypoint_candidate_since_ms = now_ms;
        return false;
    }

    if ((uint32_t)(now_ms - g_keypoint_candidate_since_ms) < BOARD_KEYPOINT_TRANSITION_STABLE_MS) {
        return false;
    }

    g_keypoint_stable_surface = surface;
    g_keypoint_candidate_active = false;

    matched = surface == expected_after;
    if (matched && g_keypoint_last_signal_ms != 0U &&
        (uint32_t)(now_ms - g_keypoint_last_signal_ms) < BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS) {
        matched = false;
    }

    if (matched) {
        g_keypoint_last_signal_ms = now_ms;
    }

    return matched;
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

static void task_one_lap_run(void)
{
    handle_keypoints(false, false);
}

static void target_mode_reset_detection(void)
{
    g_target_candidate_active = false;
    g_target_candidate_since_ms = 0;
    g_target_valid_frames = 0;
    g_target_last_valid_frame_count = 0;
}

static void target_mode_enter(target_mode_phase_t phase)
{
    gimbal_steps_t steps;

    g_target_phase = phase;
    g_target_phase_start_ms = millis();
    g_target_aim_stable_since_ms = 0;
    g_target_lost_since_ms = 0;
    if (phase == TARGET_PHASE_SCAN_CCW || phase == TARGET_PHASE_SCAN_CW ||
        phase == TARGET_PHASE_AIM) {
        target_mode_reset_detection();
    }
    if (phase == TARGET_PHASE_SCAN_CCW || phase == TARGET_PHASE_SCAN_CW) {
        steps = gimbal_get_steps();
        g_target_scan_start_yaw_steps = steps.yaw_steps;
    }
}

static void target_mode_start(void)
{
    gimbal_zero_pose();
    g_target_cycle = 0;
    g_target_after_return_phase = TARGET_PHASE_SCAN_CCW;
    target_mode_reset_detection();
    target_mode_enter(TARGET_PHASE_SCAN_CCW);
}

static bool target_mode_get_valid_frame(k230_frame_t *frame)
{
    return k230_uart_get_fresh(frame, BOARD_K230_VISION_TIMEOUT_MS) &&
        frame->valid;
}

static bool target_mode_seen_stable(const k230_frame_t *frame, uint32_t now_ms)
{
    if (!g_target_candidate_active) {
        g_target_candidate_active = true;
        g_target_candidate_since_ms = now_ms;
        g_target_valid_frames = 0;
        g_target_last_valid_frame_count = 0;
    }

    if (frame->frame_count != g_target_last_valid_frame_count) {
        g_target_last_valid_frame_count = frame->frame_count;
        g_target_valid_frames++;
    }

    return g_target_valid_frames >= BOARD_TARGET_MODE_VALID_MIN_FRAMES &&
        (uint32_t)(now_ms - g_target_candidate_since_ms) >=
            BOARD_TARGET_MODE_VALID_STABLE_MS;
}

static int16_t target_mode_limit_command(int16_t command_cdeg)
{
    return clamp_i16(command_cdeg, -BOARD_TARGET_MODE_AIM_MAX_CDEG,
        BOARD_TARGET_MODE_AIM_MAX_CDEG);
}

static bool target_mode_aim_error_stable(const k230_frame_t *frame)
{
    return abs_i32(frame->yaw_cdeg) <= BOARD_TARGET_MODE_AIM_STABLE_CDEG &&
        abs_i32(frame->pitch_cdeg) <= BOARD_TARGET_MODE_AIM_STABLE_CDEG;
}

static bool target_mode_scan_step_limit_reached(void)
{
    gimbal_steps_t steps = gimbal_get_steps();
    int32_t delta = steps.yaw_steps - g_target_scan_start_yaw_steps;
    int32_t limit = (g_target_phase == TARGET_PHASE_SCAN_CCW) ?
        BOARD_TARGET_MODE_SEARCH_HALF_STEPS : BOARD_TARGET_MODE_SEARCH_FULL_STEPS;

    if (delta < 0) {
        delta = -delta;
    }

    return delta >= limit;
}

static void target_mode_store_found_pose(void)
{
    gimbal_pose_t pose = gimbal_get_pose();

    if (g_target_cycle < BOARD_TARGET_MODE_CYCLES) {
        g_target_found_yaw_cdeg[g_target_cycle] = pose.yaw_cdeg;
        g_target_found_pitch_cdeg[g_target_cycle] = pose.pitch_cdeg;
    }
}

static void target_mode_status_led(bool red, bool green)
{
    if (!sound_light_active()) {
        ui_leds(red, green);
    }
}

static void task_target_point_run(void)
{
    k230_frame_t frame;
    uint32_t now = millis();
    bool valid = target_mode_get_valid_frame(&frame);
    bool found = valid && frame.target_ok;
    int16_t yaw_cmd;
    int16_t pitch_cmd;
    int16_t ccw_target = (int16_t)(
        BOARD_TARGET_MODE_CCW_SIGN * BOARD_TARGET_MODE_SEARCH_SOFT_CDEG);
    int16_t cw_target = (int16_t)(-ccw_target);

    motor_stop();

#if BOARD_TARGET_MODE_HALF_TURN_TEST
    (void)frame;
    (void)valid;
    (void)yaw_cmd;
    (void)pitch_cmd;
    (void)cw_target;
    if (gimbal_drive_to_cdeg(ccw_target, 0,
            BOARD_TARGET_MODE_SEARCH_TOL_CDEG,
            BOARD_TARGET_MODE_ZERO_TOL_CDEG,
            BOARD_TARGET_MODE_SCAN_STEP_MS,
            1U)) {
        complete_run();
    } else {
        target_mode_status_led(false, (now / 180U) & 0x01U);
    }
    return;
#endif

    switch (g_target_phase) {
    case TARGET_PHASE_SCAN_CCW:
    case TARGET_PHASE_SCAN_CW:
        if (found) {
            if (!target_mode_seen_stable(&frame, now)) {
                target_mode_status_led(false, true);
                break;
            }
            target_mode_store_found_pose();
            sound_light_aiming_ok();
            target_mode_enter(TARGET_PHASE_AIM);
            return;
        } else {
            if (g_target_candidate_active &&
                (uint32_t)(now - g_target_candidate_since_ms) <
                    BOARD_TARGET_MODE_CANDIDATE_HOLD_MS) {
                target_mode_status_led(false, true);
                break;
            }
            target_mode_reset_detection();
        }

        if (target_mode_scan_step_limit_reached() || gimbal_drive_to_cdeg(
                (g_target_phase == TARGET_PHASE_SCAN_CCW) ? ccw_target : cw_target,
                0,
                BOARD_TARGET_MODE_SEARCH_TOL_CDEG,
                BOARD_TARGET_MODE_ZERO_TOL_CDEG,
                BOARD_TARGET_MODE_SCAN_STEP_MS,
                1U)) {
            if (g_target_phase == TARGET_PHASE_SCAN_CCW) {
                target_mode_enter(TARGET_PHASE_SCAN_CW);
            } else {
                g_target_after_return_phase = TARGET_PHASE_SEARCH_RETRY;
                target_mode_enter(TARGET_PHASE_RETURN_ZERO);
            }
        }
        target_mode_status_led(false, (now / 180U) & 0x01U);
        break;

    case TARGET_PHASE_AIM:
        if (valid) {
            g_target_lost_since_ms = 0;
            yaw_cmd = target_mode_limit_command(frame.yaw_cdeg);
            pitch_cmd = target_mode_limit_command(frame.pitch_cdeg);
            (void)gimbal_drive_error_cdeg(yaw_cmd, pitch_cmd,
                BOARD_TARGET_MODE_AIM_DEADBAND_CDEG,
                BOARD_TARGET_MODE_AIM_STEP_MS,
                BOARD_GIMBAL_I2C_WRITES_PER_TASK);

            if ((uint32_t)(now - g_target_phase_start_ms) < BOARD_TARGET_MODE_AIM_MIN_MS) {
                g_target_aim_stable_since_ms = 0;
            } else if (target_mode_aim_error_stable(&frame)) {
                if (g_target_aim_stable_since_ms == 0U) {
                    g_target_aim_stable_since_ms = now;
                } else if ((uint32_t)(now - g_target_aim_stable_since_ms) >=
                    BOARD_TARGET_MODE_AIM_STABLE_MS) {
                    g_target_cycle++;
                    if (g_target_cycle >= BOARD_TARGET_MODE_CYCLES) {
                        complete_run();
                        return;
                    }
                    sound_light_keypoint(g_target_cycle);
                    target_mode_enter(TARGET_PHASE_WAIT_LOST);
                    return;
                }
            } else {
                g_target_aim_stable_since_ms = 0;
            }
        } else {
            if (g_target_lost_since_ms == 0U) {
                g_target_lost_since_ms = now;
            } else if ((uint32_t)(now - g_target_lost_since_ms) >=
                BOARD_TARGET_MODE_LOST_MS) {
                g_target_after_return_phase = TARGET_PHASE_SCAN_CCW;
                target_mode_enter(TARGET_PHASE_WAIT_REPLACE);
                return;
            }
        }
        target_mode_status_led(false, true);
        break;

    case TARGET_PHASE_WAIT_LOST:
        if (valid) {
            g_target_lost_since_ms = 0;
            yaw_cmd = target_mode_limit_command(frame.yaw_cdeg);
            pitch_cmd = target_mode_limit_command(frame.pitch_cdeg);
            (void)gimbal_drive_error_cdeg(yaw_cmd, pitch_cmd,
                BOARD_TARGET_MODE_AIM_DEADBAND_CDEG,
                BOARD_TARGET_MODE_AIM_STEP_MS,
                BOARD_GIMBAL_I2C_WRITES_PER_TASK);
        } else {
            if (g_target_lost_since_ms == 0U) {
                g_target_lost_since_ms = now;
            } else if ((uint32_t)(now - g_target_lost_since_ms) >=
                BOARD_TARGET_MODE_LOST_MS) {
                g_target_after_return_phase = TARGET_PHASE_SCAN_CCW;
                target_mode_enter(TARGET_PHASE_WAIT_REPLACE);
                return;
            }
        }
        target_mode_status_led(true, true);
        break;

    case TARGET_PHASE_WAIT_REPLACE:
        if ((uint32_t)(now - g_target_phase_start_ms) >=
            BOARD_TARGET_MODE_REPLACE_WAIT_MS) {
            target_mode_enter(TARGET_PHASE_RETURN_ZERO);
            return;
        }
        target_mode_status_led((now / 250U) & 0x01U, false);
        break;

    case TARGET_PHASE_RETURN_ZERO:
        if (gimbal_drive_to_cdeg(0, 0,
                BOARD_TARGET_MODE_ZERO_TOL_CDEG,
                BOARD_TARGET_MODE_ZERO_TOL_CDEG,
                BOARD_TARGET_MODE_RETURN_STEP_MS,
                BOARD_GIMBAL_I2C_WRITES_PER_TASK)) {
            gimbal_zero_pose();
            target_mode_enter(g_target_after_return_phase);
            return;
        }
        target_mode_status_led(false, (now / 100U) & 0x01U);
        break;

    case TARGET_PHASE_SEARCH_RETRY:
        if ((uint32_t)(now - g_target_phase_start_ms) >=
            BOARD_TARGET_MODE_SEARCH_RETRY_MS) {
            target_mode_enter(TARGET_PHASE_SCAN_CCW);
        }
        target_mode_status_led(true, (now / 300U) & 0x01U);
        break;

    case TARGET_PHASE_IDLE:
    default:
        target_mode_start();
        break;
    }
}

static void task_line_gimbal_run(void)
{
    encoder_snapshot_t enc = encoder_get();
    gimbal_route_set_distance(enc.distance_mm);
    handle_keypoints(false, true);
    if (g_state == RUN_ACTIVE) {
        gimbal_task();
    }
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

static void handle_keypoints(bool stop_at_b, bool loop_forever)
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
        set_keypoint_run_led();
        return;
    }

    line_follow_status_t status = line_follow_update_ex(millis(), enc.distance_mm, true);
    if (status.fault) {
        error_stop();
        return;
    }

    if (keypoint_transition_detected(&status.gray, millis(), enc.distance_mm)) {
        uint8_t point_index = (uint8_t)(g_next_keypoint + 1U);
        sound_light_keypoint(point_index);
        if (stop_at_b && g_next_keypoint == 0U) {
            g_pause_until_ms = millis() + BOARD_KEYPOINT_B_STOP_MS;
            motor_stop();
        }
        g_next_keypoint++;
        if (g_next_keypoint >= 4U) {
            if (loop_forever) {
                g_next_keypoint = 0U;
            } else {
                finish_run();
            }
            return;
        }
    }

    set_keypoint_run_led();
}

static void task_gimbal_test(void)
{
    k230_frame_t frame;
    bool has_frame = k230_uart_get_latest(&frame);
    bool fresh = has_frame &&
        ((uint32_t)(millis() - frame.received_ms) <= BOARD_K230_VISION_TIMEOUT_MS);
    bool valid = fresh && frame.valid;

    if (valid) {
        ui_green(true);
        ui_red(false);
    } else if (has_frame) {
        ui_green(false);
        ui_red((millis() / 120U) & 0x01U);
    } else if (frame.rx_byte_count != 0U) {
        ui_green((millis() / 120U) & 0x01U);
        ui_red(true);
    } else {
        ui_green(false);
        ui_red(true);
    }
}

static void task_gimbal_sweep_test(void)
{
    motor_stop();
    gimbal_sweep_test_task();
    ui_green((millis() / 250U) & 0x01U);
    ui_red(false);
}

static void active_task(void)
{
    switch (g_mode) {
    case MODE_ONE_LAP_RUN:
        task_one_lap_run();
        break;
    case MODE_TARGET_POINT_RUN:
        task_target_point_run();
        break;
    case MODE_LINE_GIMBAL_RUN:
        task_line_gimbal_run();
        break;
    case MODE_LINE_FOLLOW:
        task_line_follow_only();
        break;
    case MODE_KEYPOINT_RUN:
        handle_keypoints(false, true);
        break;
    case MODE_GIMBAL_TEST:
        task_gimbal_test();
        break;
    case MODE_GIMBAL_SWEEP_TEST:
        task_gimbal_sweep_test();
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
    sound_light_init();
    motor_init();
    grayscale_init();
    encoder_init();
    imu_init();
    k230_uart_init();
    gimbal_init();

    ui_signal_mode((uint8_t)g_mode);

    while (1) {
        ui_task();
        sound_light_task();
        k230_uart_task();
        if (g_state == RUN_ACTIVE && g_mode == MODE_GIMBAL_TEST) {
            gimbal_task();
        }

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
        } else if (!sound_light_active()) {
            blink_idle();
        }
    }
}
