#include "tasks.h"
#include "board_pins.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "k230.h"
#include "line_controller.h"
#include "motor.h"
#include "pca9685.h"
#include "ui.h"

typedef enum {
    PHASE_NONE = 0,
    PHASE_TRACE,
    PHASE_PAUSE,
    PHASE_AIM,
    PHASE_DRAW,
    PHASE_FINAL_AIM
} task_phase_t;

typedef enum {
    SELFTEST_PHASE_STARTUP = 0,
    SELFTEST_PHASE_LEFT_RUN,
    SELFTEST_PHASE_LEFT_PAUSE,
    SELFTEST_PHASE_RIGHT_RUN,
    SELFTEST_PHASE_RIGHT_PAUSE
} selftest_phase_t;

static app_task_t g_task = APP_TASK_NONE;
static task_status_t g_status = TASK_STATUS_IDLE;
static const char *g_state = "IDLE";
static task_phase_t g_phase = PHASE_NONE;
static uint32_t g_task_start_ms;
static uint32_t g_phase_start_ms;
static uint8_t g_keypoint_index;
static grayscale_sample_t g_last_gray;
static int16_t g_last_yaw_x10;
static bool g_imu_fresh;
static bool g_k230_fresh;

static uint32_t g_lost_line_ms;
static int16_t g_hold_yaw_x10;
static bool g_hold_yaw_valid;
static line_mode_t g_feedback_mode = LINE_MODE_STOP;
static uint32_t g_feedback_mode_ms;
static uint32_t g_feedback_beep_ms;

static uint8_t g_pan_deg = BOARD_SERVO_CENTER_DEG;
static uint8_t g_tilt_deg = BOARD_SERVO_CENTER_DEG;
static uint32_t g_aim_lock_ms;
static uint32_t g_laser_until_ms;
static bool g_aim_action_done;

static selftest_phase_t g_selftest_phase;
static uint32_t g_selftest_phase_ms;
static int32_t g_selftest_lr_start_count;
static int32_t g_selftest_rr_start_count;
static bool g_selftest_imu_seen;
static bool g_selftest_gray_seen;

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t clamp_servo_deg(int32_t value)
{
    if (value < BOARD_SERVO_MIN_DEG) {
        return BOARD_SERVO_MIN_DEG;
    }
    if (value > BOARD_SERVO_MAX_DEG) {
        return BOARD_SERVO_MAX_DEG;
    }
    return (uint8_t) value;
}

static void motor_safe_idle(void)
{
    motor_stop();
    motor_disable();
}

static void reset_line_feedback(void)
{
    g_feedback_mode = LINE_MODE_STOP;
    g_feedback_mode_ms = 0U;
    g_feedback_beep_ms = 0U;
}

static void reset_line_runner(uint32_t now_ms, bool reset_encoder)
{
    if (reset_encoder) {
        encoder_reset();
    }
    line_controller_reset();
    g_lost_line_ms = 0U;
    g_hold_yaw_valid = imu_get_yaw_x10(now_ms, &g_hold_yaw_x10);
    reset_line_feedback();
    motor_enable();
}

static void finish_task(void)
{
    motor_safe_idle();
    laser_set(false);
    reset_line_feedback();
    ui_set_running(false);
    ui_finish_signal();
    g_state = "DONE";
    g_status = TASK_STATUS_FINISHED;
    g_phase = PHASE_NONE;
}

static void error_task(const char *state)
{
    motor_safe_idle();
    laser_set(false);
    reset_line_feedback();
    ui_set_running(false);
    ui_set_error(true);
    ui_beep_ms(1000U);
    g_state = state;
    g_status = TASK_STATUS_ERROR;
    g_phase = PHASE_NONE;
}

static bool task_timed_out(uint32_t now_ms, uint32_t timeout_ms)
{
    return ((now_ms - g_task_start_ms) >= timeout_ms);
}

static bool keypoint_reached(void)
{
    if (g_keypoint_index >= 4U) {
        return false;
    }

    int32_t target = BOARD_KEYPOINT_DISTANCE_MM[g_keypoint_index];
    if (target <= 0) {
        return false;
    }

    int32_t distance = encoder_get_average_distance_mm();
    return distance >= (target - BOARD_KEYPOINT_TOLERANCE_MM);
}

static bool line_control_update(uint32_t now_ms)
{
    line_output_t output;
    int16_t yaw_error_x10 = 0;

    g_last_gray = grayscale_read();
    g_imu_fresh = imu_get_yaw_x10(now_ms, &g_last_yaw_x10);

    if (g_last_gray.line_seen) {
        g_lost_line_ms = 0U;
        if (g_imu_fresh) {
            g_hold_yaw_x10 = g_last_yaw_x10;
            g_hold_yaw_valid = true;
        }
        output = line_controller_gray(&g_last_gray);
    } else {
        g_lost_line_ms += BOARD_LOOP_PERIOD_MS;
        if (g_hold_yaw_valid && g_imu_fresh &&
            (g_lost_line_ms <= BOARD_LOST_HEADING_HOLD_MS)) {
            yaw_error_x10 = imu_wrap_error_x10(g_hold_yaw_x10, g_last_yaw_x10);
            output = line_controller_heading_hold(yaw_error_x10);
        } else if (g_lost_line_ms <= BOARD_LOST_STOP_MS) {
            output = line_controller_search(line_controller_last_error());
        } else {
            error_task("LINE LOST");
            return false;
        }
    }

    if (output.mode != g_feedback_mode) {
        g_feedback_mode = output.mode;
        g_feedback_mode_ms = 0U;
        g_feedback_beep_ms = 0U;

        if (output.mode == LINE_MODE_IMU_HOLD) {
            ui_beep_ms(BOARD_IMU_TAKEOVER_BEEP_MS);
        } else if (output.mode == LINE_MODE_SEARCH) {
            ui_beep_ms(BOARD_SEARCH_ENTER_BEEP_MS);
        }
    }

    switch (output.mode) {
    case LINE_MODE_GRAY:
        ui_set_status_leds(false, true);
        break;
    case LINE_MODE_IMU_HOLD:
        ui_set_status_leds(
            ((g_feedback_mode_ms / BOARD_IMU_FEEDBACK_RED_BLINK_MS) & 0x01U) == 0U,
            true);
        if ((abs32(yaw_error_x10) >= BOARD_IMU_FEEDBACK_WARN_X10) &&
            (g_feedback_beep_ms >= BOARD_IMU_FEEDBACK_BEEP_INTERVAL_MS)) {
            ui_beep_ms(BOARD_IMU_FEEDBACK_BEEP_MS);
            g_feedback_beep_ms = 0U;
        }
        break;
    case LINE_MODE_SEARCH:
        ui_set_status_leds(
            ((g_feedback_mode_ms / BOARD_SEARCH_RED_BLINK_MS) & 0x01U) == 0U,
            false);
        if (g_feedback_beep_ms >= BOARD_SEARCH_BEEP_INTERVAL_MS) {
            ui_beep_ms(BOARD_SEARCH_BEEP_MS);
            g_feedback_beep_ms = 0U;
        }
        break;
    case LINE_MODE_STOP:
    default:
        ui_set_status_leds(false, false);
        break;
    }

    motor_set_percent(output.left_percent, output.right_percent);
    g_feedback_mode_ms += BOARD_LOOP_PERIOD_MS;
    g_feedback_beep_ms += BOARD_LOOP_PERIOD_MS;
    return true;
}

static void aim_reset(void)
{
    g_pan_deg = BOARD_SERVO_CENTER_DEG;
    g_tilt_deg = BOARD_SERVO_CENTER_DEG;
    g_aim_lock_ms = 0U;
    g_laser_until_ms = 0U;
    g_aim_action_done = false;
    laser_set(false);
    (void) pca9685_set_servo_angle(BOARD_SERVO_PAN_CHANNEL, g_pan_deg);
    (void) pca9685_set_servo_angle(BOARD_SERVO_TILT_CHANNEL, g_tilt_deg);
}

static void aim_start(uint32_t now_ms)
{
    (void) now_ms;
    motor_stop();
    aim_reset();
    g_phase = PHASE_AIM;
    g_phase_start_ms = now_ms;
    g_state = "AIM";
}

static bool aim_service_laser(uint32_t now_ms, bool finish_when_done)
{
    if ((g_laser_until_ms != 0U) && (now_ms >= g_laser_until_ms)) {
        laser_set(false);
        g_laser_until_ms = 0U;
        return finish_when_done && g_aim_action_done;
    }
    return false;
}

static bool aim_follow_target(uint32_t now_ms)
{
    k230_target_t target;
    g_k230_fresh = k230_get_target(now_ms, &target);

    if (g_k230_fresh && (target.quality >= BOARD_AIM_TARGET_QUALITY_MIN)) {
        int32_t x = target.x;
        int32_t y = target.y;

        if (abs32(x) > BOARD_AIM_LOCK_ERROR) {
            int32_t step = x / BOARD_AIM_SERVO_STEP_DIV;
            if (step == 0) {
                step = (x > 0) ? 1 : -1;
            }
            g_pan_deg = clamp_servo_deg((int32_t) g_pan_deg + step);
        }

        if (abs32(y) > BOARD_AIM_LOCK_ERROR) {
            int32_t step = y / BOARD_AIM_SERVO_STEP_DIV;
            if (step == 0) {
                step = (y > 0) ? 1 : -1;
            }
            g_tilt_deg = clamp_servo_deg((int32_t) g_tilt_deg + step);
        }

        (void) pca9685_set_servo_angle(BOARD_SERVO_PAN_CHANNEL, g_pan_deg);
        (void) pca9685_set_servo_angle(BOARD_SERVO_TILT_CHANNEL, g_tilt_deg);

        if ((abs32(x) <= BOARD_AIM_LOCK_ERROR) &&
            (abs32(y) <= BOARD_AIM_LOCK_ERROR)) {
            g_aim_lock_ms += BOARD_LOOP_PERIOD_MS;
        } else {
            g_aim_lock_ms = 0U;
        }

        return (g_aim_lock_ms >= BOARD_AIM_LOCK_MS);
    } else {
        g_aim_lock_ms = 0U;
        ui_set_status_leds(true, false);
    }

    return false;
}

static void aim_fire(uint32_t now_ms, bool latch_action)
{
    laser_set(true);
    g_laser_until_ms = now_ms + BOARD_AIM_LASER_PULSE_MS;
    if (latch_action) {
        g_aim_action_done = true;
    }
    g_aim_lock_ms = 0U;
    ui_beep_ms(160U);
}

static bool aim_update(uint32_t now_ms)
{
    if (aim_service_laser(now_ms, true)) {
        return true;
    }
    if (g_aim_action_done) {
        return false;
    }
    if (aim_follow_target(now_ms) && (g_laser_until_ms == 0U)) {
        aim_fire(now_ms, true);
    }
    return false;
}

static void aim_track_update(uint32_t now_ms, bool latch_action)
{
    (void) aim_service_laser(now_ms, false);
    if (aim_follow_target(now_ms) && (g_laser_until_ms == 0U) &&
        (!latch_action || !g_aim_action_done)) {
        aim_fire(now_ms, latch_action);
    }
}

static void selftest_begin_phase(selftest_phase_t phase)
{
    g_selftest_phase = phase;
    g_selftest_phase_ms = 0U;

    if ((phase == SELFTEST_PHASE_LEFT_RUN) ||
        (phase == SELFTEST_PHASE_RIGHT_RUN)) {
        encoder_get_counts(&g_selftest_lr_start_count, &g_selftest_rr_start_count);
        motor_enable();
    }

    switch (phase) {
    case SELFTEST_PHASE_LEFT_RUN:
        motor_set_percent(BOARD_SELFTEST_MOTOR_PERCENT, 0);
        break;
    case SELFTEST_PHASE_RIGHT_RUN:
        motor_set_percent(0, BOARD_SELFTEST_MOTOR_PERCENT);
        break;
    default:
        motor_stop();
        break;
    }
}

static void selftest_signal_result(bool pass, bool right_side)
{
    if (pass) {
        ui_beep_ms(right_side ? 150U : 80U);
    } else {
        ui_beep_ms(320U);
    }
}

static void selftest_update(uint32_t now_ms)
{
    int32_t lr_count;
    int32_t rr_count;
    int32_t lr_delta = 0;
    int32_t rr_delta = 0;

    g_last_gray = grayscale_read();
    g_imu_fresh = imu_get_yaw_x10(now_ms, &g_last_yaw_x10);

    if (g_last_gray.line_seen) {
        g_selftest_gray_seen = true;
    }
    if (g_imu_fresh) {
        g_selftest_imu_seen = true;
    }

    encoder_get_counts(&lr_count, &rr_count);
    ui_set_status_leds(g_imu_fresh, g_last_gray.line_seen);

    switch (g_selftest_phase) {
    case SELFTEST_PHASE_STARTUP:
        motor_stop();
        if (g_selftest_phase_ms >= BOARD_SELFTEST_MOTOR_PERIOD_MS) {
            selftest_begin_phase(SELFTEST_PHASE_LEFT_RUN);
        }
        break;
    case SELFTEST_PHASE_LEFT_RUN:
        lr_delta = abs32(lr_count - g_selftest_lr_start_count);
        if (g_selftest_phase_ms >= BOARD_SELFTEST_MOTOR_ON_MS) {
            selftest_signal_result(
                (lr_delta >= BOARD_SELFTEST_ENCODER_DELTA_COUNTS), false);
            selftest_begin_phase(SELFTEST_PHASE_LEFT_PAUSE);
        }
        break;
    case SELFTEST_PHASE_LEFT_PAUSE:
        motor_stop();
        if (g_selftest_phase_ms >=
            (BOARD_SELFTEST_MOTOR_PERIOD_MS - BOARD_SELFTEST_MOTOR_ON_MS)) {
            selftest_begin_phase(SELFTEST_PHASE_RIGHT_RUN);
        }
        break;
    case SELFTEST_PHASE_RIGHT_RUN:
        rr_delta = abs32(rr_count - g_selftest_rr_start_count);
        if (g_selftest_phase_ms >= BOARD_SELFTEST_MOTOR_ON_MS) {
            selftest_signal_result(
                (rr_delta >= BOARD_SELFTEST_ENCODER_DELTA_COUNTS), true);
            selftest_begin_phase(SELFTEST_PHASE_RIGHT_PAUSE);
        }
        break;
    case SELFTEST_PHASE_RIGHT_PAUSE:
        motor_stop();
        if (g_selftest_phase_ms >=
            (BOARD_SELFTEST_MOTOR_PERIOD_MS - BOARD_SELFTEST_MOTOR_ON_MS)) {
            selftest_begin_phase(SELFTEST_PHASE_LEFT_RUN);
        }
        break;
    default:
        error_task("SELF ERR");
        return;
    }

    g_selftest_phase_ms += BOARD_LOOP_PERIOD_MS;
}

static void run_auto_trace(uint32_t now_ms)
{
    if (task_timed_out(now_ms, BOARD_AUTO_TRACE_TIMEOUT_MS)) {
        error_task("TIMEOUT");
        return;
    }

    if (!line_control_update(now_ms)) {
        return;
    }

    if (encoder_get_average_distance_mm() >= BOARD_LAP_DISTANCE_MM) {
        finish_task();
    }
}

static void run_trace_aim(uint32_t now_ms)
{
    if (task_timed_out(now_ms, BOARD_TRACE_AIM_TIMEOUT_MS)) {
        error_task("TIMEOUT");
        return;
    }

    if (g_phase == PHASE_AIM) {
        if (aim_update(now_ms)) {
            g_phase = PHASE_TRACE;
            g_phase_start_ms = now_ms;
            g_state = "TRACE";
            reset_line_runner(now_ms, false);
        } else if ((now_ms - g_phase_start_ms) >= BOARD_AIM_TIMEOUT_MS) {
            error_task("AIM LOST");
        }
        return;
    }

    if (g_phase == PHASE_PAUSE) {
        motor_stop();
        if ((now_ms - g_phase_start_ms) >= BOARD_KEYPOINT_PAUSE_MS) {
            g_phase = PHASE_TRACE;
            g_phase_start_ms = now_ms;
            g_state = "TRACE";
            reset_line_runner(now_ms, false);
        }
        return;
    }

    if (!line_control_update(now_ms)) {
        return;
    }

    if (!keypoint_reached()) {
        return;
    }

    uint8_t reached = g_keypoint_index;
    motor_stop();
    ui_keypoint_signal(reached);

    if (reached == 0U) {
        g_keypoint_index = 1U;
        aim_start(now_ms);
    } else if (reached >= 3U) {
        finish_task();
    } else {
        g_keypoint_index++;
        g_phase = PHASE_PAUSE;
        g_phase_start_ms = now_ms;
        g_state = "POINT";
    }
}

static void run_static_aim(uint32_t now_ms)
{
    if (aim_update(now_ms)) {
        finish_task();
    } else if (task_timed_out(now_ms, BOARD_AIM_TIMEOUT_MS)) {
        error_task("AIM LOST");
    }
}

static void run_dynamic_aim(uint32_t now_ms)
{
    if (task_timed_out(now_ms, BOARD_DYNAMIC_AIM_TIMEOUT_MS)) {
        error_task("TIMEOUT");
        return;
    }

    aim_track_update(now_ms, false);
    if (!line_control_update(now_ms)) {
        return;
    }

    if (encoder_get_average_distance_mm() >= BOARD_LAP_DISTANCE_MM) {
        finish_task();
    }
}

static void run_sync_draw(uint32_t now_ms)
{
    static const uint8_t pan_seq[] = {70, 110, 110, 70, 70, 90, 78, 102, 90};
    static const uint8_t tilt_seq[] = {70, 70, 110, 110, 70, 90, 105, 105, 78};
    uint32_t elapsed = now_ms - g_phase_start_ms;
    uint8_t step = (uint8_t) (elapsed / BOARD_DRAW_STEP_MS);
    uint8_t count = (uint8_t) (sizeof(pan_seq) / sizeof(pan_seq[0]));

    motor_safe_idle();
    if ((step >= count) || task_timed_out(now_ms, BOARD_SYNC_DRAW_TIMEOUT_MS)) {
        laser_set(false);
        finish_task();
        return;
    }

    laser_set((step > 0U) && (step < (count - 1U)));
    (void) pca9685_set_servo_angle(BOARD_SERVO_PAN_CHANNEL, pan_seq[step]);
    (void) pca9685_set_servo_angle(BOARD_SERVO_TILT_CHANNEL, tilt_seq[step]);
    ui_set_status_leds(false, true);
}

static void run_four_laps(uint32_t now_ms)
{
    int32_t target_distance = BOARD_LAP_DISTANCE_MM * 4;

    if (task_timed_out(now_ms, BOARD_FOUR_LAPS_TIMEOUT_MS)) {
        error_task("TIMEOUT");
        return;
    }

    if (g_phase == PHASE_FINAL_AIM) {
        if (aim_update(now_ms)) {
            finish_task();
        } else if ((now_ms - g_phase_start_ms) >= BOARD_AIM_TIMEOUT_MS) {
            error_task("AIM LOST");
        }
        return;
    }

    aim_track_update(now_ms, true);
    if (!line_control_update(now_ms)) {
        return;
    }

    if (encoder_get_average_distance_mm() >= target_distance) {
        if (g_aim_action_done) {
            finish_task();
        } else {
            aim_start(now_ms);
            g_phase = PHASE_FINAL_AIM;
            g_state = "FINAL AIM";
        }
    }
}

static void run_sensor_dash(uint32_t now_ms)
{
    motor_safe_idle();
    g_last_gray = grayscale_read();
    g_imu_fresh = imu_get_yaw_x10(now_ms, &g_last_yaw_x10);
    g_k230_fresh = k230_get_target(now_ms, 0);
    ui_set_status_leds(!g_imu_fresh, g_last_gray.line_seen);
}

static void run_straight_test(uint32_t now_ms)
{
    int32_t lr_count;
    int32_t rr_count;
    int32_t diff;
    int16_t yaw_error = 0;
    int32_t correction;

    if (task_timed_out(now_ms, 10000U)) {
        error_task("TIMEOUT");
        return;
    }

    if (encoder_get_average_distance_mm() >= BOARD_STRAIGHT_TEST_DISTANCE_MM) {
        finish_task();
        return;
    }

    encoder_get_counts(&lr_count, &rr_count);
    diff = lr_count - rr_count;
    g_imu_fresh = imu_get_yaw_x10(now_ms, &g_last_yaw_x10);
    if (g_hold_yaw_valid && g_imu_fresh) {
        yaw_error = imu_wrap_error_x10(g_hold_yaw_x10, g_last_yaw_x10);
    }

    correction = ((diff * BOARD_STRAIGHT_ENCODER_KP) / 10) +
                 ((int32_t) yaw_error * BOARD_STRAIGHT_YAW_KP) / 100;
    motor_enable();
    motor_set_percent(BOARD_STRAIGHT_TEST_BASE_PERCENT + correction,
        BOARD_STRAIGHT_TEST_BASE_PERCENT - correction);
    ui_set_status_leds(false, true);
}

void tasks_init(void)
{
    tasks_stop();
}

void tasks_start(app_task_t task, uint32_t now_ms)
{
    tasks_stop();

    g_task = task;
    g_status = TASK_STATUS_RUNNING;
    g_task_start_ms = now_ms;
    g_phase_start_ms = now_ms;
    g_keypoint_index = 0U;
    g_state = tasks_name(task);
    g_phase = PHASE_NONE;
    g_k230_fresh = false;
    g_imu_fresh = false;

    ui_set_error(false);
    ui_set_running(true);
    ui_beep_ms(120U);
    aim_reset();

    switch (task) {
    case APP_TASK_AUTO_TRACE:
        g_phase = PHASE_TRACE;
        g_state = "TRACE";
        reset_line_runner(now_ms, true);
        break;
    case APP_TASK_STATIC_AIM:
        aim_start(now_ms);
        break;
    case APP_TASK_TRACE_AIM:
        g_phase = PHASE_TRACE;
        g_state = "TRACE";
        reset_line_runner(now_ms, true);
        break;
    case APP_TASK_DYNAMIC_AIM:
        g_phase = PHASE_TRACE;
        g_state = "DYN AIM";
        reset_line_runner(now_ms, true);
        break;
    case APP_TASK_SYNC_DRAW:
        g_phase = PHASE_DRAW;
        g_state = "DRAW";
        g_phase_start_ms = now_ms;
        motor_safe_idle();
        break;
    case APP_TASK_FOUR_LAPS:
        g_phase = PHASE_TRACE;
        g_state = "4 LAPS";
        reset_line_runner(now_ms, true);
        break;
    case APP_TASK_SELFTEST:
        g_phase = PHASE_NONE;
        g_state = "SELFTEST";
        encoder_reset();
        line_controller_reset();
        g_selftest_imu_seen = false;
        g_selftest_gray_seen = false;
        selftest_begin_phase(SELFTEST_PHASE_STARTUP);
        break;
    case APP_TASK_SENSOR_DASH:
        g_phase = PHASE_NONE;
        g_state = "SENSORS";
        motor_safe_idle();
        break;
    case APP_TASK_STRAIGHT_TEST:
        g_phase = PHASE_TRACE;
        g_state = "STRAIGHT";
        reset_line_runner(now_ms, true);
        break;
    case APP_TASK_NONE:
    default:
        tasks_stop();
        break;
    }
}

void tasks_stop(void)
{
    motor_safe_idle();
    laser_set(false);
    reset_line_feedback();
    ui_set_running(false);
    g_task = APP_TASK_NONE;
    g_status = TASK_STATUS_IDLE;
    g_state = "IDLE";
    g_phase = PHASE_NONE;
}

void tasks_update(uint32_t now_ms)
{
    if (g_status != TASK_STATUS_RUNNING) {
        return;
    }

    switch (g_task) {
    case APP_TASK_AUTO_TRACE:
        run_auto_trace(now_ms);
        break;
    case APP_TASK_STATIC_AIM:
        run_static_aim(now_ms);
        break;
    case APP_TASK_TRACE_AIM:
        run_trace_aim(now_ms);
        break;
    case APP_TASK_DYNAMIC_AIM:
        run_dynamic_aim(now_ms);
        break;
    case APP_TASK_SYNC_DRAW:
        run_sync_draw(now_ms);
        break;
    case APP_TASK_FOUR_LAPS:
        run_four_laps(now_ms);
        break;
    case APP_TASK_SELFTEST:
        selftest_update(now_ms);
        break;
    case APP_TASK_SENSOR_DASH:
        run_sensor_dash(now_ms);
        break;
    case APP_TASK_STRAIGHT_TEST:
        run_straight_test(now_ms);
        break;
    case APP_TASK_NONE:
    default:
        error_task("BAD TASK");
        break;
    }
}

bool tasks_running(void)
{
    return (g_status == TASK_STATUS_RUNNING);
}

bool tasks_done(void)
{
    return (g_status == TASK_STATUS_FINISHED) || (g_status == TASK_STATUS_ERROR);
}

app_task_t tasks_current(void)
{
    return g_task;
}

const char *tasks_name(app_task_t task)
{
    switch (task) {
    case APP_TASK_AUTO_TRACE:
        return "AUTO TRACE";
    case APP_TASK_STATIC_AIM:
        return "STATIC AIM";
    case APP_TASK_TRACE_AIM:
        return "TRACE AIM";
    case APP_TASK_DYNAMIC_AIM:
        return "DYN AIM";
    case APP_TASK_SYNC_DRAW:
        return "DRAW";
    case APP_TASK_FOUR_LAPS:
        return "4 LAPS";
    case APP_TASK_SELFTEST:
        return "SELFTEST";
    case APP_TASK_SENSOR_DASH:
        return "SENSORS";
    case APP_TASK_STRAIGHT_TEST:
        return "STRAIGHT";
    case APP_TASK_NONE:
    default:
        return "IDLE";
    }
}

app_task_t tasks_from_slot(uint8_t slot)
{
    switch (slot) {
    case 1U:
        return APP_TASK_AUTO_TRACE;
    case 2U:
        return APP_TASK_STATIC_AIM;
    case 3U:
        return APP_TASK_TRACE_AIM;
    case 4U:
        return APP_TASK_DYNAMIC_AIM;
    case 5U:
        return APP_TASK_SYNC_DRAW;
    case 6U:
        return APP_TASK_FOUR_LAPS;
    case 7U:
        return APP_TASK_SELFTEST;
    case 8U:
        return APP_TASK_SENSOR_DASH;
    case 9U:
        return APP_TASK_STRAIGHT_TEST;
    default:
        return APP_TASK_NONE;
    }
}

void tasks_snapshot(uint32_t now_ms, task_snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    if (g_status != TASK_STATUS_RUNNING) {
        g_last_gray = grayscale_read();
        g_imu_fresh = imu_get_yaw_x10(now_ms, &g_last_yaw_x10);
        g_k230_fresh = k230_get_target(now_ms, 0);
    }

    snapshot->task = g_task;
    snapshot->status = g_status;
    snapshot->state = g_state;
    snapshot->phase = (uint8_t) g_phase;
    snapshot->keypoint = g_keypoint_index;
    snapshot->gray = g_last_gray;
    snapshot->yaw_x10 = g_last_yaw_x10;
    snapshot->distance_mm = encoder_get_average_distance_mm();
    snapshot->imu_fresh = g_imu_fresh;
    snapshot->k230_fresh = g_k230_fresh;
}
