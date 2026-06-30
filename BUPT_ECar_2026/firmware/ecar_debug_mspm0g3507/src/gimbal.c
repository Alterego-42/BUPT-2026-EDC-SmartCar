#include "gimbal.h"

#include <stdint.h>

#include "board_config.h"
#include "gimbal_route.h"
#include "k230_uart.h"
#include "laser.h"
#include "pca9685.h"
#include "timebase.h"
#include "util.h"

#define GIMBAL_YAW_BASE_CH      (0U)
#define GIMBAL_PITCH_BASE_CH    (4U)
#define GIMBAL_SWEEP_STEP_MS    (8U)

typedef struct {
    uint8_t base_ch;
    uint8_t step_index;
    bool energized;
    int8_t direction_sign;
    int16_t step_cdeg;
    int16_t position_cdeg;
    int32_t step_count;
    uint16_t deadband_cdeg;
    uint16_t max_error_cdeg;
    uint16_t min_step_ms;
    uint16_t max_step_ms;
    uint32_t next_step_ms;
} gimbal_axis_t;

static const uint8_t k_step_seq[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

static gimbal_axis_t g_yaw;
static gimbal_axis_t g_pitch;
static bool g_active;
static bool g_pca_ready;
static uint32_t g_next_pca_retry_ms;

static uint16_t abs_i16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static int8_t sign_i16(int16_t value)
{
    return (value >= 0) ? 1 : -1;
}

static void axis_config(gimbal_axis_t *axis, uint8_t base_ch, int8_t direction_sign,
    int16_t step_cdeg, uint16_t deadband_cdeg, uint16_t max_error_cdeg,
    uint16_t min_step_ms, uint16_t max_step_ms)
{
    axis->base_ch = base_ch;
    axis->step_index = 0;
    axis->energized = false;
    axis->direction_sign = direction_sign;
    axis->step_cdeg = step_cdeg;
    axis->position_cdeg = 0;
    axis->step_count = 0;
    axis->deadband_cdeg = deadband_cdeg;
    axis->max_error_cdeg = max_error_cdeg;
    axis->min_step_ms = min_step_ms;
    axis->max_step_ms = max_step_ms;
    axis->next_step_ms = 0;
}

static uint16_t axis_compute_interval(const gimbal_axis_t *axis, int16_t error_cdeg)
{
    uint16_t mag = abs_i16(error_cdeg);
    uint32_t span_ms;
    uint32_t scaled_ms;

    if (mag >= axis->max_error_cdeg) {
        return axis->min_step_ms;
    }
    if (mag <= axis->deadband_cdeg ||
        axis->max_error_cdeg <= axis->deadband_cdeg) {
        return axis->max_step_ms;
    }

    span_ms = (uint32_t)(axis->max_step_ms - axis->min_step_ms);
    scaled_ms = ((uint32_t)(mag - axis->deadband_cdeg) * span_ms) /
        (uint32_t)(axis->max_error_cdeg - axis->deadband_cdeg);
    return (uint16_t)((uint32_t)axis->max_step_ms - scaled_ms);
}

static bool axis_update(gimbal_axis_t *axis, int16_t error_cdeg,
    bool tracking_active, uint32_t now_ms)
{
    uint16_t mag;
    uint16_t interval;
    int8_t step_dir;

    mag = abs_i16(error_cdeg);
    if (!tracking_active || mag <= axis->deadband_cdeg) {
#if BOARD_GIMBAL_RELEASE_COILS_IDLE
        axis_release(axis);
#endif
        axis->next_step_ms = now_ms;
        return false;
    }

    if ((int32_t)(now_ms - axis->next_step_ms) < 0) {
        return false;
    }

    interval = axis_compute_interval(axis, error_cdeg);
    step_dir = (int8_t)(-sign_i16(error_cdeg) * axis->direction_sign);
    if (step_dir > 0) {
        axis->step_index = (uint8_t)((axis->step_index + 1U) & 0x07U);
    } else {
        axis->step_index = (uint8_t)((axis->step_index + 7U) & 0x07U);
    }

    if (pca9685_set_motor4(axis->base_ch, k_step_seq[axis->step_index])) {
        axis->energized = true;
        axis->position_cdeg = clamp_i16(
            (int32_t)axis->position_cdeg +
                (int32_t)sign_i16(error_cdeg) * axis->step_cdeg,
            -BOARD_GIMBAL_POSITION_LIMIT_CDEG,
            BOARD_GIMBAL_POSITION_LIMIT_CDEG);
        axis->step_count += sign_i16(error_cdeg);
    } else {
        g_pca_ready = false;
        g_next_pca_retry_ms = now_ms + BOARD_GIMBAL_PCA_RETRY_MS;
        axis->next_step_ms = now_ms + interval;
        return true;
    }

    axis->next_step_ms = now_ms + interval;
    return true;
}

static bool axis_update_fixed(gimbal_axis_t *axis, int16_t error_cdeg,
    uint16_t deadband_cdeg, uint16_t step_interval_ms, uint32_t now_ms)
{
    uint16_t mag;
    int8_t step_dir;

    mag = abs_i16(error_cdeg);
    if (mag <= deadband_cdeg) {
        axis->next_step_ms = now_ms;
        return false;
    }

    if ((int32_t)(now_ms - axis->next_step_ms) < 0) {
        return false;
    }

    step_dir = (int8_t)(-sign_i16(error_cdeg) * axis->direction_sign);
    if (step_dir > 0) {
        axis->step_index = (uint8_t)((axis->step_index + 1U) & 0x07U);
    } else {
        axis->step_index = (uint8_t)((axis->step_index + 7U) & 0x07U);
    }

    if (pca9685_set_motor4(axis->base_ch, k_step_seq[axis->step_index])) {
        axis->energized = true;
        axis->position_cdeg = clamp_i16(
            (int32_t)axis->position_cdeg +
                (int32_t)sign_i16(error_cdeg) * axis->step_cdeg,
            -BOARD_GIMBAL_POSITION_LIMIT_CDEG,
            BOARD_GIMBAL_POSITION_LIMIT_CDEG);
        axis->step_count += sign_i16(error_cdeg);
    } else {
        g_pca_ready = false;
        g_next_pca_retry_ms = now_ms + BOARD_GIMBAL_PCA_RETRY_MS;
        axis->next_step_ms = now_ms + step_interval_ms;
        return true;
    }

    axis->next_step_ms = now_ms + step_interval_ms;
    return true;
}

static bool gimbal_prepare_drive(uint32_t now_ms)
{
    if (!g_active) {
        return false;
    }

    if (!g_pca_ready) {
        if ((int32_t)(now_ms - g_next_pca_retry_ms) >= 0) {
            g_pca_ready = pca9685_init();
            g_next_pca_retry_ms = now_ms + BOARD_GIMBAL_PCA_RETRY_MS;
        }
        return false;
    }

    return true;
}

#if BOARD_GIMBAL_RELEASE_COILS_IDLE
static void axis_release(gimbal_axis_t *axis)
{
    static const uint8_t off_pattern[4] = {0, 0, 0, 0};

    if (axis->energized && pca9685_is_ready()) {
        (void)pca9685_set_motor4(axis->base_ch, off_pattern);
        axis->energized = false;
    }
}
#endif

void gimbal_init(void)
{
    g_active = false;
    g_pca_ready = false;
    g_next_pca_retry_ms = 0;
    laser_init();
    axis_config(&g_yaw, GIMBAL_YAW_BASE_CH, BOARD_GIMBAL_YAW_DIR,
        BOARD_GIMBAL_YAW_STEP_CDEG, BOARD_GIMBAL_YAW_DEADBAND_CDEG,
        BOARD_GIMBAL_YAW_MAX_ERROR_CDEG, BOARD_GIMBAL_YAW_STEP_MIN_MS,
        BOARD_GIMBAL_YAW_STEP_MAX_MS);
    axis_config(&g_pitch, GIMBAL_PITCH_BASE_CH, BOARD_GIMBAL_PITCH_DIR,
        BOARD_GIMBAL_PITCH_STEP_CDEG, BOARD_GIMBAL_PITCH_DEADBAND_CDEG,
        BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG, BOARD_GIMBAL_PITCH_STEP_MIN_MS,
        BOARD_GIMBAL_PITCH_STEP_MAX_MS);
}

void gimbal_start(void)
{
    uint32_t now = millis();

    axis_config(&g_yaw, GIMBAL_YAW_BASE_CH, BOARD_GIMBAL_YAW_DIR,
        BOARD_GIMBAL_YAW_STEP_CDEG, BOARD_GIMBAL_YAW_DEADBAND_CDEG,
        BOARD_GIMBAL_YAW_MAX_ERROR_CDEG, BOARD_GIMBAL_YAW_STEP_MIN_MS,
        BOARD_GIMBAL_YAW_STEP_MAX_MS);
    axis_config(&g_pitch, GIMBAL_PITCH_BASE_CH, BOARD_GIMBAL_PITCH_DIR,
        BOARD_GIMBAL_PITCH_STEP_CDEG, BOARD_GIMBAL_PITCH_DEADBAND_CDEG,
        BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG, BOARD_GIMBAL_PITCH_STEP_MIN_MS,
        BOARD_GIMBAL_PITCH_STEP_MAX_MS);

    g_active = true;
    laser_set(true);
    g_pca_ready = pca9685_init();
    g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
}

void gimbal_stop(void)
{
    g_active = false;
    laser_off();
    if (pca9685_is_ready()) {
        (void)pca9685_all_off();
    }
    g_yaw.energized = false;
    g_pitch.energized = false;
}

void gimbal_task(void)
{
    k230_frame_t frame;
    gimbal_route_sample_t route;
    uint32_t now = millis();
    bool tracking_active;
    bool drive_active;
    int16_t yaw_error_cdeg;
    int16_t pitch_error_cdeg;
    uint8_t writes_left;
#if BOARD_GIMBAL_LOCAL_FORCE_TEST
    static uint8_t force_step_index;
    static uint32_t force_next_step_ms;
#endif

    if (!g_active) {
        return;
    }

    if (!g_pca_ready) {
        if ((int32_t)(now - g_next_pca_retry_ms) >= 0) {
            g_pca_ready = pca9685_init();
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
        }
        return;
    }

#if BOARD_GIMBAL_LOCAL_FORCE_TEST
    if ((int32_t)(now - force_next_step_ms) < 0) {
        return;
    }
    force_step_index = (uint8_t)((force_step_index + 1U) & 0x07U);
    if (!pca9685_set_motor4(GIMBAL_YAW_BASE_CH, k_step_seq[force_step_index])) {
        g_pca_ready = false;
        g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
    }
    force_next_step_ms = now + BOARD_GIMBAL_LOCAL_FORCE_STEP_MS;
    return;
#else
    tracking_active = k230_uart_get_fresh(&frame, BOARD_K230_VISION_TIMEOUT_MS) &&
        frame.valid;
#endif
    route = gimbal_route_get_sample(g_yaw.position_cdeg, g_pitch.position_cdeg);
    drive_active = tracking_active || route.active;

    if (!drive_active) {
#if BOARD_GIMBAL_RELEASE_COILS_IDLE
        axis_release(&g_yaw);
        axis_release(&g_pitch);
#endif
        return;
    }

    if (route.active) {
        yaw_error_cdeg = route.yaw_error_cdeg;
        pitch_error_cdeg = route.pitch_error_cdeg;
        if (tracking_active) {
            yaw_error_cdeg = clamp_i16((int32_t)yaw_error_cdeg +
                    clamp_i16(frame.yaw_cdeg, -BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG,
                        BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG),
                -BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG,
                BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG);
            pitch_error_cdeg = clamp_i16((int32_t)pitch_error_cdeg +
                    clamp_i16(frame.pitch_cdeg, -BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG,
                        BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG),
                -BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG,
                BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG);
        }
    } else {
        yaw_error_cdeg = frame.yaw_cdeg;
        pitch_error_cdeg = frame.pitch_cdeg;
    }

    writes_left = BOARD_GIMBAL_I2C_WRITES_PER_TASK;
    if (writes_left > 0U && axis_update(&g_yaw, yaw_error_cdeg, true, now)) {
        writes_left--;
        now = millis();
    }
    if (writes_left > 0U) {
        (void)axis_update(&g_pitch, pitch_error_cdeg, true, now);
    }
}

void gimbal_sweep_test_task(void)
{
    uint32_t now = millis();
    uint32_t phase;
    uint8_t step;
    static uint8_t last_phase = 0xFFU;
    static uint8_t last_step = 0xFFU;
    static const uint8_t off_pattern[4] = {0, 0, 0, 0};
    static const uint8_t dual_phase[4][4] = {
        {1, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 1},
        {1, 0, 0, 1},
    };

    if (!g_active) {
        last_phase = 0xFFU;
        last_step = 0xFFU;
        return;
    }

    if (!g_pca_ready) {
        if ((int32_t)(now - g_next_pca_retry_ms) >= 0) {
            g_pca_ready = pca9685_init();
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
        }
        return;
    }

    phase = (now / 6000U) & 0x03U;
    step = (uint8_t)((now / GIMBAL_SWEEP_STEP_MS) & 0x03U);
    if ((phase & 0x01U) != 0U) {
        step = (uint8_t)((4U - step) & 0x03U);
    }

    if ((last_phase == (uint8_t)phase) && (last_step == step)) {
        return;
    }

    if (phase == 0U) {
        if (!pca9685_set_motor4(GIMBAL_YAW_BASE_CH, dual_phase[step]) ||
            !pca9685_set_motor4(GIMBAL_PITCH_BASE_CH, off_pattern)) {
            g_pca_ready = false;
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
            return;
        }
    } else if (phase == 1U) {
        if (!pca9685_set_motor4(GIMBAL_YAW_BASE_CH, dual_phase[step]) ||
            !pca9685_set_motor4(GIMBAL_PITCH_BASE_CH, off_pattern)) {
            g_pca_ready = false;
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
            return;
        }
    } else if (phase == 2U) {
        if (!pca9685_set_motor4(GIMBAL_YAW_BASE_CH, off_pattern) ||
            !pca9685_set_motor4(GIMBAL_PITCH_BASE_CH, dual_phase[step])) {
            g_pca_ready = false;
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
            return;
        }
    } else {
        if (!pca9685_set_motor4(GIMBAL_YAW_BASE_CH, off_pattern) ||
            !pca9685_set_motor4(GIMBAL_PITCH_BASE_CH, dual_phase[step])) {
            g_pca_ready = false;
            g_next_pca_retry_ms = now + BOARD_GIMBAL_PCA_RETRY_MS;
            return;
        }
    }

    last_phase = (uint8_t)phase;
    last_step = step;
}

bool gimbal_is_active(void)
{
    return g_active;
}

void gimbal_zero_pose(void)
{
    g_yaw.position_cdeg = 0;
    g_pitch.position_cdeg = 0;
    g_yaw.step_count = 0;
    g_pitch.step_count = 0;
}

gimbal_pose_t gimbal_get_pose(void)
{
    gimbal_pose_t pose;

    pose.yaw_cdeg = g_yaw.position_cdeg;
    pose.pitch_cdeg = g_pitch.position_cdeg;
    return pose;
}

gimbal_steps_t gimbal_get_steps(void)
{
    gimbal_steps_t steps;

    steps.yaw_steps = g_yaw.step_count;
    steps.pitch_steps = g_pitch.step_count;
    return steps;
}

bool gimbal_drive_to_cdeg(int16_t yaw_target_cdeg, int16_t pitch_target_cdeg,
    uint16_t yaw_tolerance_cdeg, uint16_t pitch_tolerance_cdeg,
    uint16_t step_interval_ms, uint8_t writes_per_task)
{
    uint32_t now = millis();
    int16_t yaw_error;
    int16_t pitch_error;
    bool yaw_done;
    bool pitch_done;
    uint8_t writes_left = writes_per_task;

    if (!gimbal_prepare_drive(now)) {
        return false;
    }

    yaw_target_cdeg = clamp_i16(yaw_target_cdeg, -BOARD_GIMBAL_POSITION_LIMIT_CDEG,
        BOARD_GIMBAL_POSITION_LIMIT_CDEG);
    pitch_target_cdeg = clamp_i16(pitch_target_cdeg, -BOARD_GIMBAL_POSITION_LIMIT_CDEG,
        BOARD_GIMBAL_POSITION_LIMIT_CDEG);
    yaw_error = clamp_i16((int32_t)yaw_target_cdeg - g_yaw.position_cdeg,
        -BOARD_GIMBAL_POSITION_LIMIT_CDEG, BOARD_GIMBAL_POSITION_LIMIT_CDEG);
    pitch_error = clamp_i16((int32_t)pitch_target_cdeg - g_pitch.position_cdeg,
        -BOARD_GIMBAL_POSITION_LIMIT_CDEG, BOARD_GIMBAL_POSITION_LIMIT_CDEG);
    yaw_done = abs_i16(yaw_error) <= yaw_tolerance_cdeg;
    pitch_done = abs_i16(pitch_error) <= pitch_tolerance_cdeg;

    if (yaw_done && pitch_done) {
        return true;
    }

    if (!yaw_done && writes_left > 0U &&
        axis_update_fixed(&g_yaw, yaw_error, yaw_tolerance_cdeg,
            step_interval_ms, now)) {
        writes_left--;
        now = millis();
    }
    if (!pitch_done && writes_left > 0U) {
        (void)axis_update_fixed(&g_pitch, pitch_error, pitch_tolerance_cdeg,
            step_interval_ms, now);
    }

    return false;
}

bool gimbal_drive_error_cdeg(int16_t yaw_error_cdeg, int16_t pitch_error_cdeg,
    uint16_t deadband_cdeg, uint16_t step_interval_ms, uint8_t writes_per_task)
{
    uint32_t now = millis();
    bool yaw_done = abs_i16(yaw_error_cdeg) <= deadband_cdeg;
    bool pitch_done = abs_i16(pitch_error_cdeg) <= deadband_cdeg;
    uint8_t writes_left = writes_per_task;

    if (!gimbal_prepare_drive(now)) {
        return false;
    }

    if (yaw_done && pitch_done) {
        return true;
    }

    if (!yaw_done && writes_left > 0U &&
        axis_update_fixed(&g_yaw, yaw_error_cdeg, deadband_cdeg,
            step_interval_ms, now)) {
        writes_left--;
        now = millis();
    }
    if (!pitch_done && writes_left > 0U) {
        (void)axis_update_fixed(&g_pitch, pitch_error_cdeg, deadband_cdeg,
            step_interval_ms, now);
    }

    return false;
}
