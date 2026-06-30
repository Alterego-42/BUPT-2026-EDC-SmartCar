#include "gimbal.h"

#include <stdint.h>

#include "board_config.h"
#include "k230_uart.h"
#include "laser.h"
#include "pca9685.h"
#include "timebase.h"

#define GIMBAL_YAW_BASE_CH      (0U)
#define GIMBAL_PITCH_BASE_CH    (4U)
#define GIMBAL_SWEEP_STEP_MS    (8U)

typedef struct {
    uint8_t base_ch;
    uint8_t step_index;
    bool energized;
    int8_t direction_sign;
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
    uint16_t deadband_cdeg, uint16_t max_error_cdeg, uint16_t min_step_ms,
    uint16_t max_step_ms)
{
    axis->base_ch = base_ch;
    axis->step_index = 0;
    axis->energized = false;
    axis->direction_sign = direction_sign;
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
    } else {
        g_pca_ready = false;
        g_next_pca_retry_ms = now_ms + BOARD_GIMBAL_PCA_RETRY_MS;
        axis->next_step_ms = now_ms + interval;
        return true;
    }

    axis->next_step_ms = now_ms + interval;
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
        BOARD_GIMBAL_YAW_DEADBAND_CDEG, BOARD_GIMBAL_YAW_MAX_ERROR_CDEG,
        BOARD_GIMBAL_YAW_STEP_MIN_MS, BOARD_GIMBAL_YAW_STEP_MAX_MS);
    axis_config(&g_pitch, GIMBAL_PITCH_BASE_CH, BOARD_GIMBAL_PITCH_DIR,
        BOARD_GIMBAL_PITCH_DEADBAND_CDEG, BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG,
        BOARD_GIMBAL_PITCH_STEP_MIN_MS, BOARD_GIMBAL_PITCH_STEP_MAX_MS);
}

void gimbal_start(void)
{
    uint32_t now = millis();

    axis_config(&g_yaw, GIMBAL_YAW_BASE_CH, BOARD_GIMBAL_YAW_DIR,
        BOARD_GIMBAL_YAW_DEADBAND_CDEG, BOARD_GIMBAL_YAW_MAX_ERROR_CDEG,
        BOARD_GIMBAL_YAW_STEP_MIN_MS, BOARD_GIMBAL_YAW_STEP_MAX_MS);
    axis_config(&g_pitch, GIMBAL_PITCH_BASE_CH, BOARD_GIMBAL_PITCH_DIR,
        BOARD_GIMBAL_PITCH_DEADBAND_CDEG, BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG,
        BOARD_GIMBAL_PITCH_STEP_MIN_MS, BOARD_GIMBAL_PITCH_STEP_MAX_MS);

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
    uint32_t now = millis();
    bool tracking_active;
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

    if (!tracking_active) {
#if BOARD_GIMBAL_RELEASE_COILS_IDLE
        axis_release(&g_yaw);
        axis_release(&g_pitch);
#endif
        return;
    }

    writes_left = BOARD_GIMBAL_I2C_WRITES_PER_TASK;
    if (writes_left > 0U && axis_update(&g_yaw, frame.yaw_cdeg, true, now)) {
        writes_left--;
        now = millis();
    }
    if (writes_left > 0U) {
        (void)axis_update(&g_pitch, frame.pitch_cdeg, true, now);
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
