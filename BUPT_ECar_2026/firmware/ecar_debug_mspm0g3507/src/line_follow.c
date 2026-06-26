#include "line_follow.h"

#include "board_config.h"
#include "imu.h"
#include "motor.h"
#include "timebase.h"
#include "util.h"

static int16_t g_last_error;
static uint32_t g_last_seen_ms;
static int32_t g_last_seen_distance_mm;
static uint32_t g_last_update_ms;
static int16_t g_hold_yaw_x10;
static int16_t g_last_line_correction;
static uint32_t g_bridge_enter_ms;
static bool g_bridge_active;
static line_follow_status_t g_last_status;

static int16_t wrap_heading_x10(int16_t yaw_x10)
{
    while (yaw_x10 > 1800) {
        yaw_x10 = (int16_t)(yaw_x10 - 3600);
    }
    while (yaw_x10 < -1800) {
        yaw_x10 = (int16_t)(yaw_x10 + 3600);
    }
    return yaw_x10;
}

static void capture_bridge_heading(void)
{
    g_hold_yaw_x10 = wrap_heading_x10((int16_t)(imu_get_yaw_x10() + BOARD_BRIDGE_HEADING_OFFSET_X10));
}

void line_follow_reset_at(int32_t distance_mm)
{
    line_follow_status_t clear_status = {0};
    g_last_error = 0;
    g_last_seen_ms = millis();
    g_last_seen_distance_mm = distance_mm;
    g_last_update_ms = 0;
    g_last_line_correction = 0;
    g_bridge_enter_ms = 0;
    g_bridge_active = false;
    capture_bridge_heading();
    g_last_status = clear_status;
}

void line_follow_reset(void)
{
    line_follow_reset_at(0);
}

void line_follow_stop(void)
{
#if BOARD_MOTOR_BRAKE_ON_STOP
    motor_brake();
#else
    motor_stop();
#endif
}

line_follow_status_t line_follow_update_ex(uint32_t now_ms, int32_t distance_mm, bool bridge_gap)
{
    line_follow_status_t status = {0};
    if ((uint32_t)(now_ms - g_last_update_ms) < BOARD_LINE_CONTROL_PERIOD_MS) {
        status = g_last_status;
        status.active = true;
        status.distance_mm = distance_mm;
        return status;
    }
    g_last_update_ms = now_ms;

    status.gray = grayscale_read();
    status.distance_mm = distance_mm;
    status.active = true;

    int16_t correction = 0;
    int16_t error = g_last_error;
    bool line_seen_this_update = status.gray.line_seen;

    if (status.gray.line_seen) {
        error = status.gray.error;
        g_last_seen_ms = now_ms;
        g_last_seen_distance_mm = distance_mm;
        g_bridge_active = false;
        capture_bridge_heading();
        int16_t derivative = (int16_t)(error - g_last_error);
        correction = (int16_t)(((int32_t)error * BOARD_LINE_KP_PER_MILLE +
            (int32_t)derivative * BOARD_LINE_KD_PER_MILLE) / 1000L);
        g_last_error = error;
    } else {
        uint32_t lost_ms = now_ms - g_last_seen_ms;
        int32_t lost_mm = distance_mm - g_last_seen_distance_mm;
        status.lost = true;
        if (!g_bridge_active) {
            g_bridge_active = true;
            g_bridge_enter_ms = now_ms;
        }

        uint32_t max_lost_ms = bridge_gap ? BOARD_LINE_BRIDGE_MAX_MS : BOARD_LINE_LOST_STOP_MS;
        int32_t max_lost_mm = bridge_gap ? BOARD_LINE_BRIDGE_MAX_MM : 0;
        if (lost_ms > max_lost_ms || (bridge_gap && lost_mm > max_lost_mm)) {
            status.fault = true;
            line_follow_stop();
            g_last_status = status;
            return status;
        }

        if (BOARD_IMU_ENABLE_HEADING_HOLD && (bridge_gap || lost_ms <= BOARD_LINE_LOST_HOLD_MS) && imu_yaw_valid()) {
            int16_t heading_error = imu_yaw_error_x10(g_hold_yaw_x10);
            correction = (int16_t)(((int32_t)heading_error * BOARD_HEADING_KP_PER_DEG) / 10L);
            if (bridge_gap && (uint32_t)(now_ms - g_bridge_enter_ms) <= BOARD_LINE_BRIDGE_KEEP_LAST_MS) {
                correction = (int16_t)(correction + clamp_i16(g_last_line_correction,
                    -BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT, BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT));
            }
        } else if (bridge_gap) {
            if ((uint32_t)(now_ms - g_bridge_enter_ms) <= BOARD_LINE_BRIDGE_KEEP_LAST_MS) {
                correction = clamp_i16(g_last_line_correction,
                    -BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT, BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT);
            } else {
                correction = 0;
            }
        } else {
            correction = (g_last_error >= 0) ? BOARD_MOTOR_SEARCH_PERCENT : -BOARD_MOTOR_SEARCH_PERCENT;
        }
    }

    if (BOARD_STEERING_INVERT) {
        correction = (int16_t)-correction;
    }

    correction = clamp_i16(correction, -BOARD_MOTOR_MAX_PERCENT, BOARD_MOTOR_MAX_PERCENT);
    if (line_seen_this_update) {
        g_last_line_correction = correction;
    }
    int16_t base = BOARD_MOTOR_START_PERCENT;
    int16_t left = clamp_i16((int32_t)base + correction + BOARD_STRAIGHT_TRIM_PERCENT,
        -BOARD_MOTOR_MAX_PERCENT, BOARD_MOTOR_MAX_PERCENT);
    int16_t right = clamp_i16((int32_t)base - correction - BOARD_STRAIGHT_TRIM_PERCENT,
        -BOARD_MOTOR_MAX_PERCENT, BOARD_MOTOR_MAX_PERCENT);

    status.left_percent = left;
    status.right_percent = right;
    status.correction_percent = correction;
    motor_set_percent(left, right);
    g_last_status = status;
    return status;
}

line_follow_status_t line_follow_update(uint32_t now_ms, int32_t distance_mm)
{
    return line_follow_update_ex(now_ms, distance_mm, false);
}
