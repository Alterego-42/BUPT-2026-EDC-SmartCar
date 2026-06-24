#include "line_controller.h"
#include "board_pins.h"

static int16_t g_last_error;

static int32_t clamp_motor(int32_t value)
{
    if (value > BOARD_MOTOR_MAX_PERCENT) {
        return BOARD_MOTOR_MAX_PERCENT;
    }
    if (value < -BOARD_MOTOR_MAX_PERCENT) {
        return -BOARD_MOTOR_MAX_PERCENT;
    }
    return value;
}

static line_output_t make_output(int32_t left, int32_t right, line_mode_t mode)
{
    line_output_t out;
    out.left_percent = clamp_motor(left);
    out.right_percent = clamp_motor(right);
    out.mode = mode;
    return out;
}

void line_controller_reset(void)
{
    g_last_error = 0;
}

line_output_t line_controller_gray(const grayscale_sample_t *sample)
{
    int16_t error = sample->position;
    int16_t derivative = error - g_last_error;
    g_last_error = error;

    int32_t correction =
        ((int32_t) BOARD_LINE_KP * error + (int32_t) BOARD_LINE_KD * derivative) /
        1000;
    int32_t base = BOARD_MOTOR_START_PERCENT;

    return make_output(base + correction, base - correction, LINE_MODE_GRAY);
}

line_output_t line_controller_heading_hold(int16_t yaw_error_x10)
{
    int32_t correction = ((int32_t) BOARD_HEADING_KP * yaw_error_x10) / 100;
    int32_t base = BOARD_MOTOR_START_PERCENT;

    return make_output(base + correction, base - correction, LINE_MODE_IMU_HOLD);
}

line_output_t line_controller_search(int16_t last_error)
{
    int32_t turn = (last_error >= 0) ? 8 : -8;
    return make_output(turn, -turn, LINE_MODE_SEARCH);
}

line_output_t line_controller_stop(void)
{
    return make_output(0, 0, LINE_MODE_STOP);
}

int16_t line_controller_last_error(void)
{
    return g_last_error;
}
