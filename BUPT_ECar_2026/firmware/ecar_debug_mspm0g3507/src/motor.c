#include "motor.h"

#include "board_config.h"
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "util.h"

static uint16_t percent_to_counts(int16_t percent)
{
    int32_t duty = abs_i32(percent);
    duty = clamp_i32(duty, 0, 100);
    return (uint16_t)(((int32_t) BOARD_PWM_PERIOD_COUNTS * duty) / 100);
}

static void pwm_write(uint32_t channel, int16_t percent)
{
    uint16_t duty_counts = percent_to_counts(percent);
    uint16_t compare = (uint16_t)(BOARD_PWM_PERIOD_COUNTS - duty_counts);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, compare, channel);
}

static void set_left_dir(int16_t percent)
{
    bool forward = (percent >= 0);
    if (BOARD_MOTOR_LEFT_INVERT) {
        forward = !forward;
    }

    if (percent == 0) {
        DL_GPIO_clearPins(GPIO_MOTOR_L_IN1_PORT, GPIO_MOTOR_L_IN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_L_IN2_PORT, GPIO_MOTOR_L_IN2_PIN);
    } else if (forward) {
        DL_GPIO_setPins(GPIO_MOTOR_L_IN1_PORT, GPIO_MOTOR_L_IN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_L_IN2_PORT, GPIO_MOTOR_L_IN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_L_IN1_PORT, GPIO_MOTOR_L_IN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_L_IN2_PORT, GPIO_MOTOR_L_IN2_PIN);
    }
}

static void set_right_dir(int16_t percent)
{
    bool forward = (percent >= 0);
    if (BOARD_MOTOR_RIGHT_INVERT) {
        forward = !forward;
    }

    if (percent == 0) {
        DL_GPIO_clearPins(GPIO_MOTOR_R_IN1_PORT, GPIO_MOTOR_R_IN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_R_IN2_PORT, GPIO_MOTOR_R_IN2_PIN);
    } else if (forward) {
        DL_GPIO_setPins(GPIO_MOTOR_R_IN1_PORT, GPIO_MOTOR_R_IN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_R_IN2_PORT, GPIO_MOTOR_R_IN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_R_IN1_PORT, GPIO_MOTOR_R_IN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_R_IN2_PORT, GPIO_MOTOR_R_IN2_PIN);
    }
}

void motor_init(void)
{
    motor_stop();
    DL_TimerG_startCounter(PWM_MOTOR_INST);
}

void motor_standby(uint8_t enable)
{
    if (enable) {
        DL_GPIO_setPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
    }
}

void motor_stop(void)
{
    pwm_write(GPIO_PWM_MOTOR_C0_IDX, 0);
    pwm_write(GPIO_PWM_MOTOR_C1_IDX, 0);
    set_left_dir(0);
    set_right_dir(0);
    motor_standby(0U);
}

void motor_brake(void)
{
    pwm_write(GPIO_PWM_MOTOR_C0_IDX, 0);
    pwm_write(GPIO_PWM_MOTOR_C1_IDX, 0);
    DL_GPIO_setPins(GPIO_MOTOR_L_IN1_PORT, GPIO_MOTOR_L_IN1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_L_IN2_PORT, GPIO_MOTOR_L_IN2_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_R_IN1_PORT, GPIO_MOTOR_R_IN1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_R_IN2_PORT, GPIO_MOTOR_R_IN2_PIN);
    motor_standby(1U);
}

void motor_set_percent(int16_t left_percent, int16_t right_percent)
{
    left_percent = clamp_i16(left_percent, -BOARD_MOTOR_MAX_PERCENT, BOARD_MOTOR_MAX_PERCENT);
    right_percent = clamp_i16(right_percent, -BOARD_MOTOR_MAX_PERCENT, BOARD_MOTOR_MAX_PERCENT);

    motor_standby((left_percent != 0 || right_percent != 0) ? 1U : 0U);
    set_left_dir(left_percent);
    set_right_dir(right_percent);
    pwm_write(GPIO_PWM_MOTOR_C0_IDX, left_percent);
    pwm_write(GPIO_PWM_MOTOR_C1_IDX, right_percent);
}

void motor_point_left(int16_t percent)
{
    motor_set_percent(percent, 0);
}

void motor_point_right(int16_t percent)
{
    motor_set_percent(0, percent);
}
