#include "motor.h"
#include "board_pins.h"

static int32_t clamp_percent(int32_t value)
{
    if (value > BOARD_MOTOR_MAX_PERCENT) {
        return BOARD_MOTOR_MAX_PERCENT;
    }
    if (value < -BOARD_MOTOR_MAX_PERCENT) {
        return -BOARD_MOTOR_MAX_PERCENT;
    }
    return value;
}

static uint32_t compare_from_percent(uint32_t duty_percent)
{
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    return BOARD_PWM_PERIOD_COUNTS -
           ((BOARD_PWM_PERIOD_COUNTS * duty_percent) / 100U);
}

static void set_pwm(uint32_t cc_index, uint32_t duty_percent)
{
    DL_TimerG_setCaptureCompareValue(
        BOARD_PWM_INST, compare_from_percent(duty_percent), cc_index);
}

static void set_side(GPIO_Regs *in1_port, uint32_t in1_pin, GPIO_Regs *in2_port,
    uint32_t in2_pin, uint32_t cc_index, int32_t percent, int invert)
{
    int32_t cmd = clamp_percent(percent);
    if (invert) {
        cmd = -cmd;
    }

    if (cmd > 0) {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
        set_pwm(cc_index, (uint32_t) cmd);
    } else if (cmd < 0) {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_setPins(in2_port, in2_pin);
        set_pwm(cc_index, (uint32_t) (-cmd));
    } else {
        set_pwm(cc_index, 0U);
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
}

void motor_init(void)
{
    DL_GPIO_initDigitalOutput(BOARD_L_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_L_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_R_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_R_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_MOTOR_STBY_IOMUX);

    DL_GPIO_clearPins(BOARD_L_IN1_PORT, BOARD_L_IN1_PIN | BOARD_L_IN2_PIN);
    DL_GPIO_clearPins(BOARD_R_IN1_PORT, BOARD_R_IN1_PIN | BOARD_R_IN2_PIN);
    DL_GPIO_clearPins(BOARD_MOTOR_STBY_PORT, BOARD_MOTOR_STBY_PIN);

    DL_GPIO_enableOutput(BOARD_L_IN1_PORT, BOARD_L_IN1_PIN | BOARD_L_IN2_PIN);
    DL_GPIO_enableOutput(BOARD_R_IN1_PORT, BOARD_R_IN1_PIN | BOARD_R_IN2_PIN);
    DL_GPIO_enableOutput(BOARD_MOTOR_STBY_PORT, BOARD_MOTOR_STBY_PIN);

    set_pwm(BOARD_PWM_LEFT_CC_INDEX, 0U);
    set_pwm(BOARD_PWM_RIGHT_CC_INDEX, 0U);
    DL_TimerG_startCounter(BOARD_PWM_INST);
}

void motor_enable(void)
{
    DL_GPIO_setPins(BOARD_MOTOR_STBY_PORT, BOARD_MOTOR_STBY_PIN);
}

void motor_disable(void)
{
    motor_stop();
    DL_GPIO_clearPins(BOARD_MOTOR_STBY_PORT, BOARD_MOTOR_STBY_PIN);
}

void motor_stop(void)
{
    motor_set_percent(0, 0);
}

void motor_set_percent(int32_t left_percent, int32_t right_percent)
{
    set_side(BOARD_L_IN1_PORT, BOARD_L_IN1_PIN, BOARD_L_IN2_PORT,
        BOARD_L_IN2_PIN, BOARD_PWM_LEFT_CC_INDEX, left_percent,
        BOARD_MOTOR_LEFT_INVERT);
    set_side(BOARD_R_IN1_PORT, BOARD_R_IN1_PIN, BOARD_R_IN2_PORT,
        BOARD_R_IN2_PIN, BOARD_PWM_RIGHT_CC_INDEX, right_percent,
        BOARD_MOTOR_RIGHT_INVERT);
}
