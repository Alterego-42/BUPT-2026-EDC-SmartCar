#include "pca9685.h"
#include "board_pins.h"
#include "i2c_bus.h"

#define PCA9685_ADDR (0x40U)
#define PCA_MODE1 (0x00U)
#define PCA_PRESCALE (0xFEU)
#define PCA_LED0_ON_L (0x06U)

static uint16_t clamp_pulse(uint16_t pulse_us)
{
    if (pulse_us < 500U) {
        return 500U;
    }
    if (pulse_us > 2500U) {
        return 2500U;
    }
    return pulse_us;
}

void pca9685_init(void)
{
    (void) i2c_bus_write_reg(PCA9685_ADDR, PCA_MODE1, 0x10U);
    (void) i2c_bus_write_reg(PCA9685_ADDR, PCA_PRESCALE, 121U);
    (void) i2c_bus_write_reg(PCA9685_ADDR, PCA_MODE1, 0x20U);
    delay_cycles(CPUCLK_FREQ / 100U);
    (void) pca9685_set_servo_angle(0U, 90U);
    (void) pca9685_set_servo_angle(1U, 90U);
}

bool pca9685_set_servo_angle(uint8_t channel, uint8_t angle_deg)
{
    if (channel > 15U) {
        return false;
    }
    if (angle_deg > 180U) {
        angle_deg = 180U;
    }

    uint16_t pulse = clamp_pulse((uint16_t) (500U + ((uint32_t) angle_deg * 2000U) / 180U));
    uint16_t off = (uint16_t) (((uint32_t) pulse * 4096U) / 20000U);
    uint8_t reg = (uint8_t) (PCA_LED0_ON_L + 4U * channel);
    uint8_t data[5] = {
        reg, 0x00U, 0x00U, (uint8_t) (off & 0xFFU), (uint8_t) (off >> 8)};
    return i2c_bus_write(PCA9685_ADDR, data, 5U);
}

void laser_init(void)
{
    DL_GPIO_initDigitalOutput(BOARD_LASER_SW_IOMUX);
    DL_GPIO_clearPins(BOARD_LASER_SW_PORT, BOARD_LASER_SW_PIN);
    DL_GPIO_enableOutput(BOARD_LASER_SW_PORT, BOARD_LASER_SW_PIN);
}

void laser_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BOARD_LASER_SW_PORT, BOARD_LASER_SW_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_LASER_SW_PORT, BOARD_LASER_SW_PIN);
    }
}
