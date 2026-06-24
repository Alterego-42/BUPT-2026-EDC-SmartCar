#include "i2c_bus.h"
#include "board_pins.h"

static void i2c_delay(void)
{
    delay_cycles(CPUCLK_FREQ / 2000000U);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(BOARD_OLED_SDA_PORT, BOARD_OLED_SDA_PIN);
    DL_GPIO_enableOutput(BOARD_OLED_SDA_PORT, BOARD_OLED_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(BOARD_OLED_SDA_PORT, BOARD_OLED_SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(BOARD_OLED_SCL_PORT, BOARD_OLED_SCL_PIN);
    DL_GPIO_enableOutput(BOARD_OLED_SCL_PORT, BOARD_OLED_SCL_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(BOARD_OLED_SCL_PORT, BOARD_OLED_SCL_PIN);
}

static bool sda_read(void)
{
    return (DL_GPIO_readPins(BOARD_OLED_SDA_PORT, BOARD_OLED_SDA_PIN) != 0U);
}

static void i2c_start(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_release();
    i2c_delay();
    sda_release();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t value)
{
    for (uint8_t bit = 0; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        scl_release();
        i2c_delay();
        scl_low();
        value <<= 1;
    }

    sda_release();
    i2c_delay();
    scl_release();
    i2c_delay();
    bool ack = !sda_read();
    scl_low();
    return ack;
}

void i2c_bus_init(void)
{
    DL_GPIO_initDigitalInputFeatures(BOARD_OLED_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_OLED_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    sda_release();
    scl_release();
}

bool i2c_bus_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    i2c_start();
    bool ok = i2c_write_byte((uint8_t) (addr7 << 1));
    for (uint16_t i = 0; (i < len) && ok; i++) {
        ok = i2c_write_byte(data[i]);
    }
    i2c_stop();
    return ok;
}

bool i2c_bus_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_bus_write(addr7, data, 2U);
}
