#include "pca9685.h"

#include "board_config.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

#define PCA9685_ADDR_7BIT       (0x40U)
#define PCA9685_MODE1           (0x00U)
#define PCA9685_MODE2           (0x01U)
#define PCA9685_LED0_ON_L       (0x06U)
#define PCA9685_ALL_LED_ON_L    (0xFAU)

#define PCA_I2C_INST            (I2C0)
#define PCA_SDA_IOMUX           (IOMUX_PINCM3)
#define PCA_SCL_IOMUX           (IOMUX_PINCM6)
#define PCA_SDA_FUNC            (IOMUX_PINCM3_PF_I2C0_SDA)
#define PCA_SCL_FUNC            (IOMUX_PINCM6_PF_I2C0_SCL)
#define PCA_I2C_TIMER_400KHZ    (7U)

static bool g_ready;
static uint32_t g_errors;

static bool wait_for_idle(void)
{
    uint32_t start = millis();
    while ((DL_I2C_getControllerStatus(PCA_I2C_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((uint32_t)(millis() - start) > BOARD_PCA9685_I2C_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

static void record_error(void)
{
    g_errors++;
    g_ready = false;
    DL_I2C_resetControllerTransfer(PCA_I2C_INST);
    DL_I2C_flushControllerTXFIFO(PCA_I2C_INST);
}

static bool i2c_write_bytes(const uint8_t *bytes, uint8_t len)
{
    uint16_t sent;
    uint32_t start;

    if (len == 0U) {
        return false;
    }

    if (!wait_for_idle()) {
        record_error();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C_INST);
    sent = DL_I2C_fillControllerTXFIFO(PCA_I2C_INST, bytes, len);

    DL_I2C_startControllerTransfer(
        PCA_I2C_INST, PCA9685_ADDR_7BIT, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    start = millis();
    while (sent < len) {
        uint16_t added;

        if ((DL_I2C_getControllerStatus(PCA_I2C_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            record_error();
            return false;
        }

        added = DL_I2C_fillControllerTXFIFO(
            PCA_I2C_INST, &bytes[sent], (uint16_t)(len - sent));
        if (added > 0U) {
            sent = (uint16_t)(sent + added);
            start = millis();
            continue;
        }

        if ((uint32_t)(millis() - start) > BOARD_PCA9685_I2C_TIMEOUT_MS) {
            record_error();
            return false;
        }
    }

    if (!wait_for_idle() ||
        (DL_I2C_getControllerStatus(PCA_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        record_error();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C_INST);
    return true;
}

static bool write_reg_data(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buffer[17];

    if (len > 16U) {
        return false;
    }

    buffer[0] = reg;
    for (uint8_t i = 0; i < len; i++) {
        buffer[i + 1U] = data[i];
    }

    return i2c_write_bytes(buffer, (uint8_t)(len + 1U));
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    return write_reg_data(reg, &value, 1U);
}

static void i2c0_init(void)
{
    static const DL_I2C_ClockConfig clock_config = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    DL_I2C_reset(PCA_I2C_INST);
    DL_I2C_enablePower(PCA_I2C_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunctionFeatures(PCA_SDA_IOMUX,
        PCA_SDA_FUNC, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(PCA_SCL_IOMUX,
        PCA_SCL_FUNC, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(PCA_SDA_IOMUX);
    DL_GPIO_enableHiZ(PCA_SCL_IOMUX);

    DL_I2C_setClockConfig(PCA_I2C_INST, (DL_I2C_ClockConfig *)&clock_config);
    DL_I2C_disableAnalogGlitchFilter(PCA_I2C_INST);
    DL_I2C_resetControllerTransfer(PCA_I2C_INST);
    DL_I2C_setTimerPeriod(PCA_I2C_INST, PCA_I2C_TIMER_400KHZ);
    DL_I2C_setControllerTXFIFOThreshold(PCA_I2C_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(PCA_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(PCA_I2C_INST);
    DL_I2C_enableController(PCA_I2C_INST);
}

bool pca9685_init(void)
{
    i2c0_init();
    g_ready = false;

    if (!write_reg(PCA9685_MODE1, 0x00U)) {
        return false;
    }
    delay_ms(2U);
    if (!write_reg(PCA9685_MODE2, 0x04U)) {
        return false;
    }
    if (!write_reg(PCA9685_MODE1, 0x20U)) {
        return false;
    }

    g_ready = true;
    return pca9685_all_off();
}

bool pca9685_is_ready(void)
{
    return g_ready;
}

bool pca9685_all_off(void)
{
    const uint8_t data[4] = {0x00U, 0x00U, 0x00U, 0x10U};
    bool ok = write_reg_data(PCA9685_ALL_LED_ON_L, data, sizeof(data));
    g_ready = ok;
    return ok;
}

bool pca9685_set_motor4(uint8_t base_ch, const uint8_t pattern[4])
{
    uint8_t data[16];

    if (!g_ready || pattern == 0 || base_ch > 12U) {
        return false;
    }

    for (uint8_t i = 0; i < 4U; i++) {
        uint8_t idx = (uint8_t)(i * 4U);

        if (pattern[i] != 0U) {
            data[idx + 0U] = 0x00U;
            data[idx + 1U] = 0x10U;
            data[idx + 2U] = 0x00U;
            data[idx + 3U] = 0x00U;
        } else {
            data[idx + 0U] = 0x00U;
            data[idx + 1U] = 0x00U;
            data[idx + 2U] = 0x00U;
            data[idx + 3U] = 0x10U;
        }
    }

    if (!write_reg_data((uint8_t)(PCA9685_LED0_ON_L + (base_ch * 4U)),
            data, sizeof(data))) {
        g_ready = false;
        return false;
    }

    g_ready = true;
    return true;
}

uint32_t pca9685_error_count(void)
{
    return g_errors;
}
