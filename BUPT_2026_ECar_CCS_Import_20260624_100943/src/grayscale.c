#include "grayscale.h"
#include "board_pins.h"

static const int16_t SENSOR_WEIGHT[8] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

static void delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

static void set_addr(uint8_t index)
{
    if ((index & 0x01U) != 0U) {
        DL_GPIO_setPins(BOARD_GRAY_AD0_PORT, BOARD_GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_GRAY_AD0_PORT, BOARD_GRAY_AD0_PIN);
    }

    if ((index & 0x02U) != 0U) {
        DL_GPIO_setPins(BOARD_GRAY_AD1_PORT, BOARD_GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_GRAY_AD1_PORT, BOARD_GRAY_AD1_PIN);
    }

    if ((index & 0x04U) != 0U) {
        DL_GPIO_setPins(BOARD_GRAY_AD2_PORT, BOARD_GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_GRAY_AD2_PORT, BOARD_GRAY_AD2_PIN);
    }
}

void grayscale_init(void)
{
    DL_GPIO_initDigitalOutput(BOARD_GRAY_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_GRAY_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_GRAY_AD2_IOMUX);
    DL_GPIO_initDigitalInputFeatures(BOARD_GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(BOARD_GRAY_AD0_PORT, BOARD_GRAY_AD0_PIN);
    DL_GPIO_clearPins(BOARD_GRAY_AD1_PORT, BOARD_GRAY_AD1_PIN);
    DL_GPIO_clearPins(BOARD_GRAY_AD2_PORT, BOARD_GRAY_AD2_PIN);
    DL_GPIO_enableOutput(BOARD_GRAY_AD0_PORT, BOARD_GRAY_AD0_PIN);
    DL_GPIO_enableOutput(BOARD_GRAY_AD1_PORT, BOARD_GRAY_AD1_PIN);
    DL_GPIO_enableOutput(BOARD_GRAY_AD2_PORT, BOARD_GRAY_AD2_PIN);
}

grayscale_sample_t grayscale_read(void)
{
    grayscale_sample_t sample = {0};
    int32_t weighted_sum = 0;

    for (uint8_t i = 0; i < 8U; i++) {
        set_addr(i);
        delay_us(BOARD_GRAY_SETTLE_US);

        bool raw_high =
            (DL_GPIO_readPins(BOARD_GRAY_OUT_PORT, BOARD_GRAY_OUT_PIN) != 0U);
        bool black = (BOARD_GRAY_BLACK_IS_HIGH != 0U) ? raw_high : !raw_high;

        if (black) {
            sample.mask |= (uint8_t) (1U << i);
            sample.active_count++;
            weighted_sum += SENSOR_WEIGHT[i];
        }
    }

    if (sample.active_count > 0U) {
        sample.line_seen = true;
        sample.position = (int16_t) (weighted_sum / sample.active_count);
    } else {
        sample.line_seen = false;
        sample.position = 0;
    }

    return sample;
}
